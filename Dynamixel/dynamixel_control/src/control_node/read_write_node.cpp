// terminal #1: ros2 run dynamixel_examples read_write_node
// terminal #2: 
// position control:        ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "data: [0,0,0,0,0,0,0,0]" (radian단위, 정수 입력도 가능)
// torque enable/disable:   ros2 topic pub -1 /command std_msgs/msg/String "data: 'enable'" 
#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream> // Csv 파일 저장
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0
#define PI                              3.1415926

#define DXL_MIN_ID                      1
#define DXL_MAX_ID                      2

// Define BIG_MOTOR and SMALL_MOTOR sets
const std::set<uint8_t> BIG_MOTOR = {1};
const std::set<uint8_t> SMALL_MOTOR = {2};

#define BIG_MIN_POS                     -250961
#define BIG_MAX_POS                     250961
#define SMALL_MIN_POS                   -151875
#define SMALL_MAX_POS                   151875

class DynamixelController : public rclcpp::Node {
public:
  DynamixelController()
    : Node("dxl") {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    param_goal_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0); // 초기화
    dxl_present_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    
    // csv 데이터 파일 초기화
    data_file_.open("motor_data.csv", std::ios::out | std::ios::trunc);
    if (data_file_.is_open()) {
        data_file_ << "Average timer callback time, Average topic callback time,Desired Position Motor #1,Present Position Motor #1,"
                      "Desired Position Motor #2,Present Position Motor #2\n";
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open motor_data.csv for writing");
    }

    // Open port, set port baudrate
    if (!portHandler->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
      return;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to change the baudrate!");
      return;
    }

    // Enable Dynamixel Torque (write1ByteTxRx(포트?, id, 토크켜는 위치, 값, 에러값))
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        packetHandler->getTxRxResult(dxl_comm_result);
      } else if (dxl_error != 0) {
        packetHandler->getRxPacketError(dxl_error);
      } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel#%d has been successfully connected", dxl_id);
      }
    }

    // Initialize GroupSyncWrite instance
    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, 4);

    // Subscribe to the states topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&DynamixelController::topicCallback, this, std::placeholders::_1));

    // Subscribe to the command topic
    command_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "command", 10, std::bind(&DynamixelController::commandCallback, this, std::placeholders::_1));

    // Start the timer - Printing current position of dynamixel every 10ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DynamixelController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dynamixel Controller has been initialized");
  }

  ~DynamixelController() {
    // Disable Dynamixel Torque
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        packetHandler->getTxRxResult(dxl_comm_result);
      } else if (dxl_error != 0) {
        packetHandler->getRxPacketError(dxl_error);
      }
    } 

    // csv 데이터 파일 닫기
    if (data_file_.is_open()) {
        data_file_.close();
        RCLCPP_INFO(this->get_logger(), "Data file motor_data.csv has been saved successfully.");
    }

    // Close port
    portHandler->closePort();
    delete groupSyncWrite;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupSyncWrite *groupSyncWrite;
  uint8_t dxl_error = 0;
  std::ofstream data_file_; // csv 파일 저장
  // int32_t dxl_present_position = 0;
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;
  bool torque_enabled_ = true;
  // csv 저장용
  std::vector<int32_t> dxl_present_position;
  std::vector<int> param_goal_position;
  // 응답속도 확인용
  long long total_time_ = 0;  // 총 시간 누적
  int callback_count_ = 0;    // 콜백 호출 횟수
  long long total_topic_time_ = 0;  // 총 시간 누적
  int topic_callback_count_ = 0;    // 콜백 호출 횟수
  double average_time_topic = 0;
  // int* param_goal_position = new int[num_joints]{0}; // csv 저장을 위한 클래스 변수 승격
  
  // uint8_t joint_sequence[8] = {3,0,1,4,2,9,5,7};  // /joint_states 순서: [235147.8.6] -> [23561478]?
  // uint8_t joint_sequence[8] = {4, 0, 1, 5, 2, 3, 6, 7};
  uint8_t joint_sequence[8] = {1, 2};

  void timerCallback() {
    std::ostringstream log_stream; // 로그를 버퍼에 저장할 스트림
    auto start_time = std::chrono::high_resolution_clock::now(); // 응답 시간 체크
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&dxl_present_position[dxl_id - DXL_MIN_ID]), &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
        } else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
        }

        log_stream << "Motor#" << dxl_id 
                   << ": " << Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[dxl_id - DXL_MIN_ID]));
        if (dxl_id == DXL_MAX_ID) {
            log_stream << "\n";
        } else {
            log_stream << " | ";
        }
    }
    // 현재 각도 출력
    std::cout << log_stream.str() << std::endl;
    
    // 응답 시간 체크 및 출력
    auto end_time = std::chrono::high_resolution_clock::now();

    // 걸린 시간 계산 (마이크로초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // 시간 누적 및 평균 계산
    total_time_ += duration;
    callback_count_++;
    double average_time = total_time_ / static_cast<double>(callback_count_);

    // 디버깅 출력
    RCLCPP_INFO(this->get_logger(), "Timer execution time: %ld microseconds, Average time: %.2f microseconds", duration, average_time);

    // csv 데이터 저장
    if (data_file_.is_open()) {
        auto now = this->get_clock()->now(); // 현재 시간
        data_file_ << average_time << "," // 타임스탬프(초 단위)
                  << average_time_topic << "," // topiccallback
                  << Rad2Deg(Pos2Rad(1, param_goal_position[0])) << ","   // 목표 위치 모터 #1
                  << Rad2Deg(Pos2Rad(1, dxl_present_position[0])) << "," // 현재 위치 모터 #1
                  << Rad2Deg(Pos2Rad(2, param_goal_position[1])) << ","   // 목표 위치 모터 #2
                  << Rad2Deg(Pos2Rad(2, dxl_present_position[1])) << "\n"; // 현재 위치 모터 #2
                  // 종료 시간 기록
    }
    }

  void topicCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    // 응답 시간 체크
    auto start_time = std::chrono::high_resolution_clock::now();

    // 디버깅 로그
    RCLCPP_DEBUG(this->get_logger(), "Received /joint_states message");
    // 수신된 메시지의 각도를 출력
    for (size_t i = 0; i < msg->position.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "Joint %zu: %f rad", i + 1, msg->position[i]);
    }

    // 조인트의 최대 개수에 맞게 배열 크기를 동적으로 설정
    int num_joints = DXL_MAX_ID - DXL_MIN_ID + 1;
    std::vector<float> raw_goal_position(num_joints, 0.0); // 크기를 동적으로 설정
    // int* param_goal_position = new int[num_joints]{0}; // csv 저장을 위한 클래스 변수 승격

    // // Set goal position parameter array from topic messages
    // float raw_goal_position[] = {0.0};
    // int* param_goal_position = new int[DXL_MAX_ID - DXL_MIN_ID + 1]{0};
    for(int i = 0; i <= (DXL_MAX_ID - DXL_MIN_ID); i++){
        raw_goal_position[i] = msg->position[joint_sequence[i]-1];
          // raw_goal_position[i] = msg->position[i];
          // if(i != 2)
          //   raw_goal_position[i] *= -1;
          
        // Change radian value to position with integer value and proceed clipping
        RCLCPP_DEBUG(this->get_logger(), "Joint %i: original goal position = %d, raw_goal position = %f", i+1, param_goal_position[i], raw_goal_position[i]);
        param_goal_position[i] = Clipping(i + DXL_MIN_ID, Rad2Pos(i + DXL_MIN_ID, raw_goal_position[i]));
        RCLCPP_DEBUG(this->get_logger(), "Joint %i: clipped goal position = %d", i+1, param_goal_position[i]);
        // RCLCPP_INFO(this->get_logger(), "clipped goal position of #%d: %d", i + DXL_MIN_ID, param_goal_position[i]);
    }

    // Add Dynamixel goal position value to the Syncwrite parameter storage
    int max_id = DXL_MAX_ID == 3 ? 2 : DXL_MAX_ID;
    for(int dxl_id = DXL_MIN_ID; dxl_id <= max_id; dxl_id++){
      dxl_addparam_result = groupSyncWrite->addParam(dxl_id, reinterpret_cast<uint8_t*>(&param_goal_position[dxl_id - DXL_MIN_ID]));
      RCLCPP_DEBUG(this->get_logger(), "Add parameter complete %d.", dxl_id);
      if (dxl_addparam_result != true) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add Dynamixel#%d goal position to the Syncwrite parameter storage", dxl_id);
        return;
      }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite->clearParam();
    // delete[] param_goal_position;

    // 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // 시간 누적 및 평균 계산
    total_topic_time_ += duration;
    topic_callback_count_++;
    double average_time = total_topic_time_ / static_cast<double>(topic_callback_count_);
    average_time_topic = average_time;

    // 디버깅 출력
    RCLCPP_INFO(this->get_logger(), "topicCallback execution time: %ld microseconds, Average time: %.2f microseconds", duration, average_time);
  }

  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    // 토크 enable / disable 제어
    if (msg->data == "enable") {
      enableTorque();
    } else if (msg->data == "disable") {
      disableTorque();
    } else {
      RCLCPP_WARN(this->get_logger(), "Received unknown command: %s", msg->data.c_str());
    }
  }

  void enableTorque() {
    // Enable Dynamixel Torque
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        packetHandler->getTxRxResult(dxl_comm_result);
      } else if (dxl_error != 0) {
        packetHandler->getRxPacketError(dxl_error);
      } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel#%d torque has been enabled", dxl_id);
      }
    }

    torque_enabled_ = true;
  }

  void disableTorque() {
    // Disable Dynamixel Torque
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        packetHandler->getTxRxResult(dxl_comm_result);
      } else if (dxl_error != 0) {
        packetHandler->getRxPacketError(dxl_error);
      } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel#%d torque has been disabled", dxl_id);
      }
    } 

    torque_enabled_ = false;
  }
  
  int Clipping(uint8_t dxl_id, int goal_position) {
    RCLCPP_DEBUG(this->get_logger(), "Get dxl_id: %i, goal position: %i", dxl_id, goal_position);
      if (BIG_MOTOR.find(dxl_id) != BIG_MOTOR.end()) {
          goal_position = goal_position > BIG_MAX_POS ? BIG_MAX_POS : goal_position;
          goal_position = goal_position < BIG_MIN_POS ? BIG_MIN_POS : goal_position;
          RCLCPP_DEBUG(this->get_logger(), "%d motor is BIG, and clipped to %i", dxl_id, goal_position);
      } else if (SMALL_MOTOR.find(dxl_id) != SMALL_MOTOR.end()) {
          goal_position = goal_position > SMALL_MAX_POS ? SMALL_MAX_POS : goal_position;
          goal_position = goal_position < SMALL_MIN_POS ? SMALL_MIN_POS : goal_position;
          RCLCPP_DEBUG(this->get_logger(), "%d motor is SMALL, and clipped to %i", dxl_id, goal_position);
      } else {
          RCLCPP_WARN(this->get_logger(), "Unknown motor ID for Clipping: %d", dxl_id);
      }
      return goal_position;
  }

  // Modified Pos2Rad function
  float Pos2Rad(uint8_t dxl_id, int position) {
      float degree = 0.0;
      if (BIG_MOTOR.find(dxl_id) != BIG_MOTOR.end()) {
          degree = position * PI / float(BIG_MAX_POS);
      } else if (SMALL_MOTOR.find(dxl_id) != SMALL_MOTOR.end()) {
          degree = position * PI / float(SMALL_MAX_POS);
      } else {
          RCLCPP_WARN(this->get_logger(), "Unknown motor ID for Pos2Rad: %d", dxl_id);
      }
      return degree;
  }

  // Modified Rad2Pos function
  int Rad2Pos(uint8_t dxl_id, float degree) {
      int position = 0;
      if (BIG_MOTOR.find(dxl_id) != BIG_MOTOR.end()) {
          position = degree * BIG_MAX_POS / PI;
      } else if (SMALL_MOTOR.find(dxl_id) != SMALL_MOTOR.end()) {
          position = degree * SMALL_MAX_POS / PI;
      } else {
          RCLCPP_WARN(this->get_logger(), "Unknown motor ID for Rad2Pos: %d", dxl_id);
      }
      return position;
  }

  float Rad2Deg(float degree) {
    float angle_deg = 0.0;
    angle_deg = degree * 180 / PI;
    return angle_deg;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelController>());
  rclcpp::shutdown();
  return 0;
}
