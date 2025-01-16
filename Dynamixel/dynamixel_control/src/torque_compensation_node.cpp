// Check the GITHUb
//branch work

#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611
#define ADDR_PRO_PRESENT_CURRENT        621

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

#define BIG_MIN_POS                     -125480 //-250961
#define BIG_MAX_POS                     125480  //250961
#define BIG_POS_VALUE                   250961

#define SMALL_MIN_POS                   -151875
#define SMALL_MAX_POS                   151875
#define SMALL_POS_VALUE                   151875

#define ENABLE 0

class TorqueController : public rclcpp::Node {
public:
  TorqueController()
    : Node("dxl") {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    joint_present_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); // present degree publisher
    
    param_goal_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0); // 초기화
    dxl_present_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    present_current_raw.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    present_current_mA.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    
    // csv 데이터 파일 초기화
    data_file_.open("Degree_data.csv", std::ios::out | std::ios::trunc);
    if (data_file_.is_open()) {
        data_file_ << "Average timer callback time, Average topic callback time,Desired Position Motor #1,Present Position Motor #1,"
                      "Present Current #1,Present Position Motor #2\n";
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
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, ENABLE, &dxl_error);
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
      "/joint_states", 10, std::bind(&TorqueController::topicCallback, this, std::placeholders::_1));

    // Subscribe to the command topic
    command_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "command", 10, std::bind(&TorqueController::commandCallback, this, std::placeholders::_1));

    // Start the timer - Printing current position of dynamixel every 10ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TorqueController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dynamixel Controller has been initialized");
  }

  ~TorqueController() {
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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_present_state_publisher_; // present degree 발행
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
  std::vector<int16_t> present_current_raw;
  std::vector<int16_t> present_current_mA;

  // 응답속도 확인용
  long long total_time_ = 0;  // 총 시간 누적
  int callback_count_ = 0;    // 콜백 호출 횟수
  long long total_topic_time_ = 0;  // 총 시간 누적
  int topic_callback_count_ = 0;    // 콜백 호출 횟수
  double average_time_topic = 0;

  uint8_t joint_sequence[8] = {1, 2};

  void timerCallback() {
    std::ostringstream log_stream; // 로그를 버퍼에 저장할 스트림
    auto start_time = std::chrono::high_resolution_clock::now(); // 응답 시간 체크
    const double current_unit = 16.11328; // Current 단위 변환 (mA)

    // JointPresentState 메시지 생성
    auto joint_present_state_msg = sensor_msgs::msg::JointState();
    joint_present_state_msg.header.stamp = this->get_clock()->now();

    // 조인트 이름 설정 (1번부터 21번까지)
    for (int i = 1; i <= 21; ++i) {
        joint_present_state_msg.name.push_back("joint" + std::to_string(i));
    }
    // 조인트 각도 초기화
    joint_present_state_msg.position.resize(21, 0.0); // 초기값 0으로 설정
    
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
      int index = dxl_id - DXL_MIN_ID;
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&dxl_present_position[index]), &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
        } else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
        } else {
          joint_present_state_msg.position[dxl_id - 1] = Pos2Rad(dxl_id, dxl_present_position[index]);
        }

        // 현재 전류 읽기
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, ADDR_PRO_PRESENT_CURRENT, 
                        reinterpret_cast<uint16_t*>(&present_current_raw[index]), &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
        } else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
        }else {
          RCLCPP_DEBUG(this->get_logger(), "Successfully read present current for Motor %d, %d", dxl_id, present_current_raw[index]);
          }

        // 전류(mA) 계산
        present_current_mA[index] = present_current_raw[index] * current_unit;

        log_stream << std::fixed << std::setprecision(4) // 소수점 4자리 출력
                   << "Motor#" << dxl_id 
                   << ": " << Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[index]))
                   << " | Current " << present_current_mA[index] << " mA";
        if (dxl_id == DXL_MAX_ID) {
            log_stream << "\n";
        } else {
            log_stream << " | ";
        }
        // JointState 메시지에 데이터 추가
        // joint_present_state_msg.name.push_back("joint" + std::to_string(dxl_id));
        // joint_present_state_msg.position.push_back(Pos2Rad(dxl_id, dxl_present_position[index]));
        // joint_present_state_msg.effort.push_back(present_current_mA[index]);
    }

    // 퍼블리시
    joint_present_state_publisher_->publish(joint_present_state_msg);

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
    RCLCPP_DEBUG(this->get_logger(), "Timer execution time: %ld microseconds, Average time: %.2f microseconds", duration, average_time);

    // csv 데이터 저장
    if (data_file_.is_open()) {
        auto now = this->get_clock()->now(); // 현재 시간
        data_file_ << average_time << "," // 타임스탬프(초 단위)
                  << average_time_topic << "," // topiccallback
                  << Rad2Deg(Pos2Rad(1, param_goal_position[0])) << ","   // 목표 위치 모터 #1
                  << Rad2Deg(Pos2Rad(1, dxl_present_position[0])) << "," // 현재 위치 모터 #1
                  << present_current_mA[0] << ","   // 현재 전류 모터 #1
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
    RCLCPP_DEBUG(this->get_logger(), "topicCallback execution time: %ld microseconds, Average time: %.2f microseconds", duration, average_time);
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
          degree = position * PI / float(BIG_POS_VALUE);
      } else if (SMALL_MOTOR.find(dxl_id) != SMALL_MOTOR.end()) {
          degree = position * PI / float(SMALL_POS_VALUE);
      } else {
          RCLCPP_WARN(this->get_logger(), "Unknown motor ID for Pos2Rad: %d", dxl_id);
      }
      return degree;
  }

  // Modified Rad2Pos function
  int Rad2Pos(uint8_t dxl_id, float degree) {
      int position = 0;
      if (BIG_MOTOR.find(dxl_id) != BIG_MOTOR.end()) {
          position = degree * BIG_POS_VALUE / PI;
      } else if (SMALL_MOTOR.find(dxl_id) != SMALL_MOTOR.end()) {
          position = degree * SMALL_POS_VALUE / PI;
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
  rclcpp::spin(std::make_shared<TorqueController>());
  rclcpp::shutdown();
  return 0;
}
