#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream> // Csv 파일 저장
#include <map>
#include <utility> // for std::pair
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

// Control table address
#define ADDR_MX28_TORQUE_ENABLE         64
#define ADDR_MX28_GOAL_POSITION         116
#define ADDR_MX28_PRESENT_POSITION      132
#define ADDR_MX28_MAX_POSITION          48
#define ADDR_MX28_MIN_POSITION          52
#define ADDR_MX28_PROFILE_VEL           112

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0
#define PI                              3.1415926
#define MX28_MOTOR_POS                  2047 // 0~360 , 0~4095 --> 11.375 / 1도 --> 180도는 2047

#define DXL_MIN_ID                      1
#define DXL_MAX_ID                      13
// const std::set<uint8_t> WHOLE_MOTOR = {21, 22};
// const std::set<uint8_t> MX28 = {21, 22};
const std::set<uint8_t> WHOLE_MOTOR = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const std::set<uint8_t> MX28 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

#define PROFILE_VEL                     50

#define CSV_NAME                        "250209_torque_test_2.csv"

std::map<int, std::pair<int, int>> CONST_DEG = {
    {1, {-155, 120}},
    {2, {-110, 105}},
    {3, {-105, 105}},
    {4, {-1, 148}},
    {5, {-120, 155}},
    {6, {-105, 110}},
    {7, {-105, 105}},
    {8, {-148, 1}},
    {9, {-90, 90}},
    {10, {-45, 6}},
    {11, {-40, 40}},
    {12, {-67, 27}},
    {13, {-90, 90}}
};
// std::map<int, std::pair<int, int>> CONST_DEG = {
//     {1, {-100, 100}},
//     {2, {-180, 180}}
// };

class PoppyController : public rclcpp::Node {
public:
  PoppyController()
    : Node("poppy_degree_write") {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    param_goal_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0); // 초기화
    dxl_present_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    
    // csv 데이터 파일 초기화
    data_file_.open("motor_data.csv", std::ios::out | std::ios::trunc);
    if (data_file_.is_open()) {
      std::ostringstream header_stream;
      header_stream << "Timestamp,"
                    << "TimerCallback Average Time,";

      // "Desired Position" 헤더 작성
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; ++dxl_id) {
          header_stream << "Motor " << dxl_id << " Desired Position,";
      }
      // "Present Position" 헤더 작성
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; ++dxl_id) {
          header_stream << "Motor " << dxl_id << " Present Position,";
      }
      header_stream << "\n";
      // 헤더를 파일에 기록
      data_file_ << header_stream.str();
      RCLCPP_INFO(this->get_logger(), "CSV file initialized with header: %s", header_stream.str().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open torque_data.csv for writing");
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

    // // Enable Dynamixel Torque (write1ByteTxRx(포트?, id, 토크켜는 위치, 값, 에러값))
    // for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
    //   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX28_TORQUE_ENABLE, 1, &dxl_error);
    //   if (dxl_comm_result != COMM_SUCCESS) {
    //     packetHandler->getTxRxResult(dxl_comm_result);
    //   } else if (dxl_error != 0) {
    //     packetHandler->getRxPacketError(dxl_error);
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Dynamixel#%d has been successfully connected", dxl_id);
    //   }
    // }

    disableTorque();
    setPositionLimit(WHOLE_MOTOR);
    setGoalVelocity(WHOLE_MOTOR, PROFILE_VEL);
    enableTorque();

    // Initialize GroupSyncWrite instance
    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX28_GOAL_POSITION, 4);

    // Subscribe to the states topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&PoppyController::topicCallback, this, std::placeholders::_1));

    // Start the timer - Printing current position of dynamixel every 10ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PoppyController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dynamixel Controller has been initialized");
  }

  ~PoppyController() {
    // // Disable Dynamixel Torque
    // for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++){
    //   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX28_TORQUE_ENABLE, 0, &dxl_error);
    //   if (dxl_comm_result != COMM_SUCCESS) {
    //     packetHandler->getTxRxResult(dxl_comm_result);
    //   } else if (dxl_error != 0) {
    //     packetHandler->getRxPacketError(dxl_error);
    //   }
    // } 
    disableTorque();

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
  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupSyncWrite *groupSyncWrite;
  uint8_t dxl_error = 0;
  std::ofstream data_file_; // csv 파일 저장
  std::vector<int32_t> dxl_present_position;
  std::vector<int> param_goal_position;
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;
  // 응답속도 확인용
  long long total_time_ = 0;  // 총 시간 누적
  int callback_count_ = 0;    // 콜백 호출 횟수

  // uint8_t joint_sequence[8] = {1, 2};

  void enableTorque() {
    for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX28_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for motor #%d", dxl_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Torque enabled for motor #%d", dxl_id);
      }
    }
  }

  void disableTorque() {
    for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX28_TORQUE_ENABLE, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable torque for motor #%d", dxl_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Torque disabled for motor #%d", dxl_id);
      }
    }
  }

  void setGoalVelocity(const std::set<uint8_t> &motors, int value) {
    for (auto dxl_id : motors) {
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_MX28_PROFILE_VEL, value, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set goal velocity for motor #%d", dxl_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel#%d velocity has been set %d", dxl_id, value);
      }
    }
  }

  void setPositionLimit(const std::set<uint8_t> &motors) {
    for (auto dxl_id : motors) {
        if (CONST_DEG.find(dxl_id) == CONST_DEG.end()) {
            RCLCPP_WARN(this->get_logger(), "Motor ID %d not found in CONST_DEG!", dxl_id);
            continue;
        }

        // 최소/최대 각도 가져오기
        auto [min_degree, max_degree] = CONST_DEG[dxl_id];

        // 각도를 라디안으로 변환한 후 포지션 값으로 변환
        int min_pos = Rad2Pos(dxl_id, Deg2Rad(min_degree)) + MX28_MOTOR_POS;
        int max_pos = Rad2Pos(dxl_id, Deg2Rad(max_degree)) + MX28_MOTOR_POS;

        // 최소 위치 리미트 설정
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_MX28_MIN_POSITION, min_pos, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Min Position Limit for Motor #%d (Value: %d)", dxl_id, min_pos);
        } else {
            RCLCPP_INFO(this->get_logger(), "Min Position Limit set for Motor #%d: %d", dxl_id, min_pos);
        }

        // 최대 위치 리미트 설정
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_MX28_MAX_POSITION, max_pos, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Max Position Limit for Motor #%d (Value: %d)", dxl_id, max_pos);
        } else {
            RCLCPP_INFO(this->get_logger(), "Max Position Limit set for Motor #%d: %d", dxl_id, max_pos);
        }
    }
  }

  void timerCallback() {
    std::ostringstream log_stream; // 로그를 버퍼에 저장할 스트림
    auto start_time = std::chrono::high_resolution_clock::now(); // 응답 시간 체크
    for(int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_MX28_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&dxl_present_position[dxl_id - DXL_MIN_ID]), &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
        } else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
        }

        log_stream << "Motor#" << dxl_id 
                   << ": " << Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[dxl_id - DXL_MIN_ID]));
        if (dxl_id == 4 || dxl_id == 8 || dxl_id == 13) {
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
    // RCLCPP_INFO(this->get_logger(), "Timer execution time: %ld microseconds, Average time: %.2f microseconds", duration, average_time);

    // csv 데이터 저장
    if (data_file_.is_open()) {
      auto now = this->get_clock()->now(); // 현재 시간
      data_file_ << now.seconds() << ","
                 << average_time << ",";

      // 모든 모터의 목표 각도 기록
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        data_file_ << Rad2Deg(Pos2Rad(dxl_id, param_goal_position[dxl_id - DXL_MIN_ID])) << ",";
      }
      // 모든 모터의 현재 각도 기록
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        data_file_ << Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[dxl_id - DXL_MIN_ID])) << ",";
        // 마지막 값이면 줄바꿈 추가, 아니면 `,` 추가
      }
    }
    }

  void topicCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 응답 시간 체크
    // auto start_time = std::chrono::high_resolution_clock::now();
    // 디버깅 로그
    RCLCPP_DEBUG(this->get_logger(), "Received /joint_states message");
    // 수신된 메시지의 각도를 출력
    for (size_t i = 0; i < msg->position.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "Joint %zu: %f rad", i + 1, msg->position[i]);
    }

    // 조인트의 최대 개수에 맞게 배열 크기를 동적으로 설정
    int num_joints = DXL_MAX_ID - DXL_MIN_ID + 1;
    std::vector<float> raw_goal_position(num_joints, 0.0); // 크기를 동적으로 설정

    for(int i = 0; i <= (DXL_MAX_ID - DXL_MIN_ID); i++){
        raw_goal_position[i] = msg->position[i];
        RCLCPP_DEBUG(this->get_logger(), "Joint %i: original goal position = %d, raw_goal position = %f", i+1, param_goal_position[i], raw_goal_position[i]);
        param_goal_position[i] = Clipping(i + DXL_MIN_ID, Rad2Pos(i + DXL_MIN_ID, raw_goal_position[i]));
        param_goal_position[i] += (MX28_MOTOR_POS); // MX28은 -180~180이 아니라 0~360 이기 때문에 pi에 해당하는 위치만큼 평행이동
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
  }

  int Clipping(uint8_t dxl_id, int goal_position) {
      RCLCPP_DEBUG(this->get_logger(), "Get dxl_id: %i, goal position: %i", dxl_id, goal_position);

      if (CONST_DEG.find(dxl_id) == CONST_DEG.end()) {
          RCLCPP_WARN(this->get_logger(), "Motor ID %d not found in Constraint_Degree!", dxl_id);
          return goal_position; // 기본값 반환
      }

      // 모터의 최소/최대 각도를 라디안으로 변환
      auto[min_degree, max_degree] = CONST_DEG[dxl_id];
      int min_pos = Rad2Pos(dxl_id, Deg2Rad(min_degree));
      int max_pos = Rad2Pos(dxl_id, Deg2Rad(max_degree));
      int original_position = goal_position; // 원래 목표 위치 저장

      goal_position = goal_position > max_pos ? max_pos : goal_position;
      goal_position = goal_position < min_pos ? min_pos : goal_position;
      if (goal_position != original_position) {
          RCLCPP_INFO(this->get_logger(),
                      "Clipping occurred for Motor ID %d: Original Position: %.2f, Clipped Position: %.2f (Min: %.2d, Max: %.2d)",
                      dxl_id, Rad2Deg(Pos2Rad(dxl_id, original_position)), Rad2Deg(Pos2Rad(dxl_id, goal_position)), min_degree, max_degree);
      }

      return goal_position;
  }


  // Modified Pos2Rad function
  float Pos2Rad(uint8_t dxl_id, int position) {
      float degree = 0.0;
      if (MX28.find(dxl_id) != MX28.end()) {
          degree = position * PI / float(MX28_MOTOR_POS);
      } else {
          RCLCPP_WARN(this->get_logger(), "Unknown motor ID for Pos2Rad: %d", dxl_id);
      }
      return degree;
  }

  // Modified Rad2Pos function
  int Rad2Pos(uint8_t dxl_id, float degree) {
      int position = 0;
      if (MX28.find(dxl_id) != MX28.end()) {
          position = degree * MX28_MOTOR_POS / PI;
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
  float Deg2Rad(float degree) {
    return degree * PI / 180.0f; // degree -> radian
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoppyController>());
  rclcpp::shutdown();
  return 0;
}
