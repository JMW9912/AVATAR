#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream> // Csv 파일 저장
#include <map>
#include <utility> // for std::pair
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

// Control table address
#define ADDR_MX28_TORQUE_ENABLE         64
#define ADDR_MX28_VELOCITY_LIMIT        44
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
#define DXL_MAX_ID                      15
// const std::set<uint8_t> WHOLE_MOTOR = {21, 22};
// const std::set<uint8_t> MX28 = {21, 22};
const std::set<uint8_t> WHOLE_MOTOR = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const std::set<uint8_t> MX28 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

#define CONTROL_FPS                     60   // CSV 퍼블리셔 발행 주파수와 동일하게 유지

// ── 속도 설정 ─────────────────────────────────────────────────────────────
// 단위: 0.229 RPM/unit  |  변환: unit = (°/s) / 6 / 0.229
// 예시: 800°/s → 133.3 RPM → 582 unit
//       400°/s →  66.7 RPM → 291 unit
//       200°/s →  33.3 RPM → 145 unit
#define VELOCITY_LIMIT                  582  // EEPROM(addr 44)에 기록되는 최고속도 (≈800°/s)
#define MAX_PROFILE_VEL                 VELOCITY_LIMIT  // Profile Velocity 소프트웨어 상한 (VELOCITY_LIMIT와 동일하게 유지)
// ── 워밍업 설정 ────────────────────────────────────────────────────────────
#define WARMUP_FRAMES                   (CONTROL_FPS * 2)  // 2초 (60프레임)
#define WARMUP_START_VEL                50                  // 워밍업 시작 속도 (≈68°/s)
// ─────────────────────────────────────────────────────────────────────────
// ── 동작 모드 ──────────────────────────────────────────────────────────────
// true : 정상 제어 (토크 ON, 모터에 목표각도 전송)
// false: 시각화 테스트 (토크 OFF, 현재 각도 읽기만 — 손으로 직접 움직이며 확인)
constexpr bool TORQUE_WRITE_ENABLED = true;
// ─────────────────────────────────────────────────────────────────────────

#define CSV_NAME                        "250209_torque_test_2.csv"

std::map<int, std::pair<int, int>> CONST_DEG = {
    {1,  {-155, 120}},
    {2,  {-110, 105}},
    {3,  {-105, 105}},
    {4,  {  -1, 148}},
    {5,  {-120, 155}},
    {6,  {-105, 110}},
    {7,  {-105, 105}},
    {8,  {-148,   1}},
    {9,  { -90,  90}},
    {10, { -45,   6}},
    {11, { -40,  40}},
    {12, { -67,  27}},
    {13, { -45,  45}},
    {14, { -40,  40}},  // abs_x (허리 좌우) — 실제 범위에 맞게 조정 필요
    {15, { -40,  40}},  // abs_y (허리 앞뒤) — 실제 범위에 맞게 조정 필요
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
    
    param_goal_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, MX28_MOTOR_POS); // center(0도)로 초기화
    dxl_present_position.resize(DXL_MAX_ID - DXL_MIN_ID + 1, MX28_MOTOR_POS); // 응답 없는 모터는 중립으로 표시
    param_goal_velocity_.resize(DXL_MAX_ID - DXL_MIN_ID + 1, 0);
    
    // csv 데이터 파일 초기화
    data_file_.open("motor_data.csv", std::ios::out | std::ios::trunc);
    if (data_file_.is_open()) {
      std::ostringstream header_stream;
      header_stream << "Timestamp,"
                    << "TimerCallback Average Time,";

      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; ++dxl_id)
          header_stream << "Motor " << dxl_id << " Desired Position,";
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; ++dxl_id)
          header_stream << "Motor " << dxl_id << " Present Position,";
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; ++dxl_id)
          header_stream << "Motor " << dxl_id << " Target Velocity(unit),";
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
    if (TORQUE_WRITE_ENABLED) {
        setVelocityLimit(WHOLE_MOTOR, VELOCITY_LIMIT);  // EEPROM 기록 — 토크 비활성 상태에서만 가능
        setPositionLimit(WHOLE_MOTOR);
        enableTorque();
        RCLCPP_INFO(this->get_logger(), "[모드] 정상 제어 — 토크 ON, 모터 제어 활성화");
    } else {
        RCLCPP_WARN(this->get_logger(), "[모드] 시각화 테스트 — 토크 OFF, 현재 각도 읽기만");
    }

    // Profile Velocity(addr 112, 4B) + Goal Position(addr 116, 4B) 연속 주소이므로 한 패킷으로 전송
    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX28_PROFILE_VEL, 8);

    // GroupSyncRead: ping에 응답하는 모터만 추가
    // 응답 없는 모터가 포함되면 SDK가 해당 모터 응답을 기다리다 타임아웃되면서
    // 전체 SyncRead 결과가 무효화되어 연결된 모터 데이터도 읽지 못함
    groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_MX28_PRESENT_POSITION, 4);
    for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
      uint8_t ping_error = 0;
      int ping_result = packetHandler->ping(portHandler, static_cast<uint8_t>(dxl_id), &ping_error);
      if (ping_result == COMM_SUCCESS) {
        if (!groupSyncRead->addParam(static_cast<uint8_t>(dxl_id))) {
          RCLCPP_ERROR(this->get_logger(), "GroupSyncRead: motor #%d 파라미터 추가 실패", dxl_id);
        } else {
          RCLCPP_INFO(this->get_logger(), "Motor #%d 연결 확인 — GroupSyncRead 추가", dxl_id);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Motor #%d 응답 없음 — GroupSyncRead 제외", dxl_id);
      }
    }

    // Subscribe to the states topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&PoppyController::topicCallback, this, std::placeholders::_1));

    // Publish actual (measured) joint positions for dual visualization
    actual_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_actual", 10);

    // 터미널 로그 전용 타이머 (10Hz — 사람이 읽기 적합한 속도)
    // 읽기/CSV저장은 topicCallback 내부에서 120Hz 동기 처리
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PoppyController::timerCallback, this));

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
    delete groupSyncRead;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr actual_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupSyncWrite *groupSyncWrite;
  dynamixel::GroupSyncRead  *groupSyncRead;
  uint8_t dxl_error = 0;
  std::ofstream data_file_; // csv 파일 저장
  std::vector<int32_t> dxl_present_position;
  std::vector<int> param_goal_position;
  std::vector<int> param_goal_velocity_;
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;
  // 응답속도 확인용
  long long total_time_ = 0;  // 총 시간 누적
  int callback_count_ = 0;    // 콜백 호출 횟수
  // 워밍업
  int warmup_frame_ = 0;
  // GroupSyncRead 경고 중복 억제 (모터 당 최초 1회만 출력)
  std::set<int> sync_read_warned_;

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

  void setVelocityLimit(const std::set<uint8_t> &motors, int value) {
    for (auto dxl_id : motors) {
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_MX28_VELOCITY_LIMIT, value, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity Limit for motor #%d", dxl_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Motor #%d Velocity Limit set to %d (%.1f RPM, %.0f°/s)",
                    dxl_id, value, value * 0.229, value * 0.229 * 6.0);
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

  // 10Hz — 현재 위치 읽기 + actual 발행 + 터미널 로그
  // /joint_states 수신 여부와 무관하게 항상 실행됨
  void timerCallback() {
    // ① GroupSyncRead: 현재 모터 위치 읽기
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_DEBUG(this->get_logger(), "GroupSyncRead txRxPacket: %s",
                   packetHandler->getTxRxResult(dxl_comm_result));
    }
    for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
      int idx = dxl_id - DXL_MIN_ID;
      if (groupSyncRead->isAvailable(dxl_id, ADDR_MX28_PRESENT_POSITION, 4)) {
        dxl_present_position[idx] = static_cast<int32_t>(
          groupSyncRead->getData(dxl_id, ADDR_MX28_PRESENT_POSITION, 4));
        sync_read_warned_.erase(dxl_id);
      } else {
        if (sync_read_warned_.find(dxl_id) == sync_read_warned_.end()) {
          RCLCPP_WARN(this->get_logger(),
                      "GroupSyncRead: motor #%d 데이터 없음 (이후 동일 경고 억제)", dxl_id);
          sync_read_warned_.insert(dxl_id);
        }
      }
    }

    // ② /joint_states_actual 발행 — dual visualization 실제(불투명) 모델
    {
      static const std::vector<std::string> actual_joint_names = {
        "r_shoulder_y", "r_shoulder_x", "r_arm_z",   "r_elbow_y",
        "l_shoulder_y", "l_shoulder_x", "l_arm_z",   "l_elbow_y",
        "head_z",       "head_y",
        "bust_x",       "bust_y",
        "abs_z",        "abs_x",        "abs_y",
        "r_hip_x",  "r_hip_z",  "r_hip_y",  "r_knee_y", "r_ankle_y",
        "l_hip_x",  "l_hip_z",  "l_hip_y",  "l_knee_y", "l_ankle_y",
      };
      auto actual_msg = sensor_msgs::msg::JointState();
      actual_msg.header.stamp = this->get_clock()->now();
      actual_msg.name = actual_joint_names;
      actual_msg.position.reserve(25);
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        int idx = dxl_id - DXL_MIN_ID;
        actual_msg.position.push_back(static_cast<double>(
          Pos2Rad(dxl_id, dxl_present_position[idx] - MX28_MOTOR_POS)));
      }
      for (int i = 0; i < 10; i++) actual_msg.position.push_back(0.0);
      actual_pub_->publish(actual_msg);
    }

    // ③ 터미널 로그
    std::ostringstream log_stream;
    for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        int idx = dxl_id - DXL_MIN_ID;
        double cur_deg = Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[idx]));
        double tgt_deg = Rad2Deg(Pos2Rad(dxl_id, param_goal_position[idx]));
        double vel_deg_s = param_goal_velocity_[idx] * 0.229 * 6.0;
        log_stream << std::fixed << std::setprecision(1)
                   << "M#" << dxl_id
                   << " Cur:" << cur_deg
                   << " Tgt:" << tgt_deg
                   << " Vel:" << std::setprecision(0) << vel_deg_s << "°/s";
        if (dxl_id == 4 || dxl_id == 8 || dxl_id == 13) {
            log_stream << "\n";
        } else {
            log_stream << " | ";
        }
    }
    std::cout << log_stream.str() << std::endl;
  }

  // 120Hz — compute → write 제어 루프 (읽기는 timerCallback에서 처리)
  void topicCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received /joint_states message");
    for (size_t i = 0; i < msg->position.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "Joint %zu: %f rad", i + 1, msg->position[i]);
    }

    // ② 목표 위치 파싱
    // 조인트의 최대 개수에 맞게 배열 크기를 동적으로 설정
    int num_joints = DXL_MAX_ID - DXL_MIN_ID + 1;
    std::vector<float> raw_goal_position(num_joints, 0.0); // 크기를 동적으로 설정

    for(int i = 0; i <= (DXL_MAX_ID - DXL_MIN_ID); i++){
        if (i < static_cast<int>(msg->position.size())) {
            raw_goal_position[i] = msg->position[i];
            RCLCPP_DEBUG(this->get_logger(), "Joint %i: original goal position = %d, raw_goal position = %f", i+1, param_goal_position[i], raw_goal_position[i]);
            param_goal_position[i] = Clipping(i + DXL_MIN_ID, Rad2Pos(i + DXL_MIN_ID, raw_goal_position[i]));
            param_goal_position[i] += (MX28_MOTOR_POS); // MX28은 -180~180이 아니라 0~360 이기 때문에 pi에 해당하는 위치만큼 평행이동
            RCLCPP_DEBUG(this->get_logger(), "Joint %i: clipped goal position = %d", i+1, param_goal_position[i]);
        } else {
            // 메시지에 해당 인덱스가 없으면 이전 목표 위치 유지
            RCLCPP_DEBUG(this->get_logger(), "Motor #%d: index %d not in message (size=%zu), holding previous goal",
                         i + DXL_MIN_ID, i, msg->position.size());
        }
    }

    // ③ 워밍업 + 속도 계산: 처음 WARMUP_FRAMES 프레임 동안 속도 상한을 선형으로 증가
    int effective_max_vel;
    if (warmup_frame_ < WARMUP_FRAMES) {
        effective_max_vel = WARMUP_START_VEL +
            (MAX_PROFILE_VEL - WARMUP_START_VEL) * warmup_frame_ / WARMUP_FRAMES;
        warmup_frame_++;
        if (warmup_frame_ == 1)
            RCLCPP_INFO(this->get_logger(),
                "Warmup 시작 — %d 프레임(%.0f초) 동안 속도 %d → %d unit 으로 선형 증가",
                WARMUP_FRAMES, (double)WARMUP_FRAMES / CONTROL_FPS,
                WARMUP_START_VEL, MAX_PROFILE_VEL);
        if (warmup_frame_ == WARMUP_FRAMES)
            RCLCPP_INFO(this->get_logger(), "Warmup 완료 — 최대 속도 허용 (%d unit)", MAX_PROFILE_VEL);
    } else {
        effective_max_vel = MAX_PROFILE_VEL;
    }

    // 프레임당 이동 각도에 비례한 속도 계산 (dxl_present_position은 timerCallback에서 갱신됨)
    const double FRAME_PERIOD = 1.0 / CONTROL_FPS;
    for (int i = 0; i <= (DXL_MAX_ID - DXL_MIN_ID); i++) {
        int dxl_id = i + DXL_MIN_ID;
        int delta_pos = std::abs(param_goal_position[i] - static_cast<int>(dxl_present_position[i]));
        double delta_deg = std::abs(Rad2Deg(Pos2Rad(dxl_id, delta_pos)));
        // delta_deg / FRAME_PERIOD → °/s → RPM (/6) → DXL unit (/0.229), effective_max_vel로 상한
        int vel_unit = std::min(effective_max_vel,
                        std::max(1, static_cast<int>(delta_deg / FRAME_PERIOD / 6.0 / 0.229)));
        param_goal_velocity_[i] = vel_unit;
    }

    // ④ GroupSyncWrite: Profile Velocity(4B) + Goal Position(4B) 한 패킷 전송 (~1.5ms)
    if (TORQUE_WRITE_ENABLED) {
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++) {
        int idx = dxl_id - DXL_MIN_ID;
        uint8_t data[8];
        *reinterpret_cast<int32_t*>(data)     = param_goal_velocity_[idx];
        *reinterpret_cast<int32_t*>(data + 4) = param_goal_position[idx];
        dxl_addparam_result = groupSyncWrite->addParam(dxl_id, data);
        RCLCPP_DEBUG(this->get_logger(), "Add parameter complete %d.", dxl_id);
        if (!dxl_addparam_result) {
          RCLCPP_ERROR(this->get_logger(), "Failed to add Dynamixel#%d to the Syncwrite parameter storage", dxl_id);
          return;
        }
      }
      dxl_comm_result = groupSyncWrite->txPacket();
      if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
      groupSyncWrite->clearParam();
    }

    // ⑤ CSV 저장 (dxl_present_position은 timerCallback에서 갱신된 최신값)
    if (data_file_.is_open()) {
      auto now = this->get_clock()->now();
      data_file_ << now.seconds() << ",";
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++)
        data_file_ << Rad2Deg(Pos2Rad(dxl_id, param_goal_position[dxl_id - DXL_MIN_ID])) << ",";
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++)
        data_file_ << Rad2Deg(Pos2Rad(dxl_id, dxl_present_position[dxl_id - DXL_MIN_ID])) << ",";
      for (int dxl_id = DXL_MIN_ID; dxl_id <= DXL_MAX_ID; dxl_id++)
        data_file_ << param_goal_velocity_[dxl_id - DXL_MIN_ID] << ",";
      data_file_ << "\n";
    }
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
