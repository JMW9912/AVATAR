#include <set>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <fstream>
#include <map>
#include <utility>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"

// Dynamixel-P/H series control table (Protocol 2.0)
#define ADDR_PRO_TORQUE_ENABLE      512
#define ADDR_PRO_VELOCITY_LIMIT     44   // EEPROM
#define ADDR_PRO_MAX_POSITION       48   // EEPROM
#define ADDR_PRO_MIN_POSITION       52   // EEPROM
#define ADDR_PRO_PROFILE_VEL        560  // RAM — Profile Velocity (위치 제어 모드 속도 프로파일)
#define ADDR_PRO_GOAL_POSITION      564  // RAM
#define ADDR_PRO_PRESENT_POSITION   580  // RAM

#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"
#define PROTOCOL_VERSION            2.0
#define PI                          3.1415926f

#define DXL_MIN_ID                  1
#define DXL_MAX_ID                  21

const std::set<uint8_t> WHOLE_MOTOR  = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21};
const std::set<uint8_t> BIG_MOTOR    = {1,2,3,4,5,9,10,11,12,13,20,21};  // 200W/100W
const std::set<uint8_t> MIDDLE_MOTOR = {6,7,8,14,15,16,18};              // 20W
const std::set<uint8_t> SMALL_MOTOR  = {17,19};                           // 10W

#define BIG_MOTOR_POS               501923   // pi rad 에 해당하는 position 값
#define MIDDLE_MOTOR_POS            303750
#define SMALL_MOTOR_POS             263187

// ── 속도 설정 ─────────────────────────────────────────────────────────────
// 단위 변환: 1 unit = 0.01 RPM (모든 AVATAR 모터 공통)
// vel_unit = (°/s) / 6 / VEL_RPM_PER_UNIT
// 예시: 600°/s → 600/6/0.01 = 10,000 unit (소프트웨어 목표치)
// 하지만 하드웨어 최대: BIG 29 RPM ≈ 174°/s, SMALL 26 RPM ≈ 156°/s
// → EEPROM 상한은 각 그룹 하드웨어 최대치로, 소프트웨어 상한은 SMALL 최대(2600)로 통일
#define VEL_RPM_PER_UNIT            0.01f
#define VELOCITY_LIMIT_BIG          2900    // H54-200, PH54-200: 최대 2900 unit
#define VELOCITY_LIMIT_MIDDLE       2920    // H42-20, H54-100: 최대 2920 unit
#define VELOCITY_LIMIT_SMALL        2600    // PM42-010: 최대 2600 unit
#define MAX_PROFILE_VEL             2600    // 소프트웨어 상한 (SMALL 모터 하드웨어 최대 기준)
// ─────────────────────────────────────────────────────────────────────────
#define WARMUP_FRAMES               (CONTROL_FPS * 2)  // 2초 워밍업
#define WARMUP_START_VEL            100                 // ~1 RPM = 6°/s

// CSV 퍼블리셔 발행 주파수와 반드시 동일하게 유지
#define CONTROL_FPS                 120

// true : 정상 제어 / false : 시각화 테스트 (토크 OFF, 현재 각도만 읽기)
constexpr bool TORQUE_WRITE_ENABLED = true;

// 모터별 각도 제한 [min_deg, max_deg]
std::map<int, std::pair<int,int>> CONST_DEG = {
    // 오른팔 (joint1~8)
    {1,  {-30,  60}},  {2,  {-45, 160}},  {3,  {  0, 160}},  {4,  {-90,  90}},
    {5,  {  0, 120}},  {6,  {-90,  90}},  {7,  {-60,  60}},  {8,  {-90,  60}},
    // 왼팔 (joint9~16)
    {9,  {-60,  30}},  {10, {-160, 45}},  {11, {-160,  0}},  {12, {-90,  90}},
    {13, {-120,  0}},  {14, {-90,  90}},  {15, {-60,  60}},  {16, {-90,  60}},
    // 목/머리 (joint17~19)
    {17, {-90,  90}},  {18, {-90,  90}},  {19, {-90,  90}},
    // 허리 (joint20~21)
    {20, {-30,  30}},  {21, {-30,  30}},
};

class AvatarController : public rclcpp::Node {
public:
  AvatarController() : Node("avatar_read_write") {
    portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    const int N = DXL_MAX_ID - DXL_MIN_ID + 1;
    param_goal_position.resize(N, 0);
    dxl_present_position.resize(N, 0);
    param_goal_velocity_.resize(N, 0);

    // 로그 폴더 생성(없으면) + 파일명에 날짜·시간 접두사(YYMMDD_HHMM) 추가
    std::filesystem::create_directories("log");
    std::time_t now_t = std::time(nullptr);
    char ts[16];
    std::strftime(ts, sizeof(ts), "%y%m%d_%H%M", std::localtime(&now_t));
    std::string log_path = std::string("log/") + ts + "_avatar_motor_data.csv";

    data_file_.open(log_path, std::ios::out | std::ios::trunc);
    if (data_file_.is_open()) {
      RCLCPP_INFO(get_logger(), "로그 파일 생성: %s", log_path.c_str());
      std::ostringstream hdr;
      hdr << "Timestamp,";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; ++id) hdr << "Motor" << id << "_Desired(deg),";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; ++id) hdr << "Motor" << id << "_Present(deg),";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; ++id) {
        hdr << "Motor" << id << "_Velocity(unit)";
        if (id < DXL_MAX_ID) hdr << ",";
      }
      hdr << "\n";
      data_file_ << hdr.str();
    } else {
      RCLCPP_ERROR(get_logger(), "CSV 파일 열기 실패");
    }

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "포트 열기 또는 보드레이트 설정 실패!");
      return;
    }

    disableTorque();
    if (TORQUE_WRITE_ENABLED) {
      setVelocityLimit(BIG_MOTOR,    VELOCITY_LIMIT_BIG);
      setVelocityLimit(MIDDLE_MOTOR, VELOCITY_LIMIT_MIDDLE);
      setVelocityLimit(SMALL_MOTOR,  VELOCITY_LIMIT_SMALL);
      setPositionLimit(WHOLE_MOTOR);
      enableTorque();
      RCLCPP_INFO(get_logger(), "[모드] 정상 제어 — 토크 ON, 모터 제어 활성화");
    } else {
      RCLCPP_WARN(get_logger(), "[모드] 시각화 테스트 — 토크 OFF, 현재 각도 읽기만");
    }

    // Profile Velocity(560, 4B) + Goal Position(564, 4B) 연속 주소 → 8바이트 단일 SyncWrite
    groupSyncWrite = new dynamixel::GroupSyncWrite(
      portHandler, packetHandler, ADDR_PRO_PROFILE_VEL, 8);
    // Present Position SyncRead (addr 580, 4B)
    groupSyncRead = new dynamixel::GroupSyncRead(
      portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, 4);

    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
      uint8_t ping_err = 0;
      if (packetHandler->ping(portHandler, static_cast<uint8_t>(id), &ping_err) == COMM_SUCCESS) {
        if (!groupSyncRead->addParam(static_cast<uint8_t>(id)))
          RCLCPP_ERROR(get_logger(), "SyncRead: motor #%d 파라미터 추가 실패", id);
        else
          RCLCPP_INFO(get_logger(), "Motor #%d 연결 확인 — SyncRead 추가", id);
      } else {
        RCLCPP_WARN(get_logger(), "Motor #%d 응답 없음 — SyncRead 제외", id);
      }
    }

    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AvatarController::topicCallback, this, std::placeholders::_1));

    actual_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states_actual", 10);

    // 10Hz 터미널 로그 + actual 발행 타이머
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AvatarController::timerCallback, this));

    RCLCPP_INFO(get_logger(), "AvatarController 초기화 완료 (FPS=%d, MAX_VEL=%d)",
                CONTROL_FPS, MAX_PROFILE_VEL);
  }

  ~AvatarController() {
    disableTorque();
    if (data_file_.is_open()) data_file_.close();
    portHandler->closePort();
    delete groupSyncWrite;
    delete groupSyncRead;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    actual_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler    *portHandler;
  dynamixel::PacketHandler  *packetHandler;
  dynamixel::GroupSyncWrite *groupSyncWrite;  // addr 560, len 8 (ProfileVel+GoalPos)
  dynamixel::GroupSyncRead  *groupSyncRead;
  uint8_t dxl_error        = 0;
  int     dxl_comm_result  = COMM_TX_FAIL;
  std::ofstream data_file_;
  std::vector<int32_t> dxl_present_position;
  std::vector<int>     param_goal_position;
  std::vector<int>     param_goal_velocity_;
  int warmup_frame_       = 0;
  std::set<int> sync_read_warned_;

  static const std::vector<std::string>& jointNames() {
    static const std::vector<std::string> names = {
      "joint1","joint2","joint3","joint4","joint5","joint6","joint7","joint8",
      "joint9","joint10","joint11","joint12","joint13","joint14","joint15","joint16",
      "joint17","joint18","joint19","joint20","joint21"
    };
    return names;
  }

  void enableTorque() {
    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
      dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, id, ADDR_PRO_TORQUE_ENABLE, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
        RCLCPP_ERROR(get_logger(), "토크 활성화 실패 motor #%d", id);
      else
        RCLCPP_INFO(get_logger(),  "토크 활성화 motor #%d", id);
    }
  }

  void disableTorque() {
    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++)
      packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, 0, &dxl_error);
  }

  void setVelocityLimit(const std::set<uint8_t> &motors, int value) {
    for (auto id : motors) {
      dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, id, ADDR_PRO_VELOCITY_LIMIT, value, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
        RCLCPP_ERROR(get_logger(), "VelocityLimit 설정 실패 motor #%d", id);
      else
        RCLCPP_INFO(get_logger(),  "Motor #%d VelocityLimit = %d", id, value);
    }
  }

  void setPositionLimit(const std::set<uint8_t> &motors) {
    for (auto id : motors) {
      if (CONST_DEG.find(id) == CONST_DEG.end()) continue;
      auto [min_d, max_d] = CONST_DEG[id];
      int min_pos = Rad2Pos(id, Deg2Rad(static_cast<float>(min_d)));
      int max_pos = Rad2Pos(id, Deg2Rad(static_cast<float>(max_d)));
      packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_MIN_POSITION, min_pos, &dxl_error);
      packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_MAX_POSITION, max_pos, &dxl_error);
      RCLCPP_INFO(get_logger(), "Motor #%d limits: [%d, %d]", id, min_pos, max_pos);
    }
  }

  ///////////////////////////////////////////[Control loop]////////////////////////////////////////////////////
  // 10Hz — SyncRead + /joint_states_actual 발행 + 터미널 로그
  void timerCallback() {
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
      RCLCPP_DEBUG(get_logger(), "GroupSyncRead: %s",
                   packetHandler->getTxRxResult(dxl_comm_result));

    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
      int idx = id - DXL_MIN_ID;
      if (groupSyncRead->isAvailable(id, ADDR_PRO_PRESENT_POSITION, 4)) {
        dxl_present_position[idx] = static_cast<int32_t>(
          groupSyncRead->getData(id, ADDR_PRO_PRESENT_POSITION, 4));
        sync_read_warned_.erase(id);
      } else {
        if (sync_read_warned_.find(id) == sync_read_warned_.end()) {
          RCLCPP_WARN(get_logger(), "SyncRead: motor #%d 데이터 없음 (이후 동일 경고 억제)", id);
          sync_read_warned_.insert(id);
        }
      }
    }

    // /joint_states_actual 발행 — dual visualization 실제(불투명) 모델
    {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = get_clock()->now();
      msg.name = jointNames();
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++)
        msg.position.push_back(
          static_cast<double>(Pos2Rad(id, dxl_present_position[id - DXL_MIN_ID])));
      actual_pub_->publish(msg);
    }

    // 터미널 로그
    std::ostringstream log;
    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
      int idx = id - DXL_MIN_ID;
      log << std::fixed << std::setprecision(1)
          << "M#" << id
          << " Cur:" << Rad2Deg(Pos2Rad(id, dxl_present_position[idx]))
          << " Tgt:" << Rad2Deg(Pos2Rad(id, param_goal_position[idx]))
          << " V:" << std::setprecision(0) << param_goal_velocity_[idx];
      if (id == 8 || id == 16 || id == 19 || id == 21)
        log << "\n";
      else
        log << " | ";
    }
    std::cout << log.str() << std::endl;
  }

  // Control FPS — 목표 위치 수신 → 비례 속도 계산 → SyncWrite
  void topicCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "/joint_states 수신");
    const int N = DXL_MAX_ID - DXL_MIN_ID + 1;

    // ① 목표 위치 파싱 + 클리핑 (index i → motor i+1)
    for (int i = 0; i < N; i++) {
      if (i < static_cast<int>(msg->position.size())) {
        float rad = static_cast<float>(msg->position[i]);
        param_goal_position[i] = Clipping(i + DXL_MIN_ID, Rad2Pos(i + DXL_MIN_ID, rad));
      }
    }

    // ② 워밍업 + 비례 속도 계산
    // 한 프레임(1/CONTROL_FPS 초) 안에 delta_deg 를 이동하기 위한 속도 계산
    int effective_max_vel;
    if (warmup_frame_ < WARMUP_FRAMES) {
      effective_max_vel = WARMUP_START_VEL +
        (MAX_PROFILE_VEL - WARMUP_START_VEL) * warmup_frame_ / WARMUP_FRAMES;
      if (warmup_frame_ == 0)
        RCLCPP_INFO(get_logger(), "Warmup 시작 — %d 프레임(%.0f초) 동안 속도 %d → %d",
                    WARMUP_FRAMES, (double)WARMUP_FRAMES / CONTROL_FPS,
                    WARMUP_START_VEL, MAX_PROFILE_VEL);
      warmup_frame_++;
      if (warmup_frame_ == WARMUP_FRAMES)
        RCLCPP_INFO(get_logger(), "Warmup 완료 — 최대 속도 허용 (%d)", MAX_PROFILE_VEL);
    } else {
      effective_max_vel = MAX_PROFILE_VEL;
    }

    const double FRAME_PERIOD = 1.0 / CONTROL_FPS;
    for (int i = 0; i < N; i++) {
      int id        = i + DXL_MIN_ID;
      int delta_pos = std::abs(param_goal_position[i] - static_cast<int>(dxl_present_position[i]));
      double delta_deg = std::abs(Rad2Deg(Pos2Rad(id, delta_pos)));
      // °/s → RPM(/6) → unit(/VEL_RPM_PER_UNIT)
      int vel_unit = std::min(effective_max_vel,
                      std::max(1, static_cast<int>(
                        delta_deg / FRAME_PERIOD / 6.0 / VEL_RPM_PER_UNIT)));
      param_goal_velocity_[i] = vel_unit;
    }

    if (!TORQUE_WRITE_ENABLED) return;

    // ③ SyncWrite: Profile Velocity(560, 4B) + Goal Position(564, 4B) — 8바이트 단일 패킷
    for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
      int idx = id - DXL_MIN_ID;
      int32_t vel = static_cast<int32_t>(param_goal_velocity_[idx]);
      int32_t pos = static_cast<int32_t>(param_goal_position[idx]);
      uint8_t param[8];
      std::memcpy(param + 0, &vel, 4);
      std::memcpy(param + 4, &pos, 4);
      if (!groupSyncWrite->addParam(id, param)) {
        RCLCPP_ERROR(get_logger(), "SyncWrite addParam 실패 motor #%d", id);
        groupSyncWrite->clearParam();
        return;
      }
    }
    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
      RCLCPP_DEBUG(get_logger(), "SyncWrite: %s",
                   packetHandler->getTxRxResult(dxl_comm_result));
    groupSyncWrite->clearParam();

    // ④ CSV 저장
    if (data_file_.is_open()) {
      data_file_ << get_clock()->now().seconds() << ",";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++)
        data_file_ << Rad2Deg(Pos2Rad(id, param_goal_position[id-DXL_MIN_ID])) << ",";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++)
        data_file_ << Rad2Deg(Pos2Rad(id, dxl_present_position[id-DXL_MIN_ID])) << ",";
      for (int id = DXL_MIN_ID; id <= DXL_MAX_ID; id++) {
        data_file_ << param_goal_velocity_[id-DXL_MIN_ID];
        data_file_ << (id < DXL_MAX_ID ? "," : "\n");
      }
    }
  }

  int Clipping(uint8_t id, int goal_pos) {
    if (CONST_DEG.find(id) == CONST_DEG.end()) return goal_pos;
    auto [min_d, max_d] = CONST_DEG[id];
    int min_p = Rad2Pos(id, Deg2Rad(static_cast<float>(min_d)));
    int max_p = Rad2Pos(id, Deg2Rad(static_cast<float>(max_d)));
    int orig  = goal_pos;
    goal_pos  = std::min(max_p, std::max(min_p, goal_pos));
    if (goal_pos != orig)
      RCLCPP_INFO(get_logger(), "Clipping motor #%d: %.1f → %.1f deg",
                  id, Rad2Deg(Pos2Rad(id, orig)), Rad2Deg(Pos2Rad(id, goal_pos)));
    return goal_pos;
  }

  float Pos2Rad(uint8_t id, int pos) {
    if (BIG_MOTOR.count(id))    return pos * PI / static_cast<float>(BIG_MOTOR_POS);
    if (MIDDLE_MOTOR.count(id)) return pos * PI / static_cast<float>(MIDDLE_MOTOR_POS);
    if (SMALL_MOTOR.count(id))  return pos * PI / static_cast<float>(SMALL_MOTOR_POS);
    RCLCPP_WARN_ONCE(get_logger(), "Pos2Rad: 알 수 없는 모터 ID %d", id);
    return 0.0f;
  }

  int Rad2Pos(uint8_t id, float rad) {
    if (BIG_MOTOR.count(id))    return static_cast<int>(rad * static_cast<float>(BIG_MOTOR_POS)    / PI);
    if (MIDDLE_MOTOR.count(id)) return static_cast<int>(rad * static_cast<float>(MIDDLE_MOTOR_POS) / PI);
    if (SMALL_MOTOR.count(id))  return static_cast<int>(rad * static_cast<float>(SMALL_MOTOR_POS)  / PI);
    RCLCPP_WARN_ONCE(get_logger(), "Rad2Pos: 알 수 없는 모터 ID %d", id);
    return 0;
  }

  float Rad2Deg(float rad) { return rad * 180.0f / PI; }
  float Deg2Rad(float deg) { return deg * PI / 180.0f; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvatarController>());
  rclcpp::shutdown();
  return 0;
}
