# dynamixel_control 패키지

Dynamixel 모터로 구동되는 로봇(AVATAR, Poppy, H1)의 제어 및 시각화 노드 모음.

---

## 빌드

```bash
# 반드시 Dynamixel/ 디렉토리에서 실행
cd ~/OneDrive/JMW/dynamixel_8DOF/Dynamixel
colcon build --packages-select dynamixel_control
source install/setup.bash
```

---

## 소스 폴더 구조

```
src/
├── control_node/
│   ├── Avatar_robot/
│   │   └── avatar_read_write_node.cpp   # AVATAR 제어 (Pro 시리즈 21모터)
│   ├── Poppy_robot/
│   │   ├── poppy_read_write_node.cpp    # Poppy 제어 (MX28 15모터)
│   │   └── poppy_csv_joint_state_publisher.cpp
│   ├── read_write_node.cpp              # 기본 2모터 테스트 노드
│   ├── csv_joint_state_publisher.cpp    # 범용 CSV 퍼블리셔
│   ├── h1_csv_joint_state_publisher.cpp # H1 CSV 퍼블리셔
│   ├── joint_state_publisher.cpp        # 수동 joint_state 퍼블리셔
│   └── torque_read_degree_write.cpp     # Poppy(Pro) 토크+각도 제어 노드
├── torque_control_node/
│   └── torque_compensation_node.cpp     # 토크 보상 제어
└── visual_node/
    └── rviz_visualization_node.cpp      # RViz 마커 시각화
```

---

## AVATAR 로봇 (Pro 시리즈, 21모터)

### 이중 시각화 (목표 각도 + 실제 각도)

```bash
# 터미널 1 — RViz 이중 시각화 실행
ros2 launch dynamixel_control avatar_dual_visualization.launch.py

# 터미널 2 — CSV 퍼블리셔 (목표 각도 → /joint_states)
ros2 run dynamixel_control avatar_csv_joint_state_publisher

# 터미널 3 — AVATAR 제어 노드 (하드웨어 연결 필요)
ros2 run dynamixel_control avatar_read_write_node
```

**토픽 구조**
| 토픽 | 방향 | 내용 |
|---|---|---|
| `/joint_states` | 입력 | 목표 각도 (joint1~21, radian) |
| `/joint_states_actual` | 출력 | 실측 각도 (SyncRead) |

**모드 전환** (`avatar_read_write_node.cpp` 상단)
```cpp
constexpr bool TORQUE_WRITE_ENABLED = true;   // 정상 제어
constexpr bool TORQUE_WRITE_ENABLED = false;  // 시각화 테스트 (토크 OFF)
```

### 단순 시각화 (목표 각도만)

```bash
ros2 launch dynamixel_control avatar_robot_visualization.launch.py
```

---

## Poppy 로봇 (MX28, 15모터)

### 이중 시각화 (목표 각도 + 실제 각도)

```bash
# 터미널 1 — RViz 이중 시각화 실행
ros2 launch dynamixel_control poppy_dual_visualization.launch.py

# 터미널 2 — Poppy CSV 퍼블리셔 (목표 각도 → /joint_states)
ros2 run dynamixel_control poppy_csv_joint_state_publisher

# 터미널 3 — Poppy 제어 노드 (하드웨어 연결 필요)
ros2 run dynamixel_control poppy_read_write_node
```

**토픽 구조**
| 토픽 | 방향 | 내용 |
|---|---|---|
| `/joint_states` | 입력 | 목표 각도 (25개 Poppy 관절, radian) |
| `/joint_states_actual` | 출력 | 실측 각도 (SyncRead, 15모터) |

**CSV 데이터 폴더**
```
data/csv/poppy/   ← poppy_csv_joint_state_publisher 가 참조하는 폴더
```

**모드 전환** (`poppy_read_write_node.cpp` 상단)
```cpp
constexpr bool TORQUE_WRITE_ENABLED = true;   // 정상 제어
constexpr bool TORQUE_WRITE_ENABLED = false;  // 시각화 테스트 (토크 OFF)
```

### 단순 시각화 (목표 각도만)

```bash
ros2 launch dynamixel_control poppy_robot_visualization.launch.py
```

---

## H1_2 로봇 (시각화 전용)

```bash
# 터미널 1 — RViz 시각화
ros2 launch dynamixel_control h1_robot_visualization.launch.py

# 터미널 2 — H1 CSV 퍼블리셔
ros2 run dynamixel_control h1_csv_joint_state_publisher
```

---

## 공통 유틸리티

### 범용 CSV 퍼블리셔

```bash
ros2 run dynamixel_control csv_joint_state_publisher
```

### 수동 joint_state 발행

```bash
# 목표 각도 직접 발행 (radian)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState \
  "{name: [], position: [0,0,0,0,0,0,0,0]}"
```

### 토크 ON/OFF

```bash
ros2 topic pub -1 /command std_msgs/msg/String "data: 'enable'"
ros2 topic pub -1 /command std_msgs/msg/String "data: 'disable'"
```

---

## 설정 파일

| 파일 | 내용 |
|---|---|
| `avatar_info.md` | AVATAR 모터 속도 단위, 각도 제한 등 확인 필요 항목 |
| `urdf/avatar_description/` | AVATAR URDF + mesh |
| `urdf/poppy_description/` | Poppy URDF + mesh |
| `urdf/h1_2_description/` | H1_2 URDF + mesh |

---

## 하드웨어 설정

| 항목 | 값 |
|---|---|
| 시리얼 포트 | `/dev/ttyUSB0` |
| 보드레이트 | 1,000,000 bps |
| 프로토콜 | 2.0 |
