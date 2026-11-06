"""추정된 Poppy 각도를 로봇으로 내보내는 송신 계층.

기존 리얼타임 파이프라인(카메라→WHAM→리타겟)과 **완전히 분리**돼 있다. 리타겟이 매
프레임 내놓는 `thetas`(도)를 UI가 `update()`로 밀어 넣기만 하면 이 계층이 규약에 맞춰
`/joint_states`(sensor_msgs/JointState, 라디안, 60Hz, position[0:15]=모터 1~15)로 보낸다.

두 가지 백엔드
--------------
이 툴은 보통 GEO conda 환경(python 3.9, torch)에서 도는데, 시스템 ROS 2(예: humble)의
rclpy는 python 3.10용 C 확장이라 GEO에서 **import가 안 된다**. 그래서:

  · **rclpy 백엔드** — 이 파이썬에서 rclpy가 실제로 import 되면(툴을 ROS 지원 환경에서
    실행한 경우) 여기서 직접 노드를 띄워 60Hz로 발행한다. 추가 프로세스가 필요 없다.
  · **UDP 백엔드**(기본 폴백) — rclpy를 못 쓰면 목표 각도(15 float64, 도)를 로컬 UDP로
    쏜다. ROS 2 환경에서 `poppy_ros_relay_claude.py`를 함께 띄우면 그 릴레이가 받아서
    60Hz로 `/joint_states`를 발행한다. 파이썬 환경이 섞이지 않는다.

두 경우 모두 로봇에서의 동작은 동일하다: target은 **0에서 시작**하고, 스트리밍을 끄면
마지막 target을 **계속 유지**한다(rclpy 백엔드는 자기 타이머로, UDP 백엔드는 릴레이가).

UDP 와이어 포맷: 리틀엔디언 float64 15개(모터 ID 1~15 순서, 클리핑된 도). 120바이트.
"""
from __future__ import annotations

import math
import socket
import struct
import threading

CONTROL_FPS = 60
UDP_HOST = "127.0.0.1"
UDP_PORT = 47921                       # 툴 ↔ poppy_ros_relay_claude.py 약속 포트
WIRE = struct.Struct("<15d")           # 모터 1~15 목표(도)

# ── Poppy 조인트 규약 (규약 문서 그대로) ──────────────────────────────
# 인덱스 0~14 → 모터 ID 1~15. C++ 노드는 name을 무시하고 position[i]를 모터 i+1로 읽으므로
# 이 순서가 틀리면 엉뚱한 모터가 움직인다.
MOTOR_NAMES = [
    "r_shoulder_y", "r_shoulder_x", "r_arm_z",   "r_elbow_y",
    "l_shoulder_y", "l_shoulder_x", "l_arm_z",   "l_elbow_y",
    "head_z",       "head_y",
    "bust_x",       "bust_y",
    "abs_z",        "abs_x",        "abs_y",
]
# 인덱스 15~24 → 다리. 제어 대상 아님, 항상 0.0 (RViz 시각화용).
PASSIVE_NAMES = [
    "r_hip_x", "r_hip_z", "r_hip_y", "r_knee_y", "r_ankle_y",
    "l_hip_x", "l_hip_z", "l_hip_y", "l_knee_y", "l_ankle_y",
]
ALL_JOINTS = MOTOR_NAMES + PASSIVE_NAMES

# 모터 ID 1~15 순서의 (min_deg, max_deg). 보내기 전에 여기서 먼저 클리핑한다(디버깅 편의).
MOTOR_LIMITS_DEG = [
    (-155.0, 120.0),  # 1  r_shoulder_y
    (-110.0, 105.0),  # 2  r_shoulder_x
    (-105.0, 105.0),  # 3  r_arm_z
    (  -1.0, 148.0),  # 4  r_elbow_y
    (-120.0, 155.0),  # 5  l_shoulder_y
    (-105.0, 110.0),  # 6  l_shoulder_x
    (-105.0, 105.0),  # 7  l_arm_z
    (-148.0,   1.0),  # 8  l_elbow_y
    ( -90.0,  90.0),  # 9  head_z
    ( -45.0,   6.0),  # 10 head_y
    ( -40.0,  40.0),  # 11 bust_x
    ( -67.0,  27.0),  # 12 bust_y
    ( -45.0,  45.0),  # 13 abs_z
    ( -40.0,  40.0),  # 14 abs_x  (임시값)
    ( -40.0,  40.0),  # 15 abs_y  (임시값)
]


def clip_target(thetas, theta_names):
    """리타겟 결과(도)를 **이름 기반**으로 모터 1~15 순서에 꽂고 리밋으로 클리핑한다.

    `theta_names`는 리타게터의 URDF 관절명 리스트(theta1..N 순서), `thetas`는 같은 순서의
    각도(도). MOTOR_NAMES의 각 이름에 해당하는 값을 찾고, 없으면 0. 순서가 아니라 이름으로
    매핑하므로 config의 theta 순서가 바뀌어도 안전하다. 길이 15 리스트(도)를 돌려준다.
    """
    name2val = {}
    for n, v in zip(theta_names, thetas):
        try:
            name2val[str(n)] = float(v)
        except (TypeError, ValueError):
            continue
    out = []
    for name, (lo, hi) in zip(MOTOR_NAMES, MOTOR_LIMITS_DEG):
        v = name2val.get(name, 0.0)
        out.append(lo if v < lo else hi if v > hi else v)
    return out


def rclpy_importable():
    """이 파이썬에서 rclpy가 **실제로 import** 되는지 (find_spec은 폴더만 보고 오탐한다)."""
    try:
        import rclpy  # noqa: F401
        import sensor_msgs.msg  # noqa: F401
        return True
    except Exception:
        return False


class RosBridge:
    """Poppy 각도 송신기. rclpy가 되면 in-process 60Hz 발행, 아니면 UDP로 릴레이에 전달.

    스레드 안전성: `update()`는 Qt 메인 스레드에서, rclpy 발행 콜백은 스핀 스레드에서 돈다.
    공유 상태는 `self._target_deg`(15개 리스트) 하나뿐이고, update가 새 리스트를 통째로
    대입하고 발행이 그 참조를 한 번 읽어 쓰므로 GIL 아래에서 원자적이다(락 불필요).
    """

    def __init__(self, udp_host=UDP_HOST, udp_port=UDP_PORT):
        # 처음 켰을 때 모든 각도가 0도에서 시작하도록 target을 0으로 둔다.
        self._target_deg = [0.0] * len(MOTOR_NAMES)
        self.backend = None            # 'rclpy' | 'udp' | None
        self._udp = (udp_host, udp_port)
        self._sock = None
        # rclpy 백엔드용
        self._node = None
        self._pub = None
        self._timer = None
        self._thread = None
        self._rclpy = None
        self._JointState = None
        self._we_inited = False

    # ── 수명주기 ─────────────────────────────────────────────────
    def start(self):
        """백엔드를 고르고 시작한다. rclpy가 되면 그걸로, 아니면 UDP. 성공 시 backend를 세팅."""
        if self.backend is not None:
            return True
        if rclpy_importable() and self._start_rclpy():
            self.backend = "rclpy"
        else:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.backend = "udp"
            self._send_udp()           # 시작 시 0 목표를 한 번 릴레이에 알린다
        return True

    def _start_rclpy(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception:
            return False
        self._rclpy = rclpy
        self._JointState = JointState
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._we_inited = True
            node = Node("poppy_target_publisher")
            self._pub = node.create_publisher(JointState, "/joint_states", 10)
            self._timer = node.create_timer(1.0 / CONTROL_FPS, self._publish_once)
            self._node = node
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            return True
        except Exception:
            return False

    def _spin(self):
        try:
            self._rclpy.spin(self._node)
        except Exception:
            pass

    @property
    def running(self):
        return self.backend is not None

    def shutdown(self):
        """백엔드를 정리한다. 여러 번 불러도 안전."""
        if self.backend == "rclpy" and self._node is not None:
            node, thread = self._node, self._thread
            self._node = self._pub = self._timer = self._thread = None
            # spin()을 먼저 끊고(→ context shutdown) 스레드를 합류시킨 뒤 노드를 파괴한다.
            try:
                if self._we_inited and self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass
            if thread is not None:
                thread.join(timeout=2.0)
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self.backend = None

    # ── 목표 갱신 ────────────────────────────────────────────────
    def update(self, thetas, theta_names):
        """리타겟 결과(도)를 목표로 반영. rclpy면 타이머가 알아서 발행, UDP면 즉시 한 번 쏜다."""
        self._target_deg = clip_target(thetas, theta_names)   # 원자적 대입
        if self.backend == "udp":
            self._send_udp()

    def _send_udp(self):
        if self._sock is None:
            return
        try:
            self._sock.sendto(WIRE.pack(*self._target_deg), self._udp)
        except OSError:
            pass                       # 릴레이가 아직 안 떠 있어도 조용히 무시

    # ── 발행 (rclpy 스핀 스레드) ─────────────────────────────────
    def _publish_once(self):
        node = self._node
        if node is None:
            return
        msg = self._JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        td = self._target_deg          # 원자적 읽기
        msg.position = [math.radians(d) for d in td] + [0.0] * len(PASSIVE_NAMES)
        self._pub.publish(msg)
