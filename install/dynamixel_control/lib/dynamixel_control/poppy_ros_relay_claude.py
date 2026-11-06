#!/usr/bin/env python3
"""Poppy 각도 릴레이 — 리타겟 툴(UDP)에서 목표 각도를 받아 `/joint_states`로 60Hz 발행.

**이 스크립트는 ROS 2 환경(rclpy가 import 되는 파이썬)에서 실행한다.** 리타겟 툴은 보통
GEO conda env(python 3.9, torch)에서 도는데 시스템 ROS 2의 rclpy(python 3.10)는 거기서
import가 안 되므로, 툴은 UDP로 각도만 흘려보내고 이 릴레이가 ROS 측을 담당한다.

    [리타겟 툴 / GEO] --UDP(15 float64, 도)--> [이 릴레이 / ROS2] --/joint_states--> [로봇]

규약(poppy_read_write_node C++ 노드와 맞춤):
  · 토픽 `/joint_states`, 타입 sensor_msgs/JointState, QoS depth 10
  · 단위 라디안, 발행 60Hz, position[0:15]=모터 ID 1~15, [15:25]=다리(0.0)
  · name=ALL_JOINTS(robot_state_publisher 이름 매칭), header.stamp=현재 시각

동작:
  · target은 **0에서 시작**한다(패킷이 오기 전까지 0을 발행).
  · UDP 패킷이 오면 최신 목표로 갱신, 안 오면 **마지막 목표를 계속 발행**(자세 유지).
  · 툴에서 ROS send를 끄면 패킷이 멎으므로 자동으로 hold 된다.

실행:
    # 예: ROS 2 humble
    source /opt/ros/humble/setup.bash
    python poppy_ros_relay_claude.py            # 기본 포트 47921
    python poppy_ros_relay_claude.py --port 47921 --host 127.0.0.1

검증:
    ros2 topic hz /joint_states          # 60 Hz 근처
    ros2 topic echo /joint_states --once # name 25 / position 25
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import threading

# 툴과 상수·와이어 포맷을 공유한다(ros_bridge_claude는 stdlib만 import → ROS2 환경 안전).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "Robot_Retargeting"))
from ros_bridge_claude import (ALL_JOINTS, CONTROL_FPS, MOTOR_LIMITS_DEG,  # noqa: E402
                               MOTOR_NAMES, PASSIVE_NAMES, UDP_HOST, UDP_PORT, WIRE)

import rclpy                                    # noqa: E402
from rclpy.node import Node                     # noqa: E402
from sensor_msgs.msg import JointState          # noqa: E402


def _clip(deg):
    out = []
    for v, (lo, hi) in zip(deg, MOTOR_LIMITS_DEG):
        out.append(lo if v < lo else hi if v > hi else v)
    return out


class PoppyRelay(Node):
    def __init__(self, host, port):
        super().__init__("poppy_ros_relay")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.target_deg = [0.0] * len(MOTOR_NAMES)     # 0에서 시작
        self._n_rx = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)
        self._alive = True
        self._rx = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx.start()

        self.create_timer(1.0 / CONTROL_FPS, self._publish_once)
        self.get_logger().info(
            f"Poppy relay up: UDP {host}:{port} -> /joint_states @ {CONTROL_FPS} Hz "
            f"(target starts at 0)")

    def _recv_loop(self):
        while self._alive:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != WIRE.size:
                continue
            self.target_deg = _clip(list(WIRE.unpack(data)))   # 원자적 대입
            self._n_rx += 1

    def _publish_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        td = self.target_deg                                   # 원자적 읽기
        msg.position = [math.radians(d) for d in td] + [0.0] * len(PASSIVE_NAMES)
        self.pub.publish(msg)

    def destroy_node(self):
        self._alive = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description="Poppy UDP→/joint_states relay")
    ap.add_argument("--host", default=UDP_HOST, help="UDP bind host")
    ap.add_argument("--port", type=int, default=UDP_PORT, help="UDP bind port")
    args = ap.parse_args()

    rclpy.init()
    node = PoppyRelay(args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
