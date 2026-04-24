#!/usr/bin/env python3
"""
Poppy Slider GUI
슬라이더로 각 모터의 목표 각도를 설정 → /joint_states 퍼블리시
GUI 측에서 MAX_DEG_PER_SEC으로 보간하여 급격한 움직임을 방지한 뒤,
poppy_read_write_node가 프레임 단위 속도를 추가로 계산해 모터에 전달.

실행: python3 scripts/poppy_slider_gui.py
     (ROS2 환경이 source된 터미널에서 실행)
"""

import math
import threading
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tkinter as tk
from tkinter import ttk

# ── 설정값 ──────────────────────────────────────────────────────────────────
CONTROL_FPS = 30   # poppy_read_write_node의 CONTROL_FPS와 동일하게 유지
# 속도 제한은 read_write_node의 MAX_PROFILE_VEL 에서 처리 (GUI 보간 없음)
# ────────────────────────────────────────────────────────────────────────────

NUM_MOTORS = 13

# 슬라이더로 제어하는 13개 모터
MOTOR_NAMES = [
    "r_shoulder_y", "r_shoulder_x", "r_arm_z",   "r_elbow_y",
    "l_shoulder_y", "l_shoulder_x", "l_arm_z",   "l_elbow_y",
    "head_z",       "head_y",
    "bust_x",       "bust_y",       "abs_z",
]

# URDF의 모든 비고정 조인트
# ※ 앞 13개(인덱스 0~12)는 반드시 MOTOR_NAMES 순서와 동일해야 함
#   → read_write_node가 msg->position[i] 인덱스로 모터 값을 읽기 때문
ALL_JOINTS = [
    # 인덱스 0~12: read_write_node가 인덱스로 읽는 제어 조인트 (MOTOR_NAMES 순서 유지)
    "r_shoulder_y", "r_shoulder_x", "r_arm_z",   "r_elbow_y",
    "l_shoulder_y", "l_shoulder_x", "l_arm_z",   "l_elbow_y",
    "head_z",       "head_y",
    "bust_x",       "bust_y",       "abs_z",
    # 인덱스 13~24: 비제어 조인트 — 0.0 고정 (시각화용)
    "r_hip_x",  "r_hip_z",  "r_hip_y",  "r_knee_y", "r_ankle_y",
    "l_hip_x",  "l_hip_z",  "l_hip_y",  "l_knee_y", "l_ankle_y",
    "abs_y",    "abs_x",
]

# motor_id(1-based) → (min_deg, max_deg)  — poppy_read_write_node의 CONST_DEG와 동일
CONST_DEG = {
    1:  (-155, 120),
    2:  (-110, 105),
    3:  (-105, 105),
    4:  (  -1, 148),
    5:  (-120, 155),
    6:  (-105, 110),
    7:  (-105, 105),
    8:  (-148,   1),
    9:  ( -90,  90),
    10: ( -45,   6),
    11: ( -40,  40),
    12: ( -67,  27),
    13: ( -90,  90),
}

# GUI 그룹 레이아웃 (표시용)
GROUPS = [
    ("Right Arm", [1, 2, 3, 4]),
    ("Left Arm",  [5, 6, 7, 8]),
    ("Head",      [9, 10]),
    ("Torso",     [11, 12, 13]),
]


class SliderPublisher(Node):
    """ROS2 노드: 보간된 각도를 /joint_states로 퍼블리시"""

    def __init__(self):
        super().__init__('poppy_slider_gui')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self._lock = threading.Lock()
        # 슬라이더가 지정하는 목표 각도 (도)
        self._target_deg = [0.0] * NUM_MOTORS

        self.create_timer(1.0 / CONTROL_FPS, self._publish_callback)

    # ── GUI 스레드에서 호출 ──────────────────────────────────────────────────
    def set_target(self, motor_idx: int, deg: float):
        with self._lock:
            self._target_deg[motor_idx] = deg

    def get_target(self):
        with self._lock:
            return list(self._target_deg)

    # ── ROS2 타이머 콜백 (spin 스레드에서 실행) ──────────────────────────────
    def _publish_callback(self):
        with self._lock:
            positions_deg = list(self._target_deg)

        # 슬라이더 값을 그대로 발행 — 속도 제어는 read_write_node가 담당
        controlled = {name: math.radians(positions_deg[i])
                      for i, name in enumerate(MOTOR_NAMES)}

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [controlled.get(name, 0.0) for name in ALL_JOINTS]
        self.publisher_.publish(msg)


class SliderGUI:
    """tkinter GUI: 모터별 슬라이더 및 현재 각도 표시"""

    def __init__(self, node: SliderPublisher):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Poppy Motor Slider Control")
        self.root.resizable(False, False)

        self._slider_vars: list[tk.DoubleVar] = []
        self._val_labels:  list[tk.Label]     = []

        self._build_ui()
        self._update_labels()   # 10fps로 현재 각도 갱신 시작

    # ── UI 빌드 ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        # 헤더
        hdr = tk.Frame(self.root, pady=6, padx=16)
        hdr.pack(fill='x')
        tk.Label(hdr, text="Poppy Motor Slider Control",
                 font=('Helvetica', 13, 'bold')).pack(side='left')
        tk.Label(hdr, text=f"speed ctrl → read_write_node",
                 font=('Helvetica', 9), fg='gray').pack(side='right')

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8)

        # 칼럼 헤더
        col_hdr = tk.Frame(self.root, padx=16)
        col_hdr.pack(fill='x', pady=(4, 0))
        tk.Label(col_hdr, text="Motor",  width=16, anchor='w', font=('Courier', 9, 'bold'), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="min",    width=5,  anchor='e', font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text=" " * 14 + "slider" + " " * 14, font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="max",    width=5,  anchor='w', font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="  Value", width=8, anchor='e', font=('Courier', 9), fg='gray').pack(side='left')

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8, pady=2)

        # 그룹별 슬라이더 행
        content = tk.Frame(self.root, padx=16)
        content.pack(fill='both', expand=True)

        for group_name, motor_ids in GROUPS:
            grp_lbl = tk.Label(content, text=f"── {group_name} ──",
                               font=('Helvetica', 9, 'italic'), fg='#555', anchor='w')
            grp_lbl.pack(fill='x', pady=(6, 1))

            for motor_id in motor_ids:
                self._add_slider_row(content, motor_id)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8, pady=6)

        # 버튼 영역
        btn_frame = tk.Frame(self.root, padx=16, pady=6)
        btn_frame.pack(fill='x')
        tk.Button(btn_frame, text="Zero All", command=self._zero_all,
                  width=12, bg='#4a90d9', fg='white', font=('Helvetica', 10)).pack(side='left', padx=4)
        tk.Button(btn_frame, text="Quit", command=self._quit,
                  width=12, font=('Helvetica', 10)).pack(side='right', padx=4)

    def _add_slider_row(self, parent: tk.Frame, motor_id: int):
        idx = motor_id - 1
        min_deg, max_deg = CONST_DEG[motor_id]
        name = MOTOR_NAMES[idx]

        row = tk.Frame(parent)
        row.pack(fill='x', pady=1)

        # 이름
        tk.Label(row, text=f"M{motor_id:02d} {name}", width=16, anchor='w',
                 font=('Courier', 10)).pack(side='left')

        # 최솟값 레이블
        tk.Label(row, text=f"{min_deg:4d}°", font=('Courier', 9), fg='#888').pack(side='left')

        # 슬라이더
        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(row, from_=min_deg, to=max_deg, orient='horizontal',
                           variable=var, length=340,
                           command=lambda _, i=idx, v=var: self.node.set_target(i, v.get()))
        slider.pack(side='left', padx=2)

        # 최댓값 레이블
        tk.Label(row, text=f"{max_deg:4d}°", font=('Courier', 9), fg='#888').pack(side='left')

        # 현재 각도 값 표시
        val_lbl = tk.Label(row, text="  +0.0°", width=8, anchor='e', font=('Courier', 10), fg='#2255aa')
        val_lbl.pack(side='left', padx=2)

        # trace_add는 (varname, index, mode) 3개 위치 인자를 전달하므로 *args로 흡수
        def on_change(*args, _v=var, _lbl=val_lbl):
            _lbl.config(text=f"{_v.get():+7.1f}°")
        var.trace_add('write', on_change)

        self._slider_vars.append(var)
        self._val_labels.append(val_lbl)

    # ── 버튼 콜백 ────────────────────────────────────────────────────────────
    def _zero_all(self):
        for i, var in enumerate(self._slider_vars):
            var.set(0.0)
            self.node.set_target(i, 0.0)

    def _quit(self):
        self.root.destroy()
        rclpy.shutdown()

    # ── 주기적 라벨 갱신 (10fps, tkinter 메인 스레드에서 실행) ────────────────
    def _update_labels(self):
        targets = self.node.get_target()
        for i, lbl in enumerate(self._val_labels):
            lbl.config(text=f"{targets[i]:+7.1f}°")
        self.root.after(100, self._update_labels)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init(args=sys.argv)
    node = SliderPublisher()

    # rclpy.spin을 별도 데몬 스레드에서 실행 (tkinter는 메인 스레드 필요)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = SliderGUI(node)
    gui.run()

    node.destroy_node()


if __name__ == '__main__':
    main()
