#!/usr/bin/env python3
"""
AVATAR Slider GUI
슬라이더로 각 모터의 목표 각도를 설정 → /joint_states 퍼블리시
avatar_read_write_node가 프레임 단위 속도를 계산해 모터에 전달.

실행: ros2 run dynamixel_control avatar_slider_gui.py
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

# avatar_read_write_node의 CONTROL_FPS와 동일하게 유지
CONTROL_FPS = 120

NUM_MOTORS = 21

# 모터 ID(1-based) 순서 → URDF joint 이름 (avatar_read_write_node의 position 인덱스와 동일)
JOINT_NAMES = [
    "joint1",  "joint2",  "joint3",  "joint4",
    "joint5",  "joint6",  "joint7",  "joint8",
    "joint9",  "joint10", "joint11", "joint12",
    "joint13", "joint14", "joint15", "joint16",
    "joint17", "joint18", "joint19",
    "joint20", "joint21",
]

# 모터 ID(1-based) → GUI 표시용 레이블 (avatar_info.md 기준)
MOTOR_LABELS = {
    1:  "R_Shoulder1 (Z)",
    2:  "R_Shoulder2 (-Y)",
    3:  "R_Shoulder3 (-X)",
    4:  "R_Upperarm (-Z)",
    5:  "R_Forearm (-Y)",
    6:  "R_Wrist1 (-Z)",
    7:  "R_Wrist2 (-Y)",
    8:  "R_Wrist3 (-X)",
    9:  "L_Shoulder1 (Z)",
    10: "L_Shoulder2 (+Y)",
    11: "L_Shoulder3 (-X)",
    12: "L_Upperarm (-Z)",
    13: "L_Forearm (+Y)",
    14: "L_Wrist1 (-Z)",
    15: "L_Wrist2 (+Y)",
    16: "L_Wrist3 (+X)",
    17: "Neck1 (Z)",
    18: "Neck2 (-Y)",
    19: "Head (X)",
    20: "Waist_Top (Y)",
    21: "Waist_Bottom (Z)",
}

# 모터 ID(1-based) → (min_deg, max_deg) — avatar_read_write_node의 CONST_DEG와 동일
CONST_DEG = {
    1:  (-30,   60),
    2:  (-45,  160),
    3:  (  0,  160),
    4:  (-90,   90),
    5:  (  0,  120),
    6:  (-90,   90),
    7:  (-60,   60),
    8:  (-90,   60),
    9:  (-60,   30),
    10: (-160,   45),
    11: (-160,    0),
    12: (-90,   90),
    13: (-120,    0),
    14: (-90,   90),
    15: (-60,   60),
    16: (-90,   60),
    17: (-90,   90),
    18: (-90,   90),
    19: (-90,   90),
    20: (-30,   30),
    21: (-30,   30),
}

# GUI 그룹 레이아웃
GROUPS = [
    ("Right Arm",   [1,  2,  3,  4,  5,  6,  7,  8]),
    ("Left Arm",    [9,  10, 11, 12, 13, 14, 15, 16]),
    ("Neck / Head", [17, 18, 19]),
    ("Waist",       [20, 21]),
]


class AvatarSliderPublisher(Node):
    """ROS2 노드: 슬라이더 목표 각도를 /joint_states로 퍼블리시"""

    def __init__(self):
        super().__init__('avatar_slider_gui')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self._lock = threading.Lock()
        self._target_deg = [0.0] * NUM_MOTORS

        self.create_timer(1.0 / CONTROL_FPS, self._publish_callback)

    def set_target(self, motor_idx: int, deg: float):
        with self._lock:
            self._target_deg[motor_idx] = deg

    def get_target(self):
        with self._lock:
            return list(self._target_deg)

    def _publish_callback(self):
        with self._lock:
            positions_deg = list(self._target_deg)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(d) for d in positions_deg]
        self.publisher_.publish(msg)


class AvatarSliderGUI:
    """tkinter GUI: 모터별 슬라이더 및 현재 각도 표시"""

    def __init__(self, node: AvatarSliderPublisher):
        self.node = node

        self.root = tk.Tk()
        self.root.title("AVATAR Motor Slider Control")
        self.root.resizable(False, False)

        self._slider_vars: list[tk.DoubleVar] = []
        self._val_labels:  list[tk.Label]     = []

        self._build_ui()
        self._update_labels()

    def _build_ui(self):
        hdr = tk.Frame(self.root, pady=6, padx=16)
        hdr.pack(fill='x')
        tk.Label(hdr, text="AVATAR Motor Slider Control",
                 font=('Helvetica', 13, 'bold')).pack(side='left')
        tk.Label(hdr, text=f"speed ctrl → avatar_read_write_node  ({CONTROL_FPS} fps)",
                 font=('Helvetica', 9), fg='gray').pack(side='right')

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8)

        col_hdr = tk.Frame(self.root, padx=16)
        col_hdr.pack(fill='x', pady=(4, 0))
        tk.Label(col_hdr, text="Motor",  width=22, anchor='w', font=('Courier', 9, 'bold'), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="  min",  width=6,  anchor='e', font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text=" " * 14 + "slider" + " " * 14, font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="max",    width=5,  anchor='w', font=('Courier', 9), fg='gray').pack(side='left')
        tk.Label(col_hdr, text="  Value", width=8, anchor='e', font=('Courier', 9), fg='gray').pack(side='left')

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8, pady=2)

        content = tk.Frame(self.root, padx=16)
        content.pack(fill='both', expand=True)

        for group_name, motor_ids in GROUPS:
            grp_lbl = tk.Label(content, text=f"── {group_name} ──",
                               font=('Helvetica', 9, 'italic'), fg='#555', anchor='w')
            grp_lbl.pack(fill='x', pady=(6, 1))

            for motor_id in motor_ids:
                self._add_slider_row(content, motor_id)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=8, pady=6)

        btn_frame = tk.Frame(self.root, padx=16, pady=6)
        btn_frame.pack(fill='x')
        tk.Button(btn_frame, text="Zero All", command=self._zero_all,
                  width=12, bg='#4a90d9', fg='white', font=('Helvetica', 10)).pack(side='left', padx=4)
        tk.Button(btn_frame, text="Quit", command=self._quit,
                  width=12, font=('Helvetica', 10)).pack(side='right', padx=4)

    def _add_slider_row(self, parent: tk.Frame, motor_id: int):
        idx = motor_id - 1
        min_deg, max_deg = CONST_DEG[motor_id]
        label = MOTOR_LABELS[motor_id]

        row = tk.Frame(parent)
        row.pack(fill='x', pady=1)

        tk.Label(row, text=f"M{motor_id:02d} {label}", width=22, anchor='w',
                 font=('Courier', 10)).pack(side='left')

        tk.Label(row, text=f"{min_deg:5d}°", font=('Courier', 9), fg='#888').pack(side='left')

        var = tk.DoubleVar(value=0.0)
        ttk.Scale(row, from_=min_deg, to=max_deg, orient='horizontal',
                  variable=var, length=340,
                  command=lambda _, i=idx, v=var: self.node.set_target(i, v.get())).pack(side='left', padx=2)

        tk.Label(row, text=f"{max_deg:4d}°", font=('Courier', 9), fg='#888').pack(side='left')

        val_lbl = tk.Label(row, text="  +0.0°", width=8, anchor='e', font=('Courier', 10), fg='#2255aa')
        val_lbl.pack(side='left', padx=2)

        def on_change(*args, _v=var, _lbl=val_lbl):
            _lbl.config(text=f"{_v.get():+7.1f}°")
        var.trace_add('write', on_change)

        self._slider_vars.append(var)
        self._val_labels.append(val_lbl)

    def _zero_all(self):
        for i, var in enumerate(self._slider_vars):
            var.set(0.0)
            self.node.set_target(i, 0.0)

    def _quit(self):
        self.root.destroy()
        rclpy.shutdown()

    def _update_labels(self):
        targets = self.node.get_target()
        for i, lbl in enumerate(self._val_labels):
            lbl.config(text=f"{targets[i]:+7.1f}°")
        self.root.after(100, self._update_labels)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init(args=sys.argv)
    node = AvatarSliderPublisher()

    # rclpy.spin을 별도 데몬 스레드에서 실행 (tkinter는 메인 스레드 필요)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = AvatarSliderGUI(node)
    gui.run()

    node.destroy_node()


if __name__ == '__main__':
    main()
