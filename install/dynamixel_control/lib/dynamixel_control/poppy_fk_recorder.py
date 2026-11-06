#!/usr/bin/env python3
"""
poppy_fk_recorder.py

/joint_states를 구독하고 TF2를 이용해 각 링크의 3D 위치(pelvis 기준)를
프레임별로 CSV에 기록한다.

실행 방법:
  터미널 1: ros2 launch dynamixel_control poppy_robot_visualization.launch.py
  터미널 2: ros2 run dynamixel_control poppy_fk_recorder.py   ← 먼저 실행
  터미널 3: ros2 run dynamixel_control poppy_csv_joint_state_publisher
  → 재생이 끝나면 터미널 2에서 Ctrl+C → CSV 자동 저장

파라미터:
  output_dir  저장 폴더 (기본: .../data/csv/poppy/fk_output)
  output_file 저장 파일명 (기본: fk_YYYYMMDD_HHMMSS.csv)
"""

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
import tf2_ros
import csv
import os
from datetime import datetime

BASE_FRAME = "pelvis"

LINKS_TO_TRACK = [
    "pelvis",
    "abdomen",
    "chest",
    "neck",
    "head",
    "r_shoulder",
    "r_upper_arm",
    "r_forearm",
    "r_wrist",       # 가상 손목 프레임 (elbow 기준 Y+0.120m, 실측 하완 길이)
    "l_shoulder",
    "l_upper_arm",
    "l_forearm",
    "l_wrist",       # 가상 손목 프레임 (elbow 기준 Y+0.120m, 실측 하완 길이)
    "r_thigh",
    "r_shin",
    "r_foot",
    "l_thigh",
    "l_shin",
    "l_foot",
]


class FKRecorder(Node):
    def __init__(self):
        super().__init__("poppy_fk_recorder")

        default_dir = os.path.expanduser(
            "~/OneDrive/JMW/dynamixel_8DOF/Dynamixel/dynamixel_control/data/csv/poppy/fk_output"
        )
        self.declare_parameter("output_dir", default_dir)
        self.declare_parameter("output_file", "")

        output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        output_file = self.get_parameter("output_file").get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)

        if not output_file:
            output_file = "fk_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        self.output_path = os.path.join(output_dir, output_file)

        # ReentrantCallbackGroup: joint_state 콜백이 executor 내에서 재진입 가능
        cb_group = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        # spin_thread=True(기본값): TF 리스너가 독립 백그라운드 스레드에서 /tf 처리
        # → executor 부하와 무관하게 버퍼가 항상 최신 상태 유지 → 중간 NaN 방지
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.frame_idx = 0
        self.records = []
        self.tf_warn_done = set()

        self.header = ["frame", "timestamp_sec"]
        for link in LINKS_TO_TRACK:
            self.header += [f"{link}_x", f"{link}_y", f"{link}_z"]

        # 큐 크기를 100으로 확대 — 30fps×3초분 버퍼, 드롭 방지
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 100,
            callback_group=cb_group,
        )

        self.get_logger().info(f"FK Recorder 시작. 저장 경로: {self.output_path}")
        self.get_logger().info("CSV 퍼블리셔를 실행하세요. 재생 완료 후 Ctrl+C 로 저장합니다.")

    def on_joint_state(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [self.frame_idx, timestamp]

        for link in LINKS_TO_TRACK:
            if link == BASE_FRAME:
                row += [0.0, 0.0, 0.0]
                continue

            try:
                # latest TF 조회. 5ms timeout: robot_state_publisher 처리 마진.
                # spin_thread=True 덕분에 버퍼가 항상 최신이라 거의 즉시 반환됨.
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    link,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.005),
                )
                tr = t.transform.translation
                row += [tr.x, tr.y, tr.z]
            except Exception as e:
                if link not in self.tf_warn_done:
                    self.get_logger().warn(f"TF 조회 실패 [{link}]: {e}")
                    self.tf_warn_done.add(link)
                row += [float("nan"), float("nan"), float("nan")]

        self.records.append(row)
        self.frame_idx += 1

        if self.frame_idx % 30 == 0:
            self.get_logger().info(f"{self.frame_idx} 프레임 기록 중...")

    def save_csv(self):
        if not self.records:
            self.get_logger().warn("기록된 데이터가 없습니다.")
            return

        with open(self.output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.header)
            writer.writerows(self.records)

        self.get_logger().info(
            f"{len(self.records)} 프레임 저장 완료 → {self.output_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FKRecorder()
    # MultiThreadedExecutor: TF 리스너 콜백과 joint_state 콜백이 병렬 실행
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
