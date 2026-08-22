#!/usr/bin/env python3
"""合成相机测试发布器。

虚拟机没有真实摄像头, camera_node 无法打开设备会直接抛异常。
本节点替代 camera_node, 以指定帧率向 /image_raw 发布内容随时间变化的
合成 BGR 图像(移动色条 + 渐变背景), 保证:
  1. datacache_node 的图像订阅/缓冲链路有真实数据流;
  2. 转换出的 jpg 每帧内容不同, 便于人工抽查。

用法:
  python3 test_camera_publisher.py --fps 30 --width 640 --height 480
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class SyntheticCameraPublisher(Node):

    def __init__(self, fps: int, width: int, height: int) -> None:
        super().__init__('test_camera_publisher')
        self.width = width
        self.height = height
        self.frame_index = 0
        self.published = 0

        # datacache_node 使用 SensorDataQoS(best_effort)订阅, 发布端必须匹配
        self.publisher = self.create_publisher(Image, '/image_raw',
                                               qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.stat_timer = self.create_timer(5.0, self.report_stats)
        self.get_logger().info(
            f'Synthetic camera started [{width}x{height} @ {fps}fps]')

    def publish_frame(self) -> None:
        frame = self.make_frame(self.frame_index)
        self.frame_index += 1

        msg = Image()
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = frame.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher.publish(msg)
        self.published += 1

    def make_frame(self, index: int) -> np.ndarray:
        """渐变背景 + 随帧号移动的色条, 每帧内容均不相同。"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # 蓝色水平渐变背景
        frame[:, :, 0] = np.linspace(0, 255, self.width, dtype=np.uint8)[None, :]
        # 随帧号移动的绿色竖条 (速度 8 像素/帧)
        x = int((index * 8) % (self.width + 40)) - 20
        left, right = max(0, x - 10), min(self.width, x + 10)
        if left < right:
            frame[:, left:right, 1] = 255
        # 顶部红色进度条指示帧号
        progress = int((index % 100) * self.width / 100)
        frame[0:12, :progress, 2] = 255
        return frame

    def report_stats(self) -> None:
        self.get_logger().info(f'published {self.published} frames')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticCameraPublisher(args.fps, args.width, args.height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
