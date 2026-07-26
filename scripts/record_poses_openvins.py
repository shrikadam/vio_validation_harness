#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseRecorder(Node):
    def __init__(self, output_file):
        super().__init__('pose_recorder')
        self.file = open(output_file, 'w')
        self.file.write("# timestamp x y z q_x q_y q_z q_w\n")
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/ov_msckf/poseimu', 
            self.cb, 
            10)
        self.get_logger().info(f"Recording poses to {output_file}...")

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.file.write(f"{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")

    def close_file(self):
        # Dedicated method to close the file without using ROS loggers
        self.file.close()

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: python3 record_poses.py <output_file.txt>")
        return
    
    node = PoseRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Gracefully swallow the exact exception ROS 2 throws on SIGINT
        pass
    finally:
        node.close_file()
        print("[pose_recorder]: File closed safely.") # Standard print avoids context crashes
        node.destroy_node()
        # Only shutdown if the context is still alive
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()