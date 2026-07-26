#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseRecorder(Node):
    def __init__(self, output_file):
        super().__init__('pose_recorder')
        self.output_file = output_file
        self.file = open(output_file, 'w', encoding='utf-8')
        
        # Write clean TUM-compliant header
        self.file.write("# timestamp x y z q_x q_y q_z q_w\n")
        self.valid_lines = 0
        
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/ov_msckf/poseimu', 
            self.cb, 
            10
        )
        self.get_logger().info(f"Recording and sanitizing poses to {output_file}...")

    def cb(self, msg):
        try:
            # Extract timestamp and pose components
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            
            # Sanitization Check 1: Ensure all values are finite numbers (prevents writing NaN/Inf crashes)
            values = [t, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(v) for v in values):
                return  # Silently drop corrupted frames
            
            # Sanitization Check 2: Format strictly to 8 space-separated columns, stripping null bytes
            line = f"{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n"
            clean_line = line.replace('\x00', '').replace(',', ' ').replace('"', '').replace("'", '')
            
            self.file.write(clean_line)
            self.valid_lines += 1
            
        except Exception as e:
            self.get_logger().warn(f"Failed to process pose message: {e}")

    def close_file(self):
        if not self.file.closed:
            self.file.close()
        print(f"[pose_recorder]: File closed safely. Saved {self.valid_lines} perfectly formatted lines.")

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: python3 record_poses.py <output_file.txt>")
        return
    
    node = PoseRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()