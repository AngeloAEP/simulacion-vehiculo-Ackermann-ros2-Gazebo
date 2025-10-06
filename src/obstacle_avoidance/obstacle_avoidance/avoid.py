#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleAvoid(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoid')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/ackermann_steering_controller/reference_unstamped')
        self.declare_parameter('min_range_front', 1.0)
        self.declare_parameter('v_nominal', 0.8)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('omega_max', 0.6)
        self.declare_parameter('sector_angle_deg', 30.0)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 10)

    def sector_min(self, scan: LaserScan, center_deg: float, half_width_deg: float):
        ang_min, ang_max, ang_inc = scan.angle_min, scan.angle_max, scan.angle_increment
        def clamp(a, lo, hi): return max(lo, min(hi, a))
        center = math.radians(center_deg)
        hw = math.radians(half_width_deg)
        a0 = clamp(center - hw, ang_min, ang_max)
        a1 = clamp(center + hw, ang_min, ang_max)
        i0 = int((a0 - ang_min) / ang_inc)
        i1 = int((a1 - ang_min) / ang_inc)
        vals = [r for r in scan.ranges[i0:i1+1] if math.isfinite(r) and r > scan.range_min]
        return min(vals) if vals else float('inf')

    def on_scan(self, scan: LaserScan):
        sector = float(self.get_parameter('sector_angle_deg').value)
        front = self.sector_min(scan, 0.0,   sector)
        left  = self.sector_min(scan, +60.0, sector)
        right = self.sector_min(scan, -60.0, sector)

        min_front = float(self.get_parameter('min_range_front').value)
        v_nominal = float(self.get_parameter('v_nominal').value)
        v_min     = float(self.get_parameter('v_min').value)
        omega_max = float(self.get_parameter('omega_max').value)

        cmd = Twist()
        if front > min_front:
            cmd.linear.x = v_nominal
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = max(v_min, v_nominal * 0.3)
            turn_dir = 1.0 if left > right else -1.0
            margin = max(0.0, min_front - min(front, 5.0))
            gain = min(1.0, margin / max(1e-3, min_front))
            cmd.angular.z = turn_dir * omega_max * gain

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = SimpleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
