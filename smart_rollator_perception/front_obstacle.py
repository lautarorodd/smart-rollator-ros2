import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}


class FrontObstacle(Node):
    def __init__(self):
        super().__init__('front_obstacle')
        self.declare_parameter('front_angle_deg', 0.0)
        self.declare_parameter('half_width_deg', 20.0)
        self.declare_parameter('warn_dist_m', 1.5)
        self.declare_parameter('stop_dist_m', 0.8)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('clear_scans', 3)

        self.pub = self.create_publisher(String, 'rollator/hazard/front', 10)
        self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.state = 'SAFE'
        self.clear_count = 0
        self.scan_count = 0
        self.get_logger().info('front_obstacle gestartet, warte auf /scan')

    def on_scan(self, msg):
        p = self.get_parameter
        front = np.deg2rad(p('front_angle_deg').value)
        half = np.deg2rad(p('half_width_deg').value)
        warn = p('warn_dist_m').value
        stop = p('stop_dist_m').value
        min_r = p('min_range_m').value
        min_pts = p('min_points').value
        clear_n = p('clear_scans').value

        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        rel = np.arctan2(np.sin(angles - front), np.cos(angles - front))

        valid = (np.isfinite(ranges) & (ranges >= min_r)
                 & (ranges <= msg.range_max))
        d = ranges[valid & (np.abs(rel) <= half)]

        if int(np.sum(d < stop)) >= min_pts:
            raw = 'STOP'
        elif int(np.sum(d < warn)) >= min_pts:
            raw = 'WARN'
        else:
            raw = 'SAFE'

        if ORDER[raw] >= ORDER[self.state]:
            changed = raw != self.state
            self.state = raw
            self.clear_count = 0
        else:
            changed = False
            self.clear_count += 1
            if self.clear_count >= clear_n:
                self.state = raw
                self.clear_count = 0
                changed = True

        out = String()
        out.data = self.state
        self.pub.publish(out)

        if changed:
            self.get_logger().info(f'Zustand: {self.state}')

        self.scan_count += 1
        if self.scan_count % 10 == 0 and valid.any():
            idx = np.where(valid)[0][np.argmin(ranges[valid])]
            abs_deg = np.rad2deg(np.arctan2(
                np.sin(angles[idx]), np.cos(angles[idx])))
            self.get_logger().info(
                f'naechster Punkt: {ranges[idx]:.2f} m bei {abs_deg:.0f} Grad '
                f'(Sensorwinkel) | Zustand {self.state}')


def main():
    rclpy.init()
    node = FrontObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
