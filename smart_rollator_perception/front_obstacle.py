"""SAFE/WARN/STOP aus einem LaserScan (Frontsektor um front_angle_deg). Logik: front_logic.py (ohne ROS)."""
from smart_rollator_perception.front_logic import (
    evaluate_scan, next_state, parse_sectors, sectors_overlap_front, status_line)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

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
            # Eigenechos der Montage, Sensorwinkel "von..bis", z. B. "70..90,-80..-70" (leer = nichts ignorieren)
            self.declare_parameter('ignore_sectors', '')

            self.ignore = parse_sectors(self.get_parameter('ignore_sectors').value)
            front = self.get_parameter('front_angle_deg').value
            half = self.get_parameter('half_width_deg').value
            if self.ignore:
                self.get_logger().info(f'ignorierte Sektoren (Sensorwinkel): {self.ignore}')
            if sectors_overlap_front(self.ignore, front, half):
                self.get_logger().warn(
                    'ACHTUNG: ein ignorierter Sektor liegt im Frontsektor, dort werden Hindernisse uebersehen')

            self.pub = self.create_publisher(String, 'rollator/hazard/front', 10)
            self.create_subscription(
                LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

            self.state = 'SAFE'
            self.clear_count = 0
            self.scan_count = 0
            self.get_logger().info('front_obstacle gestartet, warte auf /scan')

        def on_scan(self, msg):
            p = self.get_parameter
            front = p('front_angle_deg').value
            half = p('half_width_deg').value
            res = evaluate_scan(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max,
                front, half, p('warn_dist_m').value, p('stop_dist_m').value,
                p('min_range_m').value, p('min_points').value, self.ignore)

            self.state, self.clear_count, changed = next_state(
                self.state, res['raw'], self.clear_count, p('clear_scans').value)

            out = String()
            out.data = self.state
            self.pub.publish(out)

            if changed:
                self.get_logger().info(f'Zustand: {self.state}')

            self.scan_count += 1
            if self.scan_count % 10 == 0:
                self.get_logger().info(status_line(res, self.state, front, half))

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
