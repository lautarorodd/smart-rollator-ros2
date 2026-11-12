"""SAFE/WARN/STOP aus einem LaserScan (Frontsektor um front_angle_deg oder Gang, siehe front_logic.py, ohne ROS).

Ausgaenge: rollator/hazard/front  SAFE | WARN | STOP
           rollator/hazard/side   NONE | links | rechts | beide  (nur im Gang-Modus, corridor_half_width_m > 0)
"""
from smart_rollator_perception.front_logic import (
    evaluate_scan, next_side, next_state, parse_sectors, sectors_overlap_front, status_line)


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
            # Gang statt Kegel (0 = Kegel wie bisher). Halbe Gangbreite = halbe Rollatorbreite + Rand (m),
            # Seitenstreifen daneben (Breite, Laenge in m), seitlicher Versatz des LiDAR zur Rollatormitte (links +)
            self.declare_parameter('corridor_half_width_m', 0.0)
            self.declare_parameter('side_width_m', 0.35)
            self.declare_parameter('side_length_m', 0.8)
            self.declare_parameter('lidar_y_m', 0.0)

            self.ignore = parse_sectors(self.get_parameter('ignore_sectors').value)
            front = self.get_parameter('front_angle_deg').value
            half = self.get_parameter('half_width_deg').value
            if self.ignore:
                self.get_logger().info(f'ignorierte Sektoren (Sensorwinkel): {self.ignore}')
            if sectors_overlap_front(self.ignore, front, half):
                self.get_logger().warn(
                    'ACHTUNG: ein ignorierter Sektor liegt im Frontsektor, dort werden Hindernisse uebersehen')

            self.pub = self.create_publisher(String, 'rollator/hazard/front', 10)
            self.pub_side = self.create_publisher(String, 'rollator/hazard/side', 10)
            corr = self.get_parameter('corridor_half_width_m').value
            if corr > 0:
                self.get_logger().info(
                    f'Gang-Modus: halbe Breite {corr:.3f} m, Seitenstreifen '
                    f'{self.get_parameter("side_width_m").value:.2f} x {self.get_parameter("side_length_m").value:.2f} m, '
                    f'LiDAR-Versatz {self.get_parameter("lidar_y_m").value:+.3f} m')
            self.create_subscription(
                LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

            self.state = 'SAFE'
            self.clear_count = 0
            self.side = None
            self.side_clear = 0
            self.scan_count = 0
            self.get_logger().info('front_obstacle gestartet, warte auf /scan')

        def on_scan(self, msg):
            p = self.get_parameter
            front = p('front_angle_deg').value
            half = p('half_width_deg').value
            res = evaluate_scan(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max,
                front, half, p('warn_dist_m').value, p('stop_dist_m').value,
                p('min_range_m').value, p('min_points').value, self.ignore,
                corridor_half_m=p('corridor_half_width_m').value, side_width_m=p('side_width_m').value,
                side_length_m=p('side_length_m').value, lidar_y_m=p('lidar_y_m').value)

            self.state, self.clear_count, changed = next_state(
                self.state, res['raw'], self.clear_count, p('clear_scans').value)

            out = String()
            out.data = self.state
            self.pub.publish(out)

            if changed:
                self.get_logger().info(f'Zustand: {self.state}')

            self.side, self.side_clear, side_changed = next_side(
                self.side, res['side'], self.side_clear, p('clear_scans').value)
            out_side = String()
            out_side.data = self.side or 'NONE'
            self.pub_side.publish(out_side)
            if side_changed:
                self.get_logger().info(f'Seite: {self.side or "frei"}')

            self.scan_count += 1
            if self.scan_count % 10 == 0:
                self.get_logger().info(status_line(res, self.state, front, half, p('corridor_half_width_m').value))

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
