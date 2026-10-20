import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ORDER = {'SAFE': 0, 'WARN': 1, 'STOP': 2}
FILES = {'WARN': 'warn.wav', 'STOP': 'stop.wav'}


class HazardSpeaker(Node):
    def __init__(self):
        super().__init__('hazard_speaker')
        self.declare_parameter('sound_dir',
                               os.path.expanduser('~/rollator_sounds'))
        self.declare_parameter('repeat_warn_s', 4.0)
        self.declare_parameter('repeat_stop_s', 2.0)

        self.create_subscription(
            String, 'rollator/hazard/front', self.on_state, 10)

        self.proc = None
        self.last_state = 'SAFE'
        self.last_play = 0.0
        self.get_logger().info('hazard_speaker gestartet')

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def play(self, state):
        path = os.path.join(
            self.get_parameter('sound_dir').value, FILES[state])
        if not os.path.isfile(path):
            self.get_logger().error(
                f'Audiodatei fehlt: {path}', throttle_duration_sec=5.0)
            return
        if self.busy():
            self.proc.terminate()
        self.proc = subprocess.Popen(['aplay', '-q', path])
        self.last_play = time.monotonic()
        self.get_logger().info(f'spiele {state}')

    def on_state(self, msg):
        state = msg.data
        if state not in ORDER:
            return
        if state != 'SAFE':
            now = time.monotonic()
            escalated = ORDER[state] > ORDER[self.last_state]
            key = 'repeat_stop_s' if state == 'STOP' else 'repeat_warn_s'
            interval = self.get_parameter(key).value
            if escalated or (not self.busy()
                             and now - self.last_play >= interval):
                self.play(state)
        self.last_state = state


def main():
    rclpy.init()
    node = HazardSpeaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
