"""Tiny CLI to publish std_msgs/Bool on /soft_estop.

Usage:

    ros2 run rover_hardware estop_cli stop     # engage (False)
    ros2 run rover_hardware estop_cli release  # release (True)
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main(argv=None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or argv[0] not in ('stop', 'release'):
        print(__doc__, file=sys.stderr)
        sys.exit(2)

    engage = argv[0] == 'stop'

    rclpy.init()
    node = Node('estop_cli')
    pub = node.create_publisher(Bool, 'soft_estop', 1)
    msg = Bool()
    msg.data = not engage  # False -> engaged, True -> released
    # Publish a few times then exit so late subscribers catch it.
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
