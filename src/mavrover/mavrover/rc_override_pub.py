import rclpy
from rclpy.node import Node
import time
from math import *

from mavros.utils import *
from mavros import setpoint as SP
from mavros_msgs.msg import OverrideRCIn, RCOut, State, ManualControl


class My_MavRover(Node):
    def __init__(self):
        super().__init__('rc_over_pub')
        # self.rc_override_pub = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.do_rc_override()

    def do_rc_override(self,):
        while(1):
            # rc_msg = ManualControl()
            rc_msg = OverrideRCIn()
            time.sleep(0.5)
            rc_msg.channels = [1800, 1800, 1800, 1800, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0,0,0]
            # rc_msg.y = 1000.0
            # rc_msg.z = 1000.0
            self.get_logger().info("rc_msg to pub: %s" % (rc_msg))
            self.rc_override_pub.publish(rc_msg)
            

def main(args=None):
    rclpy.init(args=args)

    minimal_mavrover = My_MavRover()


    rclpy.spin(minimal_mavrover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_mavrover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

