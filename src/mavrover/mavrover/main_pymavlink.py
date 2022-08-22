import rclpy
from rclpy.node import Node
import time
from enum import IntEnum
from pymavlink import mavutil
import threading
from math import *

import sensor_msgs
from rclpy.logging import LoggingSeverity
from mavros_msgs.msg import OverrideRCIn
import geometry_msgs.msg


class Mode_Mapping(IntEnum):
    HOLD = 4
    MANUAL = 0
    AUTO = 10
    RTL = 11
    ACRO = 1

class My_MavRover(Node):
    def __init__(self):
        super().__init__('mav_rover')
        self.connection_string = "localhost:14551"
        self._rc_values = [65535 for _ in range(8)]
        # self.global_position_sub = self.create_subscription(mavros.global_position.NavSatFix, '/mavros/global_position/raw/fix', self.global_position_cb, qos_profile=SENSOR_QOS)
        # self.current_state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile=10)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # self.mavlink_command_service = self.create_client(CommandLong, '/mavros/cmd/command')
        # self.get_logger().info('System of mavros ----- %s!' % dir(SetMode.Request))

    def run(self):
        self.vehicle = self.connect(self.connection_string)
        self.set_arm()
        self.set_mode("MANUAL")

        self._t_rc_send = threading.Thread(target=self._send_rc_periodically, daemon=True, name="rc periodically")
        self._t_rc_send.start()
        self.teleop_sub = self.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            self.twist_callback,
            10)

    def twist_callback(self, twist):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving new velocity')
        rc_channel1, rc_channel3 = self.get_rc_channels(twist)
        self._rc_values[0] = rc_channel1
        self._rc_values[2] = rc_channel3

    def get_rc_channels(self, twist: geometry_msgs.msg.Twist):
        rc_channel3 = int(1500 + twist.linear.x * 500)
        rc_channel1 = int(1500 - twist.angular.z * 500)
        return rc_channel1, rc_channel3

    def connect(self, connection_string):
        try:
            vehicle = mavutil.mavlink_connection(connection_string)
            vehicle.wait_heartbeat(timeout=30)
            self.target_system = vehicle.target_system
            self.target_component = vehicle.target_component
            self.get_logger().info('Connected to ----- %s' % connection_string)
        except:
            self.get_logger().warn('Cannot connect to ----- %s' % connection_string)
        return vehicle

    def set_arm(self,):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,
            1, 0, 0, 0, 0, 0, 0)
        self.get_logger().info('Vehicle is armed')


    def set_mode(self, mode: str):
        try:
            self.vehicle.set_mode(mode)
            self.get_logger().info('Vehicle mode changed to %s' % mode)

        except:
            self.get_logger().warn('Cannot change mode to %s' % mode)


    def _send_rc_periodically(self,):
        while True:
            rc_msg = OverrideRCIn(channels=self._rc_values)
            self.vehicle.mav.rc_channels_override_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                *self._rc_values)
            time.sleep(0.01)
            self.rc_override_pub.publish(rc_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_mavrover = My_MavRover()
    minimal_mavrover.run()

    rclpy.spin(minimal_mavrover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_mavrover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

