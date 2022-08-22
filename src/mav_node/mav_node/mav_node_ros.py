import time
from dronekit import Vehicle, connect, VehicleMode, LocationGlobal, LocationGlobalRelative

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
# from rclpy.
import geometry_msgs.msg


def connect_to_vehicle() -> Vehicle:
    connection_string = "udp:127.0.0.1:14550"
    print(f'Connecting to vehicle on: {connection_string}')
    vehicle = connect(connection_string, wait_ready=True)

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("MANUAL")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Armed and ready!")
    return vehicle


class MavNode(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('mav_node')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.vehicle = connect_to_vehicle()
        self.subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            self.twist_callback,
            10)
        self.subscription
        self.is_forward = True

    def twist_callback(self, twist):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving new velocity')
        rc_channel1, rc_channel3 = self.get_rc_channels(twist)
        self.vehicle.channels.overrides = {
            '1': 65535, '2': 65535, '3': rc_channel3, '4': 65535, '5': 65535, '6': 65535, '7': 65535, '8': 65535}
        self.vehicle.channels.overrides = {
            '1': rc_channel1, '2': 65535, '3': 65535, '4': 65535, '5': 65535, '6': 65535, '7': 65535, '8': 65535}

    def get_rc_channels(self, twist: geometry_msgs.msg.Twist):
        rc_channel3 = int(1500 + twist.linear.x * 500)
        rc_channel1 = int(1500 - twist.angular.z * 500)
        return rc_channel1, rc_channel3


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    mav_node = MavNode()

    # Spin the node so the callback function is called.
    rclpy.spin(mav_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mav_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
