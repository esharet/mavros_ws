import rclpy
from rclpy.node import Node
import mavros
from mavros import mavlink

from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink

"""
This script connects to mavlink, convert a msg to ros and turn it back to the original msg
"""

class Mavlink_Msgs(Node):
    def __init__(self):
        super().__init__('mav_rover')

    def run(self):
        try:
            conn = mavutil.mavlink_connection("localhost:14551")
            conn.wait_heartbeat(timeout=30) 
        except:
            # self.get_logger().warn('Cannot connect to ----- %s' % connection_string)
            pass
        while True:
            msg = conn.recv_msg()
            if msg:
                msg.pack(conn.mav) # <----- new line to fix crc issue
                # print(msg)
                rosmsg = mavlink.convert_to_rosmsg(msg)
                # print(rosmsg)

                # Now we turn it back
                bytes = mavlink.convert_to_bytes(rosmsg)
                original_message = conn.mav.decode(bytes)
                print(original_message)

def main(args=None):
    rclpy.init(args=args)

    mavlink_msgs = Mavlink_Msgs()
    mavlink_msgs.run()

    rclpy.spin(mavlink_msgs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    # mavlink_msgs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()