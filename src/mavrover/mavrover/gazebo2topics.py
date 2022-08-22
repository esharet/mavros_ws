import rclpy
from rclpy.node import Node
import time
from enum import IntEnum
import threading
from math import *
import asyncio

from pygazebo import pygazebo
from pygazebo import msg
import sensor_msgs
from rclpy.logging import LoggingSeverity
import geometry_msgs.msg

GZ_IMU_TOPIC ="/gazebo/default/rover_ardupilot/rover_ardupilot/imu_link/imu_sensor/imu"
class Gazebo2Topics(Node):
    def __init__(self):
        super().__init__('gazebo2topics')
        loop = None
        
    def cb(self, data):
        imu_msg = msg.imu_pb2.IMU()
        imu_msg.ParseFromString(data)
        # self.get_logger().info('Data from IMU: %s' % imu_msg)

    async def publish_loop(self,):
        manager = await pygazebo.connect()
        subscriber = await manager.subscribe(GZ_IMU_TOPIC, 'gazebo.msgs.IMU', self.cb)
        while True:
            await asyncio.sleep(1)


    def run(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = Gazebo2Topics()
    node.loop = asyncio.get_event_loop()
    node.loop.run_until_complete(node.publish_loop())

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

