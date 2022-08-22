from builtins import Exception
import rclpy
from rclpy.node import Node
import time
import mavros
from enum import IntEnum
from array import array
from math import *
from mavros.utils import *
from mavros import mavlink
from mavros import setpoint as SP
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamSetV2, ParamPull
from rcl_interfaces.msg import ParameterValue, ParameterEvent
from rcl_interfaces.srv import GetParameters
from mavros_msgs.msg import OverrideRCIn, RCOut, State, RCIn, ParamEvent, VfrHud, WaypointList, Waypoint
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, UInt8
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile
from mavros.base import SENSOR_QOS, PARAMETERS_QOS, STATE_QOS
from mavros.cmd import cli, pass_client
from mavros import command, system
from rclpy.logging import LoggingSeverity

MAV_TYPE_GCS = 6

class Mode_Mapping(IntEnum):
    HOLD = 4
    MANUAL = 0
    AUTO = 10
    RTL = 11
    ACRO = 1

class My_MavRover(Node):
    def __init__(self):
        super().__init__('mav_rover')
        # subscribers
        self.global_position_sub = self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.global_position_cb, qos_profile=SENSOR_QOS)
        self.rc_out_sub = self.create_subscription(RCOut, '/mavros/rc/out', self.rc_out_cb, qos_profile=10)
        self.current_state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile=STATE_QOS)
        self.parameter_events_sub = self.create_subscription(ParamEvent, '/mavros/param/event', self.parameter_event_cb, qos_profile=PARAMETERS_QOS)
        self.vfr_hud_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_cb, qos_profile=10)
        self.imu_data_sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, qos_profile=SENSOR_QOS)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_cb, qos_profile=SENSOR_QOS)
        self.gps_vel_sub = self.create_subscription(TwistStamped, '/mavros/global_position/raw/gps_vel', self.gps_vel_cb, qos_profile=SENSOR_QOS)
        self.compass_heading_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_heading_cb, qos_profile=SENSOR_QOS)
        self.waypoints_sub = self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.waypoints_cb, qos_profile=STATE_QOS)
        self.rallywaypoints_sub = self.create_subscription(WaypointList, '/mavros/rallypoint/rallypoints', self.rallywaypoints_cb, qos_profile=STATE_QOS)
        self.heartbeat_status_sub = self.create_subscription(UInt8, '/mavros/heartbeat_status', self.heartbeat_status_cb, qos_profile=STATE_QOS)



        #services
        self.arming_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.flight_mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_service = self.create_client(ParamSetV2, '/mavros/param/set')
        self.param_get_service = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.pull_parameters_service = self.create_client(ParamPull, '/mavros/param/pull')
        self.mavlink_command_service = self.create_client(CommandLong, '/mavros/cmd/command')


        # publishers
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.current_state = State
        self.vfr_hud = VfrHud
        self.imu_data = Imu
        # time.sleep(3)
        self.set_arm(state=True)
        # self.set_mode(mode=Mode_Mapping.MANUAL)
        self.set_mode(mode="MANUAL") 
        # time.sleep(5)
        # self.do_set_servo(7, 1400)
        # time.sleep(10)
        value_servo5_max = self.get_pixhawk_parameter('SERVO5_MIN')
        self.set_pixhawk_parameter('SERVO7_MAX',1900.0)
        # self.do_rc_override(servo_number=3, servo_PWM=1700)
        
    
    def set_arm(self, state: bool):
        ''' set vehicle to arm '''
        while not self.arming_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service mavros/cmd/arming not available, waiting again...')
        req = CommandBool.Request(value=state)
        resp = self.arming_service.call_async(req)
        self.get_logger().info('vehicle is armed!')

    def set_mode(self, mode: str ,):
        ''' set vehicle mode. get mode as string and change pixhawk mode '''
        while not self.flight_mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /mavros/set_mode not available, waiting again...')
        try:
            mode = mode.upper()
            mode_request = SetMode.Request(custom_mode=mode)
            mode_response = self.flight_mode_service.call_async(mode_request)
            # self.get_logger().info('--------vehicle mode changed! %s' % (mode_response))
        except Exception as e:
            self.get_logger().info("service set_mode call failed: %s could not be set. %s" % (mode,e))

        # while not self.mavlink_command_service.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service /mavros/cmd/command not available, waiting again...')
        # try:
        #     self.get_logger().info('asking to change mode')
        #     req = CommandLong.Request(command=176, param1=157.0, param2=float(mode), param3=5.0)
        #     self.mavlink_command_service.call(req)
        #     self.get_logger().info('vehicle mode changed!')
        # except Exception as e:
        #     self.get_logger().info("service set_mode call failed: %s could not be set. %s" % (mode,e))

    def do_set_servo(self, servo_number: float, servo_PWM: float):
        ''' do set servo. get servo number and servo PWM and ask the pixhawk to change the servo PWM'''
        try:
            servo_msg = CommandLong.Request(command=183, param1=float(servo_number), param2=float(servo_PWM))
            self.mavlink_command_service.call_async(servo_msg)
            self.get_logger().info("servo : %s set to PWM %s" % (servo_number, servo_PWM))
        except Exception as e:
            self.get_logger().info("do_set_servo for servo: %s failed. %s" % (servo_number,e))

    def set_pixhawk_parameter(self, param: str , value: float):
        ''' set pixhawk parameter. get param as string and value as float and chnage the pixhawk parameter'''
        while not self.param_set_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /mavros/param/set not available, waiting again...')
        try:
            param = param.upper()
            param_value = ParameterValue(type=3,double_value=float(value))
            param_request = ParamSetV2.Request(param_id=param,value=param_value)
            param_response = self.param_set_service.call_async(param_request)
            self.get_logger().info('--------vehicle param %s changed! param_value: %s' % (param,param_value))
        except Exception as e:
            self.get_logger().info("service param_set call failed: %s could not be set. %s" % (param,e))


    def get_pixhawk_parameter(self, param: str):
        ''' get parameter as string and return the parameter value from pixhawk'''
        while not self.param_get_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /mavros/param/get_parameters not available, waiting again...')
        try:
            param = param.upper()
            param_request = GetParameters.Request(names=[str(param)])
            param_response = self.param_get_service.call_async(param_request)
            rclpy.spin_until_future_complete(self, param_response)
            param_value = param_response.result().values[0]
            return self.parse_param_value(param_value)
            # self.get_logger().info('--------got vehicle param %s. param_value: %s' % (param, self.parse_param_value(param_value)))
        except Exception as e:
            self.get_logger().info("service get_parameters call failed: %s could not be get. %s" % (param,e))


    def parse_param_value(self, param_value: ParameterValue):
        ''' got parameter value and return the value according to it's type'''
        type = param_value.type
        if type == 1:
            return param_value.bool_value
        if type == 2:
            return param_value.integer_value
        if type == 3:
            return param_value.double_value
        if type == 4:
            return param_value.string_value
        else:
            raise ValueError("Unknown ParamValue from type {}".format(type))


    def pull_parameters(self,):
        ''' ask to pull all parameters from pixhawk'''
        while not self.pull_parameters_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /mavros/param/pull not available, waiting again...')
        try:
            pull_request = ParamPull.Request(force_pull=True)
            self.pull_parameters_service.call_async(pull_request)
            self.get_logger().info("pull all pixhawk parameters")
        except Exception as e:
            self.get_logger().info("service pull_parameters call failed")

    def parameter_event_cb(self, topic):
        ''' get pixhawk parameters changes as ParamEvent '''
        pass

    def do_rc_override(self, servo_number, servo_PWM):
        ''' do rc override. get servo_number and servo_PWM and set with while loop the servo PWM'''
        rc_msg = OverrideRCIn()
        rc_msg.channels[servo_number-1] = servo_PWM
        self.get_logger().info("rc_msg to pub: %s" % (rc_msg.channels))
        while True:
            self.rc_override_pub.publish(rc_msg)
            time.sleep(0.5)

    def global_position_cb(self, topic): # has position.latitude, position.longitude, position.altitude
        self.position = topic
        # self.get_logger().info("position %s" % self.position)

    def rc_out_cb(self, topic):
        self.servo_output_raw = topic.channels
        # self.get_logger().info(f"servo_output_raw: {topic.channels}")

    def state_cb(self, topic):
        ''' topic of current vehicle state. including vehicle pix mode'''
        self.current_state = topic
        self.get_logger().info("---------current mode %s" % self.current_state.mode)
        # self.get_logger().info("---------current arming status: %s" % self.current_state.armed)

    def vfr_hud_cb(self, topic): # has vfr_hud.throttle, vfr_hud.groundspeed, vfr_hud.heading
        self.vfr_hud = topic
        # self.get_logger().info("vfr_hud %s" % self.vfr_hud)

    def imu_cb(self, topic): # has imu.linear_acc, imu.angular_velocity
        self.imu = topic 
        # self.get_logger().info("imu %s" % self.imu)

    def rel_alt_cb(self, topic):
        self.rel_alt = topic.data
        # self.get_logger().info("*******rel_alt %s" % self.rel_alt)
    
    def gps_vel_cb(self, topic): # has gps_vel.twist.linear.x-z
        self.gps_vel = topic
        # self.get_logger().info("*******gps_vel %s" % self.gps_vel)

    def compass_heading_cb(self, topic): 
        self.compass_heading = topic.data
        # self.get_logger().info("*******compass_heading %s" % self.compass_heading.data)

    def waypoints_cb(self, topic):
        self.waypoints = topic
        # self.get_logger().info("*******waypoints %s" % self.waypoints)

    def rallywaypoints_cb(self, topic):
        self.rallywaypoints = topic
        # self.get_logger().info("*******rallywaypoints %s" % self.rallywaypoints)

    def heartbeat_status_cb(self, topic):
        if topic.data == MAV_TYPE_GCS:
            self.get_logger().info("got GCS heartbeat")

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

