from cmath import nan
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

from adrclib import ADRC

import sys

class LoadCenteredControl(Node):
    def __init__(self):
        super().__init__('load_centered_control')

        # Variables
        self.timer_period = 1/300  # seconds
        self.timestamp_ = 0
        self.offboard_setpoint_counter_ = 0 # counter for the number of setpoints sent
        self.load_pos_ = np.empty(3)
        self.load_vel_ = np.empty(3)
        self.adrc_warmup_counter = 0

        # Initialize controllers
        #self.vx_adrc = ADRC(2, 1., -4/2, 3, 1/timer_period)
        self.vy_adrc = ADRC(1, 2, -4/10, 10, 1/self.timer_period, saturation=(-3.,3.))
        self.vz_adrc = ADRC(2, 20., -4/2, 5, 1/self.timer_period, saturation=(-3.,3.))

        self.sp_x = (0.5 - np.random.random_sample())*50
        self.sp_y = (0.5 - np.random.random_sample())*50
        self.sp_z = -(np.random.random_sample())*25 - 2.5
        print("Going to:", self.sp_x, self.sp_y, self.sp_z)


        # Subscribers
        self.load_pos_sub_ = self.create_subscription(Odometry, 
            'slung/load',
            self.load_pos_sub_callback, qos_profile_sensor_data)
        self.timesync_sub_ = self.create_subscription(
            Timesync, "fmu/timesync/out", self.timesync_callback, 10)

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, "fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, "fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "fmu/vehicle_command/in", 10)
        self.state_publisher_ = self.create_publisher(
            Vector3Stamped, "adrc/vz/states", 10)
        
        # Timers
        self.timer_ = self.create_timer(
            self.timer_period, self.timer_callback)

    # @brief Execute based on timer. MAIN LOOP.
    def timer_callback(self):

        self.vz_adrc.estimate(self.load_vel_[2])
        self.vy_adrc.estimate(self.load_vel_[0])

        if (self.offboard_setpoint_counter_ == 100):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
            self.arm()
        
        # Allow estimator to warm up
        warmup_time = 1.0 # seconds
        if self.adrc_warmup_counter < warmup_time*1/self.timer_period:
            self.vz_adrc.u = np.array([[0.0]])
            self.vy_adrc.u = np.array([[0.0]])
            self.adrc_warmup_counter += 1
        else:
            self.vz_adrc.control(-0.5)
            self.vy_adrc.control(-1.)

        #print("gt:",self.load_vel_[2],"\t est:",self.vz_adrc.z[0,0],"\t u:",self.vz_adrc.u[0,0])              
        # offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(vy=self.vy_adrc.u[0,0], vz=self.vz_adrc.u[0,0])
        if (self.offboard_setpoint_counter_ < 101):
            self.offboard_setpoint_counter_ += 1
        self.publish_state()

    # @brief Store load position and velocity
    def load_pos_sub_callback(self, msg):
        self.load_pos_[0] = msg.pose.pose.position.x
        self.load_pos_[1] = msg.pose.pose.position.y
        self.load_pos_[2] = msg.pose.pose.position.z
        self.load_vel_[0] = msg.twist.twist.linear.x
        self.load_vel_[1] = msg.twist.twist.linear.y
        self.load_vel_[2] = -msg.twist.twist.linear.z

    # @brief Store timestamp from PX4
    def timesync_callback(self, msg):
        self.timestamp_ = msg.timestamp


    # @brief  Send a command to Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        #self.get_logger().info("Arm command send")

    # @brief  Send a command to Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    # @brief Publish the offboard control mode.
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)

    # @ brief Publish a trajectory setpoint
    def publish_trajectory_setpoint(self, x: float = nan, y: float = nan, z: float = nan, vx: float = nan, vy: float = nan, vz: float = nan):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_
        msg.x = np.NaN
        msg.y = np.NaN
        msg.z = np.NaN
        msg.vx = 0.0
        msg.vy = vy
        msg.vz = 0.0
        msg.yaw = -3.14  # [-PI:PI]
        #msg.vz = nan#float(vz)
        self.trajectory_setpoint_publisher_.publish(msg)

    #  @ brief Publish vehicle commands
    #  @ param command   Command code(matches VehicleCommand and MAVLink MAV_CMD codes)
    #  @ param param1    Command parameter 1
    #  @ param param2    Command parameter 2
    def publish_vehicle_command(self, command, param1, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def publish_state(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = self.vy_adrc.z[0,0]
        msg.vector.y = self.vy_adrc.z[1,0]
        #msg.vector.z = self.vy_adrc.z[2,0]
        self.state_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    load_centered_controller = LoadCenteredControl()
    rclpy.spin(load_centered_controller)

    load_centered_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()