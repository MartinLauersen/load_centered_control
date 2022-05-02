import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry

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
        timer_period = 0.01  # seconds
        self.timestamp_ = 0
        self.offboard_setpoint_counter_ = 0 # counter for the number of setpoints sent
        self.load_pos_ = np.empty(3)
        self.load_vel_ = np.empty(3)

        # Initialize controllers
        self.vx_adrc = ADRC(1)
        self.vy_adrc = ADRC()
        self.vz_adrc = ADRC()

        


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
        
        # Timers
        #self.timer_ = self.create_timer(
        #    timer_period, self.timer_callback)

    # @brief Execute based on timer. MAIN LOOP.
    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
            self.arm()
        # offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    # @brief Store load position and velocity
    def load_pos_sub_callback(self, msg):
        self.load_pos_[0] = msg.pose.pose.position.x
        self.load_pos_[1] = msg.pose.pose.position.y
        self.load_pos_[2] = msg.pose.pose.position.z
        self.load_vel_[0] = msg.twist.twist.linear.x
        self.load_vel_[1] = msg.twist.twist.linear.y
        self.load_vel_[2] = msg.twist.twist.linear.z

    # @brief Store timestamp from PX4
    def timesync_callback(self, msg):
        self.timestamp_ = msg.timestamp


    # @brief  Send a command to Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

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
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_
        msg.x = -30.0
        msg.y = -10.0
        msg.z = -5.0
        msg.yaw = -3.14  # [-PI:PI]
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

def main(args=None):
    rclpy.init(args=args)
    load_centered_controller = LoadCenteredControl()
    rclpy.spin(load_centered_controller)

    load_centered_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()