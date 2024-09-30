#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time
import matplotlib.pyplot as plt
# Import message type
# from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
import numpy as np
import argparse

class OLController(Node):

    def __init__(self):
        super().__init__("open_loop_control")

        self.get_logger().info(f'ROS 2 Node is running with name open_loop_control')

        # Initialize Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_history = []
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = None


    def timer_callback(self):
        current_time = time.time()
        if self.start_time is not None:
            elapsed_time = current_time - self.start_time
            if elapsed_time < self.total_time:
                self.pub.publish(self.vel_msg)
                self.pose_history.append(self.calc_position(elapsed_time))
            else:
                self.robot_stop()
                self.get_logger().info(f'Goal position reached!')
                self.plot_trajectory()
                self.timer.cancel()
            
    def calc_position(self,elapsed_time):
        return self.vel_msg.linear.x * elapsed_time

    def robot_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.pub.publish(twist)
        self.get_logger().info('Stopping the robot')

    def move_senario1(self,distance, total_time):
        self.total_time = total_time
        self.vel_msg = Twist()
        self.vel_msg.linear.x = distance / total_time
        self.total_time = total_time
        self.start_time = time.time()
        self.get_logger().info(f'Moving with velocity: {self.vel_msg.linear.x} m/s')

        while rclpy.ok():
            rclpy.spin_once(self)

        # self.get_logger().info(f'Goal Reached: {distance} m')
        self.plot_trajectory()


    def move_senario2(self, distance, max_velocity, acceleration):

    
        self.total_time = 0
        self.vel_msg = Twist()
        position = 0

        time_to_reach_v = max_velocity / acceleration
        distance_accel = 0.5 * acceleration * time_to_reach_v**2

        if distance < 2 * distance_accel:
            time_to_reach_v = np.sqrt(distance / acceleration)
            max_velocity = acceleration * time_to_reach_v
            self.total_time = 2 * time_to_reach_v
        else:
            time_to_move_constant = (distance - 2 * distance_accel) / max_velocity
            self.total_time = time_to_reach_v + time_to_move_constant + time_to_reach_v

        # Acceleration Phase
        self.get_logger().info(f'Moving with acceleration: {acceleration} m/s^2')
        self.start_time = time.time()
        for t in np.arange(0, time_to_reach_v, 0.1):
            self.vel_msg.linear.x = acceleration * t
            self.pub.publish(self.vel_msg)
            self.pose_history.append(position)
            position += self.vel_msg.linear.x * 0.1
            time.sleep(0.1)

        # Constant Velocity Phase
        self.get_logger().info(f'Moving with constant velocity: {max_velocity} m/s')
        for t in np.arange(0, time_to_move_constant, 0.1):
            self.vel_msg.linear.x = max_velocity
            self.pub.publish(self.vel_msg)
            self.pose_history.append(position)
            position += self.vel_msg.linear.x * 0.1
            time.sleep(0.1)

        # Deceleration Phase
        self.get_logger().info(f'Moving with deceleration: {acceleration} m/s^2')
        for t in np.arange(0, time_to_reach_v, 0.1):
            self.vel_msg.linear.x = max_velocity - acceleration * t
            self.pub.publish(self.vel_msg)
            self.pose_history.append(position)
            position += self.vel_msg.linear.x * 0.1
            time.sleep(0.1)


        self.robot_stop()
        self.get_logger().info(f'Goal position reached...')
        self.plot_trajectory()


    def plot_trajectory(self):
        plt.figure()
        plt.plot(self.pose_history)
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Robot Position Over Time')
        plt.show()

def main(args=None):
    parser = argparse.ArgumentParser(description='Project 0 - Open Loop Control')
    parser.add_argument('scenario', type=int,choices=[1,2], help='scenario number to run')
    parser.add_argument('-distance', type=float, default=5.0, help='Distance to move in meters')
    parser.add_argument('-time', type=float, default=5.0, help='Time constant to move in seconds')
    parser.add_argument('-acceleration', type=float, default=0.01, help='Acceleration in m/s^2')
    args = parser.parse_args()

    rclpy.init(args=None)
    controller = OLController()

    distance = args.distance
    scenario_time = args.time
    max_velocity = 0.22
    # acceleration = 0.01

    try:
        # rclpy.spin(controller)
        if args.scenario == 1:
            controller._logger.info(f'Scenario 1: Moving {args.distance} m in {args.time} s')
            controller.move_senario1(args.distance, args.time)
        elif args.scenario == 2:
            controller._logger.info(f'Scenario 2: Moving {args.distance} m with max velocity {max_velocity} m/s and acceleration {args.acceleration} m/s^2')
            controller.move_senario2(args.distance, max_velocity, args.acceleration)

    except KeyboardInterrupt:
        pass

    finally:
        controller.destroy_node()
        rclpy.shutdown()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()