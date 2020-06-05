#!/usr/bin/env python

# -------------------------------------------------------
# VU Autonomous Racing Cars (2020S) - TU Wien
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas@pintaric.org)
# SPDX-License-Identifier: 0BSD
# -------------------------------------------------------

from __future__ import print_function
import sys
import numpy as np
import rospy
import roslib
roslib.load_manifest('pintaric_lab4')

# import dynamic_reconfigure variables.
from pintaric_lab4.cfg import reactive_driving_Config as ConfigType

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# ========================================
# PID
# ========================================

class PID():

    def __init__(self, kp=0, ki=0, kd=0, target_output_value=0):
        self.Kp = kp # proportional gain
        self.Ki = ki # integral gain
        self.Kd = kd # derivative gain
        self.I = 0 # integral term (initialized to zero, accumulated over successive calls)
        self.target_output_value = target_output_value
        self.previous_input_value = np.nan

    def change_tuning_parameters(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.Kp = kp
        if ki is not None:
            self.Ki = ki
            self.I = 0
        if kd is not None:
            self.Kd = kd

    def calculate(self, input_value, dt):
        error = self.target_output_value - input_value # steady-state error
        if dt <= 0:
            raise ValueError('dt>0 required')
        P = self.Kp * error
        self.I += self.Ki * error * dt
        D = 0 if np.isnan(self.previous_input_value) else \
            (self.Kd * (self.previous_input_value - input_value) / dt)
        self.previous_input_value = input_value
        return (P + self.I + D) if (self.Kp + self.Ki + self.Kd > 0) else (input_value)

# ========================================
# DriveController
# ========================================

class DriveController:

    def __init__(self):

        self.pid_controller = PID()
        self.dyn_reconf_server = DynamicReconfigureServer(ConfigType, self.reconfigure)

        #self.linear_velocity = 0.0 # units: rad/s
        #self.angular_velocity = 0.0 # units: m/s
        self.heading_error = 0.0
        self.last_heading_timestamp = rospy.Time()

        self.steering_angle = 0.0
        self.vehicle_speed = 0.0
        self.target_vehicle_speed = 0.0

        self.max_steering_angle = np.deg2rad(24) # maximum (absolute) steering angle
        self.dt_threshold = 1.0 / 100.0 # run PID at 100 Hz (max.)

        #odom_topic = '/odom'
        #self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        drive_topic = '/nav'
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1000)

        heading_topic = '/reactive_driving/heading' # vehicle heading setpoint (units: rad)
        self.heading_sub = rospy.Subscriber(heading_topic, Float32, self.heading_callback)


    def reconfigure(self, config, level):
        rospy.loginfo("""Reconfigure request: {kp}, {ki}, {kd}, {speed}""".format(**config))
        self.pid_controller.change_tuning_parameters(kp=config["kp"], \
                                                     ki=config["ki"], \
                                                     kd=config["kd"])
        self.target_vehicle_speed = config["speed"]
        return config

    def heading_callback(self, data):
        self.heading_error = data.data

        has_previous_timestamp = not self.last_heading_timestamp.is_zero()
        current_timestamp = rospy.Time.now() # NOTE: Float32 message has no header/timestamp field

        if not has_previous_timestamp:
            self.last_heading_timestamp = current_timestamp
            return

        dt = current_timestamp.to_sec() - self.last_heading_timestamp.to_sec()

        if dt <= self.dt_threshold:
            return

        self.last_heading_timestamp = current_timestamp

        # Execute PID control loop
        control_value = self.pid_controller.calculate(self.heading_error, dt=dt)

        self.steering_angle = -control_value

        self.steering_angle = np.clip(self.steering_angle, \
                                      -np.abs(self.max_steering_angle), \
                                      +np.abs(self.max_steering_angle))

        self.vehicle_speed = self.target_vehicle_speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        #drive_msg.header.frame_id = "" # leave blank
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = self.vehicle_speed
        self.drive_pub.publish(drive_msg)

    #def odom_callback(self, data):
    #    self.linear_velocity = data.twist.twist.linear.x
    #    self.angular_velocity = data.twist.twist.angular.z

def main(args):
    rospy.init_node("drive_controller", anonymous=False)
    drive_controller = DriveController()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
