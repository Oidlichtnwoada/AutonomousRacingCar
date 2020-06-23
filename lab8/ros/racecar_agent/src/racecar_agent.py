#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ---------------------------------------------------------------------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# ---------------------------------------------------------------------------------------------------------------------

import sys
import os
from tensorforce import Agent
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import tensorflow as tf

class RacecarAgent:

    def __init__(self, model_path):

        directory = os.path.dirname(model_path)
        filename_root, _ = os.path.splitext(os.path.basename(model_path))
        self.agent = Agent.load(directory=directory, filename=filename_root, format='pb-actonly')
        self.number_of_rays = self.agent.states_spec['state']['shape'][0]
        self.agent_internals = self.agent.initial_internals()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=1)


    def lidar_callback(self, data):
        
        ranges = np.asarray(data.ranges)
        step_size = ranges.shape[0] // self.number_of_rays
        observation = ranges[::step_size]
        assert(observation.shape[0] == self.number_of_rays)
        actions, self.internals = self.agent.act(states=dict(state=observation), internals=self.agent_internals, evaluation=True)
        steering_angle = actions.flatten()[0]
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = 1.0

        try:
            self.drive_pub.publish(drive_msg)
        except rospy.ROSException:
            exit(0)


def main(args):
    verbose = True
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0' if verbose else '3'  # 0: DEBUG, 1: INFO, 2: WARNING, 3: ERROR
    tf.get_logger().setLevel('DEBUG' if verbose else 'ERROR')  # {DEBUG, INFO, WARN, ERROR, FATAL}
    MEMORY_LIMIT = 1024 * 2 # Limit GPU memory usage to 2GB
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if gpus:
        try:
            tf.config.experimental.set_virtual_device_configuration(gpus[0], [
                tf.config.experimental.VirtualDeviceConfiguration(memory_limit=MEMORY_LIMIT)])
        except RuntimeError as e:
            print(e)

    rospy.init_node("racecar_agent", anonymous=True)
    default_model_path = os.path.realpath("{}/../models/model".format(os.path.dirname(sys.argv[0])))
    model_path = rospy.get_param('model', default_model_path)
    racecar_agent = RacecarAgent(model_path)
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
