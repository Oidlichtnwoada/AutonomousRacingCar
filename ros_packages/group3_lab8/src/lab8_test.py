#!/usr/bin/env python
# -*- coding: utf-8 -*-

from stable_baselines import PPO2

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=1)

        self.model = PPO2.load("../params/deepqn_lab8")

    def lidar_callback(self, data):
        actions = model.predict(np.array(data.ranges, dtype=float32))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = actions[1]
        drive_msg.drive.speed = actions[0]
        # publish data and catch possible "Topic-already-closed"-Exception when ROS is shutdown --> exit this node
        try:
            self.drive_pub.publish(drive_msg)
        except rospy.ROSException:
            exit(0)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)