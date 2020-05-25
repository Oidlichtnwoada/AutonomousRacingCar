#!/usr/bin/env python

# -------------------------------------------------------
# VU Autonomous Racing Cars (2020S) - TU Wien
# -------------------------------------------------------
# Team 3: Adelmann, Brantner, Lukitsch, Pintaric
# -------------------------------------------------------

from __future__ import print_function

import os
import sys
import rospy
import roslib
roslib.load_manifest('group3_lab7')

import tf2_ros
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, PointStamped

from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

# import dynamic_reconfigure variables.
from group3_lab7.cfg import follow_me_Config as Configuration
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import cKDTree
from scipy.interpolate import interp1d

class PurePursuit:

    def __init__(self):

        self.forward_direction = True # whether or not the car should follow the tracked path in the
                                      # forward direction (i.e. follow waypoints in ascending order),
                                      # will be overwritten by self.reconfigure()

        self.lookahead_distance = 5.0 # will be overwritten by self.reconfigure()
        self.dyn_reconf_server = DynamicReconfigureServer(Configuration, self.reconfigure)

        self.last_processing_timestamp = rospy.Time()

        # self.dt_threshold = 0.0 # unthrottled
        # Example (throttled at 100Hz)
        self.dt_threshold = 1.0 / 100.0 # run at 100 Hz (max.)

        DEFAULT_WHEELBASE = 0.3302 # see simulator.yaml
        # The distance between the front and rear axle of the racecar
        self.wheelbase = rospy.get_param('wheelbase', DEFAULT_WHEELBASE)

        default_tracked_path = os.path.realpath("{}/../paths/levine.csv".format( \
            os.path.dirname(sys.argv[0])))
        self.tracked_path = np.genfromtxt(rospy.get_param('tracked_path', default_tracked_path), delimiter=',')

        waypoints = np.asarray(self.tracked_path)
        self.waypoint_tree = cKDTree(data=waypoints[:,(0,1)])

        self.waypoint_orientations = np.asanyarray(Rotation.from_euler('z', waypoints[:,2], degrees=False).as_quat())
        self.waypoint_positions = waypoints[:,(0,1)]

        # Create a latched(*) global path publisher
        # (*) http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.global_path_pub = rospy.Publisher('/follow_me/global_path', Path, queue_size=1, latch=True)

        self.global_path_msg = self.generate_path_msg(self.tracked_path, self.forward_direction)
        self.global_path_pub.publish(self.global_path_msg)
        self.global_path_publication_frequency = 1.0 # Hz
        self.t_last_global_path_publication = rospy.Time.now()

        # Create local path publisher
        self.local_path_pub = rospy.Publisher('/follow_me/local_path', Path, queue_size=1000)


        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.linear_velocity = 0
        self.angular_velocity = 0

        self.car_pose_is_known = False
        self.car_position = np.array((0,0), dtype=np.float)
        self.car_orientation = Rotation.from_quat((0, 0, 0, 1)).as_quat()
        self.car_heading = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.closest_waypoint_pub = rospy.Publisher('/follow_me/closest_waypoint', PoseStamped, queue_size=1000) # for RViz only
        self.goal_pub = rospy.Publisher('/follow_me/goal', PoseStamped, queue_size=1000)

    def generate_path_msg(self, path, forward_direction=True):
        path_msg = Path() # this won't change over the lifetime of the node
        t_now = rospy.Time.now()
        path_msg.header.stamp = t_now
        path_msg.header.frame_id = "map"

        for i in range(len(path)+1):
            p = PoseStamped()
            p.header.stamp = t_now
            p.header.frame_id = "map" # redundant
            j = np.mod(i,len(path))
            p.pose.position.x = path[j, 0]
            p.pose.position.y = path[j, 1]
            yaw = path[j, 2] if self.forward_direction else path[j, 2] + np.pi
            rotation = Rotation.from_euler('z', yaw, degrees=False)
            q = rotation.as_quat() # scalar-last (x, y, z, w) format
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            if forward_direction:
                path_msg.poses.append(p)
            else:
                path_msg.poses.prepend(p)
        return path_msg

    def reconfigure(self, config, level):
        rospy.loginfo("""[follow_me] reconfigure request: {lookahead_distance} {forward_direction}""".format(**config))
        self.lookahead_distance = config["lookahead_distance"]
        forward_direction = config["forward_direction"]

        if self.forward_direction != forward_direction:
            self.forward_direction = forward_direction
            self.global_path_msg = self.generate_path_msg(self.tracked_path, self.forward_direction)
            if not self.forward_direction:
                print("WARNING: Option [forward_direction=False] has not been tested!")

        assert(self.lookahead_distance > 0.0)
        return config

    def publish_waypoint_pose(self, publisher, waypoint):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = waypoint[0]
        pose_msg.pose.position.y = waypoint[1]
        yaw = waypoint[2] if self.forward_direction else waypoint[2]+np.pi
        rotation = Rotation.from_euler('z', yaw, degrees=False).as_quat()
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        publisher.publish(pose_msg)

    def compute_closest_waypoint(self, publish=True):
        distance, index = self.waypoint_tree.query(self.car_position)
        closest_waypoint = self.tracked_path[index,:]

        if publish:
            self.publish_waypoint_pose(self.closest_waypoint_pub, closest_waypoint)
        return index, distance, closest_waypoint

    def compute_goal(self, closest_waypoint_index, publish=True):
        waypoints = np.asarray(self.tracked_path)
        number_of_waypoints = waypoints.shape[0]
        index_increment = (1) if self.forward_direction else (-1)
        index = closest_waypoint_index

        best_distance_so_far = -np.Inf
        best_index_so_far = index

        interpolated_position = np.full(shape=(2,), fill_value=np.nan, dtype=np.float)
        interpolated_yaw = np.nan

        while True:
            index = np.mod(index + index_increment, number_of_waypoints)
            if index == closest_waypoint_index:
                break

            distance = np.linalg.norm(self.car_position - waypoints[index,(0,1)])
            assert(np.isfinite(distance))

            if distance > self.lookahead_distance:

                if not np.isfinite(best_distance_so_far):
                    # Workaround (when waypoints are far from each other):
                    # select the next waypoint as our goal
                    interpolated_position = waypoints[index, 0:2]
                    interpolated_yaw = waypoints[index, 2]
                    break

                # interpolate between [best_index_so_far, index]
                distance_1 = np.abs(self.lookahead_distance - distance)
                distance_0 = np.abs(self.lookahead_distance - best_distance_so_far)
                assert(distance_0 + distance_1 != 0.0)
                weight_1 = distance_0 / (distance_0 + distance_1)
                weight_0 = 1.0 - weight_1

                # interpolate position
                position_interpolator = interp1d([0.0, 1.0], waypoints[(best_index_so_far, index),0:2].T)
                interpolated_position = position_interpolator(weight_1)

                # interpolate orientation
                slerp = Slerp(times=[0.0, 1.0], rotations= \
                    Rotation.from_quat(self.waypoint_orientations[(best_index_so_far, index), :]))

                interpolated_rotation = slerp([weight_1])
                interpolated_yaw = interpolated_rotation.as_euler('zxy', degrees=False)[0][0]

                break
            elif distance > best_distance_so_far:
                best_distance_so_far = distance
                best_index_so_far = index

        goal = np.array((interpolated_position[0],
                         interpolated_position[1],
                         interpolated_yaw), dtype=np.float)

        if publish:
            self.publish_waypoint_pose(self.goal_pub, goal)

        return goal


    def odom_callback(self, data):
        self.linear_velocity = data.twist.twist.linear.x
        self.angular_velocity = data.twist.twist.angular.z

        has_previous_timestamp = not self.last_processing_timestamp.is_zero()
        current_timestamp = rospy.Time.now() # NOTE: Float32 message has no header/timestamp field

        if not has_previous_timestamp:
            self.last_processing_timestamp = current_timestamp
            return

        dt = current_timestamp.to_sec() - self.last_processing_timestamp.to_sec()

        if dt < self.dt_threshold:
            return

        self.last_processing_timestamp = current_timestamp

        try:
            transform = self.tf_buffer.lookup_transform(target_frame="map", source_frame="base_link", time=rospy.Time(0))
            self.car_position = [transform.transform.translation.x, transform.transform.translation.y]
            rotation = Rotation.from_quat([transform.transform.rotation.x,
                                           transform.transform.rotation.y,
                                           transform.transform.rotation.z,
                                           transform.transform.rotation.w])
            self.car_orientation = rotation.as_quat()
            self.car_heading = rotation.as_euler('zxy', degrees=False)[0]
            self.car_pose_is_known = True
        except:
            self.car_pose_is_known = False

        if self.car_pose_is_known:
            closest_waypoint_index, _, _ = self.compute_closest_waypoint(publish=True)
            goal = self.compute_goal(closest_waypoint_index, publish=True)

    def message_loop(self):
        while not rospy.is_shutdown():
            t_current = rospy.Time.now()
            if (t_current.to_sec() - self.t_last_global_path_publication.to_sec()) > \
                    (1.0 / self.global_path_publication_frequency):
                self.global_path_pub.publish(self.global_path_msg)
                self.t_last_global_path_publication = t_current
            rospy.sleep(0.1)

def main(args):
    rospy.init_node("follow_me", anonymous=False)
    pure_pursuit = PurePursuit()
    try:
        pure_pursuit.message_loop()
    except:
        # TODO: error handling
        raise

if __name__=='__main__':
    main(sys.argv)
