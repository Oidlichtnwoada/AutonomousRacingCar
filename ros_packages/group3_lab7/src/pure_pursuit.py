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
from group3_lab7.cfg import pure_pursuit_Config as Configuration
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import cKDTree
from scipy.interpolate import interp1d

import threading

class PurePursuit:

    def __init__(self):

        self.path_lock = threading.RLock() # Initialize the lock before anything else!

        self.forward_direction = True
        self.lookahead_distance = 2.0 # will be overwritten by self.reconfigure()

        # TODO: expose dt_threshold as node parameter
        self.dt_threshold = 1.0 / 50.0 # run at 50 Hz (max.)
        self.last_processing_timestamp = rospy.Time()

        DEFAULT_WHEELBASE = 0.3302 # see simulator.yaml
        # The distance between the front and rear axle of the racecar
        self.wheelbase = rospy.get_param('wheelbase', DEFAULT_WHEELBASE)

        self.tracked_path_is_valid = False
        self.waypoint_tree = None
        self.waypoint_orientations = None
        self.waypoints = None

        self.tracked_path_sub = rospy.Subscriber("/pure_pursuit/tracked_path", Path, self.tracked_path_callback)
        #self.tracked_path_sub = rospy.Subscriber("/motion_planner/path_to_goal", Path, self.tracked_path_callback) # <-- use this for debugging!

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.linear_velocity = 0
        self.angular_velocity = 0

        self.car_pose_is_known = False
        self.car_position = np.array((0,0), dtype=np.float)
        self.car_orientation = Rotation.from_quat((0, 0, 0, 1)).as_quat()
        self.car_heading = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1000) # for RViz
        self.track_arc_pub = rospy.Publisher('/pure_pursuit/track_arc', Marker, queue_size=1000) # for RViz
        self.commanded_heading_pub = rospy.Publisher('/pure_pursuit/commanded_heading', PoseStamped, queue_size=1000) # for RViz
        self.commanded_heading_angle_pub = rospy.Publisher('/pure_pursuit/commanded_heading_angle', Float32, queue_size=1000) # for the drive controller

        # Initialize DynamicReconfigureServer last!
        self.dyn_reconf_server = DynamicReconfigureServer(Configuration, self.reconfigure)

    def reconfigure(self, config, level):
        rospy.loginfo("""[pure_pursuit] reconfigure request: {lookahead_distance}""".format(**config))
        self.path_lock.acquire(blocking=1)
        self.lookahead_distance = config["lookahead_distance"]
        assert(self.lookahead_distance > 0.0)
        self.path_lock.release()
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

    def compute_closest_waypoint(self):
        distance, index = self.waypoint_tree.query(self.car_position)
        closest_waypoint = self.waypoints[index,:]
        return index, distance, closest_waypoint

    def compute_goal(self, closest_waypoint_index, publish=True):
        waypoints = self.waypoints
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
                slerp = Slerp(times=[0.0, 1.0], rotations=
                    Rotation.from_quat(self.waypoint_orientations[(best_index_so_far, index), :]))

                interpolated_rotation = slerp([weight_1])
                interpolated_yaw = interpolated_rotation.as_euler('zxy', degrees=False)[0][0]

                break
            elif distance > best_distance_so_far:
                best_distance_so_far = distance
                best_index_so_far = index

        if np.isnan(interpolated_position).any() or best_distance_so_far <= self.lookahead_distance:
            goal = waypoints[best_index_so_far,:]
        else:
            goal = np.array((interpolated_position[0],
                             interpolated_position[1],
                             interpolated_yaw), dtype=np.float)

        if publish:
            assert(not np.isnan(goal).any())
            self.publish_waypoint_pose(self.goal_pub, goal)

        return goal

    def tracked_path_callback(self, data):
        number_of_waypoints = len(data.poses)
        waypoints = np.empty(shape=(number_of_waypoints,3))
        waypoint_orientations = np.empty(shape=(number_of_waypoints,4))

        for i, pose in enumerate(data.poses):
            rotation = Rotation.from_quat([pose.pose.orientation.x,
                                           pose.pose.orientation.y,
                                           pose.pose.orientation.z,
                                           pose.pose.orientation.w])
            waypoint_orientations[i,:] = rotation.as_quat()
            z_angle = rotation.as_euler('zxy', degrees=False)[0]
            waypoints[i,:] = [pose.pose.position.x, pose.pose.position.y, z_angle]

        waypoint_tree = cKDTree(data=waypoints[:,(0,1)])

        self.path_lock.acquire(blocking=1)
        self.waypoints = waypoints
        self.waypoint_orientations = waypoint_orientations
        self.waypoint_tree = waypoint_tree
        self.tracked_path_is_valid = True
        self.path_lock.release()


    def publish_commanded_heading(self, angle):
        # Publish commanded_heading (for the drive controller)
        float_msg = Float32()
        float_msg.data = angle
        self.commanded_heading_angle_pub.publish(float_msg)

        # Publish commanded_heading (for RViz)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = 0
        pose_msg.pose.position.y = 0
        rotation = Rotation.from_euler('z', angle, degrees=False).as_quat()
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        self.commanded_heading_pub.publish(pose_msg)


    def odom_callback(self, data):
        self.linear_velocity = data.twist.twist.linear.x
        self.angular_velocity = data.twist.twist.angular.z

        has_previous_timestamp = not self.last_processing_timestamp.is_zero()
        current_timestamp = rospy.Time.now() # NOTE: Float32 message has no header/timestamp field

        if not has_previous_timestamp:
            self.last_processing_timestamp = current_timestamp
            return

        dt = current_timestamp.to_sec() - self.last_processing_timestamp.to_sec()

        if dt <= self.dt_threshold:
            return

        self.path_lock.acquire(blocking=1)
        if not self.tracked_path_is_valid:
            self.path_lock.release()
            return
        self.path_lock.release()

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

            self.path_lock.acquire(blocking=1)
            closest_waypoint_index, _, _ = self.compute_closest_waypoint()
            goal = self.compute_goal(closest_waypoint_index, publish=True)
            assert(not np.isnan(goal).any())
            self.path_lock.release()

            #  The control law is based on fitting a semi-circle through the vehicle's current configuration
            #  to a point on the reference path ahead of the vehicle by a distance L called the lookahead distance.
            #  The circle is defined as passing through the position of the car and the point on the path ahead of
            #  the car by one lookahead distance with the circle tangent to the car's heading. [Paden et al., 2016]
            #
            #  For details, see: https://arxiv.org/abs/1604.07446

            def cartesian_to_polar(xy):
                theta = np.arctan2(xy[1], xy[0])
                rho = np.linalg.norm(xy)
                return theta, rho

            def polar_to_cartesian(theta, rho):
                x = rho * np.cos(theta)
                y = rho * np.sin(theta)
                return x,y

            # ==========================================================================================================
            # Pure Pursuit Control Law
            # ==========================================================================================================
            # Variable names:
            # ---------------
            # alpha  ...  angle between the vehicle's heading vector and the look-ahead vector
            # l_d    ...  look-ahead distance (=length of the look-ahead vector) from the current rear axle
            #             position to the desired goal point location on the path
            # delta  ...  steering angle of the front wheel
            # L      ...  distance between the front axle and rear axle (wheelbase)
            # R      ...  radius of the circle that the rear axle will travel along at the given steering angle

            theta, l_d = cartesian_to_polar(goal[0:2] - self.car_position[0:2])
            alpha = (theta - self.car_heading)
            if alpha > np.pi:
                alpha -= 2*np.pi # ensure that alpha is in the range [-pi,pi]

            L = self.wheelbase
            delta = np.arctan((2.0 * L * np.sin(alpha)) / l_d)

            # (g_x, g_y) ... goal point in the car's coordinate frame
            g_x, g_y = polar_to_cartesian(alpha, l_d)

            # Check for a perfectly straight line
            if np.abs(g_y) <= 1e-4:
                # Goal is on a perfectly straight line [0,0] --> [g_x, 0]
                self.publish_commanded_heading(0.0)
                return

            assert(np.abs((l_d**2) - (g_x**2 + g_y**2)) <= np.finfo(np.float16).eps) # rho^2 == g_x^2 + g_y^2

            R = (l_d**2) / np.abs(2*g_y)
            R_test = np.abs(l_d / (2.0 * np.sin(alpha)))

            assert(np.abs(R-R_test) <= np.finfo(np.float16).eps)

            delta = np.arctan((2.0 * L * np.sin(alpha)) / l_d)

            def sss_angle(a,b,c):
                alpha = np.arccos((b**2 + c**2 - a**2) / (2 * b * c))
                return alpha

            central_angle = sss_angle(l_d,R,R)
            central_angle_threshold = np.deg2rad(1) # don't draw the arc if it is almost a straight line
            circle_center = np.array((0.0, np.sign(g_y) * R), dtype=np.float)

            if not np.isnan(central_angle):
                # Publish marker message
                marker_msg = Marker()
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.header.frame_id = "map"
                marker_msg.type = Marker.LINE_LIST
                marker_msg.color = ColorRGBA(0, 1, 1, 1)  # NOTE: color must be set here, not in rviz
                marker_msg.action = Marker.ADD
                marker_msg.scale.x = 0.033
                #marker_msg.scale.y = 0 # <-- ignored (any value!=0 will generate warnings in RViz)
                #marker_msg.scale.z = 0 # <-- ignored (any value!=0 will generate warnings in RViz)
                #marker_msg.pose.orientation.w = 1.0
                marker_msg.pose.position = transform.transform.translation
                marker_msg.pose.orientation = transform.transform.rotation

                circle_theta, circle_rho = cartesian_to_polar(circle_center)
                circle_arc_steps = int(np.abs(np.rad2deg(central_angle)) / 3) + 3

                if central_angle < central_angle_threshold:
                    marker_msg.points.append(Vector3(0, 0, 0))
                    marker_msg.points.append(Vector3(g_x, g_y, 0))
                else:
                    for k in range(circle_arc_steps-1):
                        offset0 = -np.pi + float(k+0) / float(circle_arc_steps-1) * (central_angle) * np.sign(g_y)
                        offset1 = -np.pi + float(k+1) / float(circle_arc_steps-1) * (central_angle) * np.sign(g_y)
                        cx0, cy0 = polar_to_cartesian(circle_theta + offset0, circle_rho)
                        cx1, cy1 = polar_to_cartesian(circle_theta + offset1, circle_rho)
                        marker_msg.points.append(Vector3(circle_center[0] + cx0, circle_center[1] + cy0,0))
                        marker_msg.points.append(Vector3(circle_center[0] + cx1, circle_center[1] + cy1,0))

                self.track_arc_pub.publish(marker_msg)

            # Publish commanded_heading (for the drive controller & RViz)
            self.publish_commanded_heading(delta)

            # Publish commanded_heading (for the drive controller)
            float_msg = Float32()
            float_msg.data = delta
            self.commanded_heading_angle_pub.publish(float_msg)

def main(args):
    rospy.init_node("pure_pursuit", anonymous=False)
    try:
        pure_pursuit = PurePursuit()
        rospy.sleep(0.1)
        rospy.spin()
    except:
        # TODO: error handling
        raise

if __name__=='__main__':
    main(sys.argv)
