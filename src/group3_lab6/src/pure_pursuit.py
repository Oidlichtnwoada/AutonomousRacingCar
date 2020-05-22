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
roslib.load_manifest('group3_lab6')

import tf2_ros
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, PointStamped

from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

# import dynamic_reconfigure variables.
from group3_lab6.cfg import pure_pursuit_paramsConfig as Configuration
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

        self.dt_threshold = 1.0 / 50.0 # run at 50 Hz (max.)
        self.last_processing_timestamp = rospy.Time()

        DEFAULT_WHEELBASE = 0.3302 # see simulator.yaml
        # The distance between the front and rear axle of the racecar
        self.wheelbase = rospy.get_param('wheelbase', DEFAULT_WHEELBASE)

        default_tracked_path = os.path.realpath("{}/../waypoint_logs/waypoints_lowpass_filtered.csv".format( \
            os.path.dirname(sys.argv[0])))
        self.tracked_path = np.genfromtxt(rospy.get_param('tracked_path', default_tracked_path), delimiter=',')
        #self.tracked_path = np.genfromtxt(rospy.get_param('tracked_path'), delimiter=',')

        if rospy.has_param('lookahead_distance'):
            raise Exception("Required parameter \"{}\" not found.".format('lookahead_distance'))

        waypoints = np.asarray(self.tracked_path)
        self.waypoint_tree = cKDTree(data=waypoints[:,(0,1)])

        self.waypoint_orientations = np.asanyarray(Rotation.from_euler('z', waypoints[:,2], degrees=False).as_quat())
        self.waypoint_positions = waypoints[:,(0,1)]

        # Create a latched(*) path publisher
        # (*) http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.tracked_path_pub = rospy.Publisher('/pure_pursuit/tracked_path', Path, queue_size=1, latch=True)

        self.tracked_path_msg = self.generate_tracked_path_msg(self.tracked_path, self.forward_direction)

        self.tracked_path_pub.publish(self.tracked_path_msg)
        self.tracked_path_publication_frequency = 1.0
        self.t_last_tracked_path_publication = rospy.Time.now()

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.linear_velocity = 0
        self.angular_velocity = 0

        self.car_pose_is_known = False
        self.car_position = np.array((0,0), dtype=np.float)
        self.car_orientation = Rotation.from_quat((0, 0, 0, 1)).as_quat()
        self.car_heading = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.closest_waypoint_pub = rospy.Publisher('/pure_pursuit/closest_waypoint', PoseStamped, queue_size=1000) # for RViz
        self.goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1000) # for RViz
        self.track_arc_pub = rospy.Publisher('/pure_pursuit/track_arc', Marker, queue_size=1000) # for RViz
        self.commanded_heading_pub = rospy.Publisher('/pure_pursuit/commanded_heading', PoseStamped, queue_size=1000) # for RViz
        self.commanded_heading_angle_pub = rospy.Publisher('/pure_pursuit/commanded_heading_angle', Float32, queue_size=1000) # for the drive controller


    def generate_tracked_path_msg(self, tracked_path, forward_direction=True):
        tracked_path_msg = Path() # this won't change over the lifetime of the node
        t_now = rospy.Time.now()
        tracked_path_msg.header.stamp = t_now
        tracked_path_msg.header.frame_id = "map"

        for i in range(len(tracked_path)+1):
            p = PoseStamped()
            p.header.stamp = t_now
            p.header.frame_id = "map" # redundant
            j = np.mod(i,len(tracked_path))
            p.pose.position.x = tracked_path[j, 0]
            p.pose.position.y = tracked_path[j, 1]
            yaw = tracked_path[j, 2] if self.forward_direction else tracked_path[j, 2] + np.pi
            rotation = Rotation.from_euler('z', yaw, degrees=False)
            q = rotation.as_quat() # scalar-last (x, y, z, w) format
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            tracked_path_msg.poses.append(p)
        return tracked_path_msg

    def reconfigure(self, config, level):
        rospy.loginfo("""Reconfigure request: {lookahead_distance} {forward_direction}""".format(**config))
        self.lookahead_distance = config["lookahead_distance"]
        forward_direction = config["forward_direction"]

        if self.forward_direction != forward_direction:
            self.forward_direction = forward_direction
            self.tracked_path_msg = self.generate_tracked_path_msg(self.tracked_path, self.forward_direction)
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

        if dt <= self.dt_threshold:
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
            closest_waypoint_index, _, _ = self.compute_closest_waypoint()
            goal = self.compute_goal(closest_waypoint_index)

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

            # Publish commanded_heading (for the drive controller)
            float_msg = Float32()
            float_msg.data = delta
            self.commanded_heading_angle_pub.publish(float_msg)

            # Publish commanded_heading (for RViz)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0
            rotation = Rotation.from_euler('z', delta, degrees=False).as_quat()
            pose_msg.pose.orientation.x = rotation[0]
            pose_msg.pose.orientation.y = rotation[1]
            pose_msg.pose.orientation.z = rotation[2]
            pose_msg.pose.orientation.w = rotation[3]
            self.commanded_heading_pub.publish(pose_msg)


    def message_loop(self):
        while not rospy.is_shutdown():
            t_current = rospy.Time.now()
            if (t_current.to_sec() - self.t_last_tracked_path_publication.to_sec()) > \
                    (1.0 / self.tracked_path_publication_frequency):
                self.tracked_path_pub.publish(self.tracked_path_msg)
                self.t_last_tracked_path_publication = t_current
            rospy.sleep(0.1)

def main(args):
    rospy.init_node("pure_pursuit", anonymous=False)
    pure_pursuit = PurePursuit()
    try:
        pure_pursuit.message_loop()
    except:
        # TODO: error handling
        raise

if __name__=='__main__':
    main(sys.argv)
