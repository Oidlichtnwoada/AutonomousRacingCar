#!/usr/bin/env python

# -------------------------------------------------------
# VU Autonomous Racing Cars (2020S) - TU Wien
# -------------------------------------------------------
# Author: Thomas Pintaric (thomas@pintaric.org)
# SPDX-License-Identifier: 0BSD
# -------------------------------------------------------

from __future__ import print_function
import sys
import itertools
import numpy as np
from scipy.ndimage import filters

import rospy
from std_msgs.msg import Float32, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

class FollowTheGap:

    def __init__(self):

        self.max_speed = 7.0   # meters/second
        self.max_decel = 8.26  # meters/second^2
        self.max_stopping_distance = np.square(self.max_speed) / (2.0 * self.max_decel)
        self.lookahead_distance = 2.0 * self.max_stopping_distance
        self.use_lookahead_distance_only_for_visualization = False
        self.vehicle_width = (0.3302 * 1.2) # use 120% of the wheelbase as vehicle width

        self.last_lidar_timestamp = rospy.Time()
        self.dt_threshold = 1.0 / 50.0 # run computation at max. 50 Hz

        heading_topic = '/reactive_driving/heading' # vehicle heading setpoint (units: rad)
        self.heading_pub = rospy.Publisher(heading_topic, Float32, queue_size=1000)

        free_space_topic = '/reactive_driving/free_space'
        #self.free_space_pub = rospy.Publisher(free_space_topic, PolygonStamped, queue_size=1000)
        self.free_space_pub = rospy.Publisher(free_space_topic, Marker, queue_size=1000)

        self.forward_scan_arc = (np.deg2rad(-90.0), np.deg2rad(+90.0))
        self.heading_computation_arc = np.deg2rad(30.0)
        self.heading_computation_percentile = 100 * (1.0 - (self.heading_computation_arc / \
                                                            (self.forward_scan_arc[1] - self.forward_scan_arc[0])))

        self.minimum_gap_lenth = 0.2 # units: m
        self.median_range_deviation_threshold = 9.0 # outlier test: x[i] > median(x) * median_deviation_threshold

        lida_topic = '/scan'
        self.lidar_sub = rospy.Subscriber(lida_topic, LaserScan, self.lidar_callback)

    def get_lidar_scan_arc(self, data, angle_start, angle_end):
        if angle_start < data.angle_min or angle_end > data.angle_max or angle_start >= angle_end:
            raise ValueError('requested angles are out-of-range')
        subrange = np.divide(np.subtract((angle_start, angle_end), data.angle_min), \
                            data.angle_increment).astype(np.int)
        angles = np.add(np.multiply(np.arange(subrange[0],subrange[1]+1).astype(np.float), \
                                    data.angle_increment), data.angle_min)
        ranges = np.array(data.ranges[subrange[0]:subrange[1]+1])
        return angles, ranges, subrange

    def lidar_callback(self, data):

        has_previous_timestamp = not self.last_lidar_timestamp.is_zero()
        current_timestamp = rospy.Time(secs=data.header.stamp.secs, nsecs=data.header.stamp.nsecs)
        
        if not has_previous_timestamp:
            self.last_lidar_timestamp = current_timestamp
            return

        dt = current_timestamp.to_sec() - self.last_lidar_timestamp.to_sec()

        if dt <= self.dt_threshold:
            return

        self.last_lidar_timestamp = current_timestamp

        angles, ranges, _ = self.get_lidar_scan_arc(data, \
                                                    self.forward_scan_arc[0], \
                                                    self.forward_scan_arc[1])

        if not self.use_lookahead_distance_only_for_visualization:
            ranges = np.clip(ranges, a_min=0.0, a_max=self.lookahead_distance)

        diff_ranges = np.abs(np.ediff1d(ranges))

        filter_scan_arc = np.deg2rad(10.0) # units: radians
        filter_width = int(filter_scan_arc / data.angle_increment) # specify filter width in terms of the circular scan arc
        median_filtered_diff_ranges = filters.median_filter(input=diff_ranges.flatten(), size=(filter_width,), mode='nearest')
        max_filtered_diff_ranges = filters.maximum_filter1d(input=diff_ranges.flatten(), size=filter_width)

        mask = np.logical_and(np.equal(diff_ranges, max_filtered_diff_ranges), \
                              np.greater(diff_ranges, np.multiply(median_filtered_diff_ranges, \
                                                                  self.median_range_deviation_threshold)))
        mask = np.logical_and(mask, np.greater(diff_ranges, self.minimum_gap_lenth))

        masked_angles = np.ma.masked_where(np.logical_not(mask), angles[:-1])

        def enumerate_masked_array(masked_array):
            mask = ~masked_array.mask.ravel()
            for i, m in itertools.izip(np.ndenumerate(masked_array), mask):
                if m: yield i

        adjusted_ranges = np.copy(ranges)

        for i, theta in enumerate_masked_array(masked_angles):
            short_side = self.vehicle_width
            long_side = np.amin(ranges[i[0]-1:i[0]+2])
            beta = np.arccos((2.0 * np.square(long_side) - np.square(short_side)) / (2.0 * np.square(long_side)))
            gamma = np.array((theta - beta, theta + beta))
            index_range = np.divide(np.subtract((gamma[0], gamma[1]), angles[0]), \
                                    data.angle_increment).astype(np.int)

            index_range = np.clip(index_range, 0, len(ranges)-1)
            for x in np.nditer(adjusted_ranges[index_range[0]:index_range[1] + 1], op_flags=['readwrite']):
                x[...] = min(x, long_side)

        if self.use_lookahead_distance_only_for_visualization:
            adjusted_ranges = np.clip(adjusted_ranges, a_min=0.0, a_max=self.lookahead_distance)

        # Publish the vehicle's target heading

        heading_msg = Float32()
        indices = np.digitize(adjusted_ranges, [0.0, np.percentile(adjusted_ranges, q=self.heading_computation_percentile), data.range_max])
        heading_msg.data = np.mean(angles[indices==2])
        self.heading_pub.publish(heading_msg)

        # Publish a polygon (triangle list) of the driveable area for rviz

        free_space_msg = Marker()
        free_space_msg.header.stamp = rospy.Time.now()
        free_space_msg.header.frame_id = data.header.frame_id # = "laser"
        free_space_msg.type = Marker.TRIANGLE_LIST
        free_space_msg.color = ColorRGBA(1,0,0,0.25) # NOTE: color must be set here, not in rviz
        free_space_msg.action = Marker.ADD

        free_space_msg.scale.x = 1
        free_space_msg.scale.y = 1
        free_space_msg.scale.z = 1
        free_space_msg.pose.orientation.w = 1.0

        px = np.multiply(adjusted_ranges, np.cos(angles))
        py = np.multiply(adjusted_ranges, np.sin(angles))

        for x0,y0,x1,y1 in zip(px[:-1],py[:-1],px[1:],py[1:]):
            free_space_msg.points.append(Vector3(0,0,0))
            free_space_msg.points.append(Vector3(x0,y0,0))
            free_space_msg.points.append(Vector3(x1,y1,0))

        self.free_space_pub.publish(free_space_msg)

def main(args):
    rospy.init_node("follow_the_gap", anonymous=False)
    follow_the_gap = FollowTheGap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
