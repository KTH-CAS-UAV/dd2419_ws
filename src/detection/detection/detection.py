#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        


    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255

        r_thresh = 190 / 255
        g_thresh_red = 70 / 255
        b_thresh_red = 80 / 255

        g_thresh_green = 100 / 255
        r_thresh_green = 10 / 255
        b_thresh_green = 110 / 255

        # add distance filtering to avoid noise!
        distance_mask = points[:,2] < 1
        
        red_mask = (colors[:, 0] > r_thresh) & (colors[:, 1] < g_thresh_red) & (colors[:, 2] < b_thresh_red) & distance_mask
        green_mask = (colors[:, 1] > g_thresh_green) & (colors[:, 0] < r_thresh_green) & (colors[:, 2] > b_thresh_green)

        # hsv color scale  TODO look into that or ok lab, that is daniels favourite
        # green_mask = (colors[:,1] > green_scaling*(colors[:,0]+colors[:,1])/2)

        red_counter = np.sum(red_mask)
        green_counter = np.sum(green_mask)

        if red_counter > 10: 
            self.get_logger().info(f'red sphere detected')
            red_points = points[red_mask]
            msg_red = pc2.create_cloud_xyz32(msg.header,red_points.astype(float))
            self._pub.publish(msg_red)

        if green_counter > 10: 
            self.get_logger().info(f'green cube detected')
            green_points = points[green_mask]
            msg_green = pc2.create_cloud_xyz32(msg.header,green_points.astype(float))
            self._pub.publish(msg_green)



def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()