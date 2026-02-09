#!/usr/bin/env python

import math

import numpy as np
import colour as co
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
            PointCloud2, '/realsense/depth/color/points', self.cloud_callback, 10)
        


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

        colors_rgb = colors.astype(np.float32) / 255
        colors_xyz = co.sRGB_to_XYZ(colors_rgb)
        colors_oklab = co.XYZ_to_Oklab(colors_xyz)
        self.get_logger().info(f'comp_colors_oklab.shape: {colors_oklab.shape}')
        
        # add distance filtering to avoid noise!
        geom_mask = (points[:,2] < 1) & (points[:,1] > -0.05)# check if this has to be [:,0] or [:,1]
        self.get_logger().info(f'geom_mask ones: {np.sum(geom_mask)}')
        thresh = self.get_thresholds()
        red_mask = (
            (thresh[0, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < thresh[0, 1]) & 
            (thresh[0, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < thresh[0, 3]) & 
            geom_mask
        )
        green_mask = (
            (thresh[1, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < thresh[1, 1]) & 
            (thresh[1, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < thresh[1, 3]) & 
            geom_mask
        )
        blue_mask = (
            (thresh[2, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < thresh[2, 1]) & 
            (thresh[2, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < thresh[2, 3]) & 
            geom_mask
        )
        wood_mask = (
            (thresh[3, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < thresh[3, 1]) & 
            (thresh[3, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < thresh[3, 3]) & 
            geom_mask
        )

        # hsv color scale  TODO look into that or ok lab, that is daniels favourite
        red_counter = np.sum(red_mask)
        green_counter = np.sum(green_mask)
        blue_counter = np.sum(blue_mask)
        wood_counter = np.sum(wood_mask)
        self.get_logger().info(f'red_mask ones: {red_counter}')

        if red_counter > 10: 
            self.get_logger().info(f'red sphere detected \n red_counter = {red_counter}')
            red_points = points[red_mask]
            msg_red = pc2.create_cloud_xyz32(msg.header,red_points.astype(float))
            self._pub.publish(msg_red)

        # if green_counter > 10: 
        #     self.get_logger().info(f'green cube detected')
        #     green_points = points[green_mask]
        #     msg_green = pc2.create_cloud_xyz32(msg.header,green_points.astype(float))
        #     self._pub.publish(msg_green)

        # if blue_counter > 10: 
        #     self.get_logger().info(f'red sphere detected')
        #     red_points = points[red_mask]
        #     msg_red = pc2.create_cloud_xyz32(msg.header,red_points.astype(float))
        #     self._pub.publish(msg_red)

        # if wood_counter > 10: 
        #     self.get_logger().info(f'green cube detected')
        #     green_points = points[green_mask]
        #     msg_green = pc2.create_cloud_xyz32(msg.header,green_points.astype(float))
        #     self._pub.publish(msg_green)
    
    def get_thresholds(self):
        comp_colors_rgb = np.array([
            [140, 45, 35], #red
            [0, 70, 57], #green
            [0, 83, 125], # blue
            [100, 75, 52] #wood
            ])
        # comparison rgb values measured: 
        # red: 137 55 50  | 145 37 17
        # green: 0 72  58 | 1 67 56 
        # blue: 1 90 134 | 2 76 117
        # wood: 111 77 49 | 90 73 54

        # comp_colors_oklab red: [ 0.64356247  0.08838131  0.05120019] 
        #  green: [ 0.58387405 -0.10329892 -0.00694187]
        #  blue [ 0.63563073 -0.08929985 -0.07248441]
        #  wood[ 0.67719286  0.01391608  0.03818419]

        comp_colors_rgb = comp_colors_rgb / 255.0
        comp_colors_xyz = co.sRGB_to_XYZ(comp_colors_rgb)
        comp_colors_oklab = co.XYZ_to_Oklab(comp_colors_xyz)
        self.get_logger().info(f'comp_colors_oklab\n red: {comp_colors_oklab[0,:]} \n green: {comp_colors_oklab[1,:]}\n blue {comp_colors_oklab[2,:]}\n wood{comp_colors_oklab[3,:]}')
        
        tol = 0.1

        thresh_red_a_low = comp_colors_oklab[0,1] - tol
        thresh_red_a_high = comp_colors_oklab[0,1] + tol
        thresh_red_b_low = comp_colors_oklab[0,2] - tol
        thresh_red_b_high = comp_colors_oklab[0,2] + tol

        thresh_green_a_low = comp_colors_oklab[1,1] - tol
        thresh_green_a_high = comp_colors_oklab[1,1] + tol
        thresh_green_b_low = comp_colors_oklab[1,2] - tol
        thresh_green_b_high = comp_colors_oklab[1,2] + tol

        thresh_blue_a_low = comp_colors_oklab[2,1] - tol
        thresh_blue_a_high = comp_colors_oklab[2,1] + tol
        thresh_blue_b_low = comp_colors_oklab[2,2] - tol
        thresh_blue_b_high = comp_colors_oklab[2,2] + tol

        thresh_wood_a_low = comp_colors_oklab[3,1] - tol
        thresh_wood_a_high = comp_colors_oklab[3,1] + tol
        thresh_wood_b_low = comp_colors_oklab[3,2] - tol
        thresh_wood_b_high = comp_colors_oklab[3,2] + tol
        thresh = np.array([[thresh_red_a_low,thresh_red_a_high,thresh_red_b_low, thresh_red_b_high],
                         [thresh_green_a_low,thresh_green_a_high,thresh_green_b_low, thresh_green_b_high],
                         [thresh_blue_a_low,thresh_blue_a_high,thresh_blue_b_low, thresh_blue_b_high],
                         [thresh_wood_a_low,thresh_wood_a_high,thresh_wood_b_low, thresh_wood_b_high]])
        return thresh


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