#!/usr/bin/env python

import math

import numpy as np
import colour as co
import rclpy
import time
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField

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
        
        self.thresh = self.get_thresholds()
        


    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """
        # Convert ROS -> NumPy
        # start_time = time.time()

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        rgb_uint32 = gen[:, 3].view(np.uint32)
        colors = np.empty((len(rgb_uint32), 3), dtype=np.uint8)
        colors[:, 0] = (rgb_uint32 >> 16) & 255
        colors[:, 1] = (rgb_uint32 >> 8) & 255
        colors[:, 2] = rgb_uint32 & 255

        # geometrical filter
        max_dist = 2
        max_height = 0.05   
        min_height = 0.08
        geom_mask = ((points[:,2] < max_dist) & (points[:,1] > max_height) & (points[:,1] < min_height))
        # the cleanest solution is to filter the points in the odom/map frame this should be implemented in the future
        # also it should be checked if the 
        # TODO filter out the floor as well!

        points_f = points[geom_mask]
        colors_f = colors[geom_mask]

        # conversion of color spaces from rgb to oklab
        colors_rgb = colors_f.astype(np.float32) / 255
        colors_xyz = co.sRGB_to_XYZ(colors_rgb)
        colors_oklab = co.XYZ_to_Oklab(colors_xyz)

        # self.get_logger().info(f'comp_colors_oklab.shape: {colors_oklab.shape}')
        # self.get_logger().info(f'geom_mask ones: {np.sum(geom_mask)}')
        
        # assembling of color masks
        red_mask = (
            (self.thresh[0, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < self.thresh[0, 1]) & 
            (self.thresh[0, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < self.thresh[0, 3]) 
        )
        green_mask = (
            (self.thresh[1, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < self.thresh[1, 1]) & 
            (self.thresh[1, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < self.thresh[1, 3]) 
        )
        blue_mask = (
            (self.thresh[2, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < self.thresh[2, 1]) & 
            (self.thresh[2, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < self.thresh[2, 3]) 
        )
        wood_mask = (
            (self.thresh[3, 0] < colors_oklab[:, 1]) & (colors_oklab[:, 1] < self.thresh[3, 1]) & 
            (self.thresh[3, 2] < colors_oklab[:, 2]) & (colors_oklab[:, 2] < self.thresh[3, 3]) 
        )

        # Chek how many red,green,... points we have
        red_counter = np.sum(red_mask)
        green_counter = np.sum(green_mask)
        blue_counter = np.sum(blue_mask)
        wood_counter = np.sum(wood_mask)

        # apply mask, create pointcloud and publish message if counter>min_num_points
        min_num_points = 4
        
        fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='color_idx', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
        if red_counter > min_num_points: 
            # self.get_logger().info(f'red sphere detected \n red_counter = {red_counter}')
            red_points = points_f[red_mask]
            red_color_idx = np.full((red_points.shape[0], 1), 1.0, dtype=np.float32)  # add color index as 4th coloumn (1 for red, 2 for green, 3 for blue, 4 for wood)
            red_points_with_idx = np.column_stack((red_points, red_color_idx))
            # msg_red = pc2.create_cloud_xyz32(msg.header,red_points_with_idx.astype(float))
            msg_red = pc2.create_cloud(msg.header,fields,red_points_with_idx)
            self._pub.publish(msg_red)

        if green_counter > min_num_points: 
            # self.get_logger().info(f'green cube detected')
            green_points = points_f[green_mask]
            green_color_idx = np.full((green_points.shape[0], 1), 2.0, dtype=np.float32)  # add color index as 4th coloumn (1 for red, 2 for green, 3 for blue, 4 for wood)
            green_points_with_idx = np.column_stack((green_points, green_color_idx))
            msg_green = pc2.create_cloud(msg.header, fields, green_points_with_idx)
            self._pub.publish(msg_green)

        if blue_counter > min_num_points: 
            # self.get_logger().info(f'blue sphere detected')
            blue_points = points_f[blue_mask]
            blue_color_idx = np.full((blue_points.shape[0], 1), 3.0, dtype=np.float32)  # add color index as 4th coloumn (1 for red, 2 for green, 3 for blue, 4 for wood)
            blue_points_with_idx = np.column_stack((blue_points, blue_color_idx))
            msg_blue = pc2.create_cloud(msg.header, fields, blue_points_with_idx)
            self._pub.publish(msg_blue)

        if wood_counter > min_num_points: 
            # self.get_logger().info(f'wood cube detected')
            wood_points = points_f[wood_mask]
            wood_color_idx = np.full((wood_points.shape[0], 1), 4.0, dtype=np.float32)  # add color index as 4th coloumn (1 for red, 2 for green, 3 for blue, 4 for wood)
            wood_points_with_idx = np.column_stack((wood_points, wood_color_idx))
            msg_wood = pc2.create_cloud(msg.header, fields, wood_points_with_idx)
            self._pub.publish(msg_wood)

        # dt = time.time() - start_time
        # self.get_logger().info(f"Callback took: {dt*1000:.2f} ms")
        
    

    def get_thresholds(self):

        comp_colors_rgb = np.array([
            [140, 45, 35], #red
            [0, 70, 57], #green
            [0, 83, 125], # blue
            [100, 75, 52] #wood
            ])
                
        comp_colors_rgb = comp_colors_rgb / 255.0
        comp_colors_xyz = co.sRGB_to_XYZ(comp_colors_rgb)
        comp_colors_oklab = co.XYZ_to_Oklab(comp_colors_xyz)
        self.get_logger().info(f'comp_colors_oklab\n red: {comp_colors_oklab[0,:]} \n green: {comp_colors_oklab[1,:]}\n blue {comp_colors_oklab[2,:]}\n wood{comp_colors_oklab[3,:]}')
        
        # define tolerances
        tol_red = 0.02
        tol_green = 0.01
        tol_blue = 0.015
        tol_wood = 0.01

        thresh_red_a_low = comp_colors_oklab[0,1] - tol_red
        thresh_red_a_high = comp_colors_oklab[0,1] + tol_red
        thresh_red_b_low = comp_colors_oklab[0,2] - tol_red
        thresh_red_b_high = comp_colors_oklab[0,2] + tol_red

        thresh_green_a_low = comp_colors_oklab[1,1] - tol_green
        thresh_green_a_high = comp_colors_oklab[1,1] + tol_green
        thresh_green_b_low = comp_colors_oklab[1,2] - tol_green
        thresh_green_b_high = comp_colors_oklab[1,2] + tol_green

        thresh_blue_a_low = comp_colors_oklab[2,1] - tol_blue
        thresh_blue_a_high = comp_colors_oklab[2,1] + tol_blue
        thresh_blue_b_low = comp_colors_oklab[2,2] - tol_blue
        thresh_blue_b_high = comp_colors_oklab[2,2] + tol_blue

        thresh_wood_a_low = comp_colors_oklab[3,1] - tol_wood
        thresh_wood_a_high = comp_colors_oklab[3,1] + tol_wood
        thresh_wood_b_low = comp_colors_oklab[3,2] - tol_wood
        thresh_wood_b_high = comp_colors_oklab[3,2] + tol_wood

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