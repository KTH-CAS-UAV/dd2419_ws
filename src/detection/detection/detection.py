#!/usr/bin/env python
import numpy as np
import colour as co
import rclpy
import time
from rclpy.node import Node

from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import PointStamped

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)
        
        self.centroid_pub = self.create_publisher(PointStamped, '/detection/objects',10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/realsense/depth/color/points', self.cloud_callback, 10)
        
        self.thresh = self.get_thresholds()

        self.min_samples = 5 # min number of samples to be considered one object
        self.eps = 0.03 # ponints within this distance to each other are considered one object
        self.dbscan = DBSCAN(eps=self.eps, min_samples=self.min_samples)
        # Define your known object size (e.g., a 10cm cube)
        self.obj_width = 0.03  # meters
        self.obj_width = 0.03 # meters
        self.tolerance = 0.01    # +/- 3cm tolerance

        


    def cloud_callback(self, msg: PointCloud2):
        """
        Takes point cloud readings to detect objects.
        This function is called for every message that is published on the '/camera/depth/color/points' topic.
        """

        # TODO for the future, if it becomes a bottleneck: merge the messages into onemessage that is published
        # this is for sure cleaner since we currently have to handle multiple messages at the same time if we detect multiple things at the same time
        self.get_logger().info(f'msg.header.frame_id = {msg.header.frame_id}')
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
        min_num_points = self.min_samples
        
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
            red_centroid = self.process_clusters(red_points)
            print(red_centroid)
            msg_red = pc2.create_cloud(msg.header,fields,red_points_with_idx)

            if len(red_centroid)==1:
                msg_red_centroid = PointStamped()
                msg_red_centroid.header = msg.header
                msg_red_centroid.point.x = float(red_centroid[0][0])
                msg_red_centroid.point.y = float(red_centroid[0][1])
                msg_red_centroid.point.z = float(red_centroid[0][2])
                self.centroid_pub.publish(msg_red_centroid)
                self.get_logger().info(f'i just published this message: {msg_red_centroid}')
            elif len(red_centroid)==0:
                self.get_logger().info(f'clustering red returned an empty list')
            elif len(red_centroid)>1:
                self.get_logger().info(f'clustering red returned multiple centroids')

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
            
            wood_centroid = self.process_clusters(wood_points)

            if len(wood_centroid)==1:
                msg_wood_centroid = PointStamped()
                msg_wood_centroid.header = msg.header
                msg_wood_centroid.point.x = float(wood_centroid[0][0])
                msg_wood_centroid.point.y = float(wood_centroid[0][1])
                msg_wood_centroid.point.z = float(wood_centroid[0][2])
                self.centroid_pub.publish(msg_wood_centroid)
                self.get_logger().info(f'i just published this message: {msg_wood_centroid}')

            elif len(wood_centroid)==0:
                self.get_logger().info(f'clustering wood returned an empty list')
            elif len(wood_centroid)>1:
                self.get_logger().info(f'clustering wood returned multiple centroids')
            msg_wood = pc2.create_cloud(msg.header, fields, wood_points_with_idx)
            self._pub.publish(msg_wood)


        # dt = time.time() - start_time
        # self.get_logger().info(f"Callback took: {dt*1000:.2f} ms")
    

    def process_clusters(self, points_3d):
        """
        Input: points_3d (N, 3) numpy array of filtered XYZ coordinates
        Output: List of centroids [x, y, z] for valid objects
        """
        if len(points_3d) < 10:
            return []

        # 1. Run Clustering (Very fast on <2000 points)
        # Returns labels like [0, 0, 1, -1, 0, 1...] (-1 is noise)
        labels = self.dbscan.fit_predict(points_3d)
        
        valid_centroids = []
        
        # Get unique labels (skip -1 which is noise)
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)

        for label in unique_labels:
            # 2. Extract Points for this specific cluster
            # Boolean indexing is fast
            cluster_mask = (labels == label)
            cluster_points = points_3d[cluster_mask]
            
            # 3. FAST Geometric Check (Axis-Aligned Bounding Box)
            # We calculate the dimensions of the cluster
            min_p = np.min(cluster_points, axis=0)
            max_p = np.max(cluster_points, axis=0)
            dims = max_p - min_p # [width_x, width_y, height_z]
            
            # Check 1: Is the size roughly correct?
            # You can get more specific (e.g., check X vs Y vs Z) if rotation is known
            if not (self.obj_width - self.tolerance < np.max(dims) < self.obj_width + self.tolerance):
                continue # Skip this cluster, it's too big/small
                
            # Check 2: Density Check (Optional but recommended)
            # If it's the right size but has only 15 points, it might be a ghost reflection
            # A real solid object should have many points
            # if len(cluster_points) < 10: 
            #     continue

            # 4. Calculate Centroid
            centroid = np.mean(cluster_points, axis=0)
            valid_centroids.append(centroid)

        return valid_centroids

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