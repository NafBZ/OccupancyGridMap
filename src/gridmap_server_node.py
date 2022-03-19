#!/usr/bin/python3

import numpy as np
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from utils_lib.gridmap import *


class GridmapServer:
    def __init__(self, scan_topic, world_frame_id, cell_size):

        self.world_frame_id = world_frame_id
        self.occ_grid_pub = rospy.Publisher(
            "occupancy_grid", OccupancyGrid, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            scan_topic, LaserScan, self.get_scan, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.gridmap = None
        self.cell_size = cell_size
        self.pub_timer = rospy.Timer(rospy.Duration(0.2), self.publish_gridmap)
        print('Occupancy Grid Map server running')

    def get_scan(self, scan):
        try:
            # Get transform from laser_frame_id to world_frame_id
            trans = self.tfBuffer.lookup_transform(
                self.world_frame_id, scan.header.frame_id, rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion([trans.transform.rotation.x,
                                                                  trans.transform.rotation.y,
                                                                  trans.transform.rotation.z,
                                                                  trans.transform.rotation.w])
            # Initialize gridmap object with the first measure
            if self.gridmap is None:
                self.gridmap = GridMap(center=(
                    trans.transform.translation.x, trans.transform.translation.y), cell_size=self.cell_size)

            # For each laser beam in the scan message, call gridmap.add_ray function
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            for i, r in enumerate(scan.ranges):
                # If the laser range is in the range of the sensor apply the update
                if r >= scan.range_min and r <= scan.range_max:
                    # Define the diferent angles of the laser beam according to the specs of the scanner
                    yaw_ray = yaw + scan.angle_min + i*scan.angle_increment
                    self.gridmap.add_ray((x, y), yaw_ray, r, 0.7)

            # self.publish_gridmap()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Invalid TF from {} to {}".format(
                scan.header.frame_id, self.world_frame_id))

    def publish_gridmap(self, event=None):

        # Reshaping and scaling the grid map to be able to publish as nav_msgs/OccupancyGrid
        gridmap_img = (self.gridmap.get_map().reshape((1, -1))[0] + 6.91)*7.23

        # Cells that have not been yet explored are set to -1
        for index, i in enumerate(gridmap_img):
            if i == 6.91*7.23:
                gridmap_img[index] = -1

        # Definning the occupancy grid map message
        occ_grid = OccupancyGrid()
        occ_grid.header.frame_id = self.world_frame_id
        occ_grid.header.stamp = rospy.Time()
        occ_grid.info.resolution = self.cell_size
        occ_grid.info.width = int(self.gridmap.map_width[0]/self.cell_size)
        occ_grid.info.height = int(self.gridmap.map_width[1]/self.cell_size)
        occ_grid.info.origin.position.x = self.gridmap.origin[0]
        occ_grid.info.origin.position.y = self.gridmap.origin[1]
        occ_grid.info.origin.position.z = 0
        occ_grid.info.origin.orientation.x = 0
        occ_grid.info.origin.orientation.y = 0
        occ_grid.info.origin.orientation.z = 0
        occ_grid.info.origin.orientation.w = 1
        occ_grid.data = gridmap_img.astype(dtype=np.int8)

        # Publishing the message
        self.occ_grid_pub.publish(occ_grid)


if __name__ == '__main__':
    rospy.init_node('gridmap_test')
    node = GridmapServer('/scan', 'odom', 0.1)
    rospy.spin()
