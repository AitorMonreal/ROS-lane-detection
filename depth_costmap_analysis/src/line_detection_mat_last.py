#!/usr/bin/env python

import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

class LaneKeeping():
    def __init__(self):
        self._quaternion_sub = rospy.Subscriber ('/odometry/filtered', Odometry, self.get_rotation)
        self._local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.grid_cb)
        self._bridge = CvBridge()
        self._image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self._lines_pub = rospy.Publisher("~line_image", Image, queue_size=1)

    def get_rotation(self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print yaw

    def grid_cb(self, msg):
        grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)

        # change 0s to black
        grid_masked = np.ma.array(grid, mask=grid==0, fill_value=255)
        # flip the axes to match ROS representation
        grid_flipped = np.flip(grid_masked, 0)

        plt.imsave('/home/aitor/ros/deutsche_ws/grid.png', grid)
        plt.imsave('/home/aitor/ros/deutsche_ws/grid_masked.png', grid_masked)
        plt.imsave('/home/aitor/ros/deutsche_ws/grid_flipped.png', grid_flipped)
        ros_img = self._bridge.cv2_to_imgmsg(grid_flipped, encoding="passthrough")
        im2 = self._bridge.imgmsg_to_cv2(ros_img)
        plt.imsave('/home/aitor/ros/deutsche_ws/ros_img.png', im2)
        #self._image_pub.publish(ros_img)

        img = cv2.imread('/home/aitor/ros/deutsche_ws/grid_flipped.png')

        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(grid_flipped,low_threshold,high_threshold,apertureSize = 5)

        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)

        lines = cv2.HoughLines(edges,rho,theta,threshold)

        chosen_lines = np.empty((0,2), int)
        for line in lines:
            for rho,theta in line:
                if theta > (-(yaw/2)+(88*(np.pi/180))) and theta < (-(yaw/2)+(92*(np.pi/180))):
                    chosen_lines = np.append(chosen_lines, np.array([[rho, theta]]), axis=0)

        rho_mean = np.mean(chosen_lines, axis=0)[0]
        rho1_arr = np.array([])
        theta1_arr = np.array([])
        rho2_arr = np.array([])
        theta2_arr = np.array([])
        for row in chosen_lines:
            if row[0] > rho_mean:
                rho1_arr = np.append(rho1_arr, row[0])
                theta1_arr = np.append(theta1_arr, row[1])
            else:
                rho2_arr = np.append(rho2_arr, row[0])
                theta2_arr = np.append(theta2_arr, row[1])
            
        rho1 = np.mean(rho1_arr)
        theta1 = np.mean(theta1_arr)
        rho2 = np.mean(rho2_arr)
        theta2 = np.mean(theta2_arr)

        a = np.cos(theta1)
        b = np.sin(theta1)
        x0 = a*rho1
        y0 = b*rho1
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(img, (x1,y1), (x2,y2),(255,0,0),1)

        a = np.cos(theta2)
        b = np.sin(theta2)
        x0 = a*rho2
        y0 = b*rho2
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(img, (x1,y1), (x2,y2),(255,0,0),1)

        ros_lines = self._bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self._lines_pub.publish(ros_lines)

if __name__ == "__main__":
    try:
        rospy.init_node("lane_keeping", anonymous=False)
        lk = LaneKeeping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass