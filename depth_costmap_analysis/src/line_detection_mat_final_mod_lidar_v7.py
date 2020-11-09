#!/usr/bin/env python
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math
import itertools
import tf2_ros
import tf
from collections import deque

class LinePoints():
    def __init__(self):
        self.line1_x1=0
        self.line1_x2=0
        self.line1_y1=0
        self.line1_y2=0
        self.line2_x1=0
        self.line2_x2=0
        self.line2_y1=0
        self.line2_y2=0

linepoints = LinePoints()

global lane_width
lane_width = 100

class Function():
    def __init__(self, a, b):
        self.a = a
        self.b = b

    # Initialize function from two points
    @classmethod
    def from_points(cls, l2, l1):
        a = float((l2[1] - l1[1]))/float(l2[0] - l1[0])
        b = (l2[1] - (a * l2[0]))
        return cls(a, b)

    def draw(self, image, x_max, color):
        pt1 = (0, int(self.b))
        pt2 = (x_max, int(self.a * x_max + self.b))
        cv2.line(image, pt1, pt2, color)

class ImageReader():
    def __init__(self):
        #self._local_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.grid_cb)
        self._local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.grid_cb)
        self.bridge = CvBridge()
        self.grid_flipped = np.array([])
        self.grid_trimmed = np.array([])
        self.origin_x = 0
        self.origin_y = 0
        self.grid_res = 0

    def grid_cb(self, msg):
        grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
        self.grid_res = msg.info.resolution
        # change 0s to black
        grid_masked = np.ma.array(grid, mask=grid==0, fill_value=255)
        # flip the axes to match ROS representation
        self.grid_flipped = np.flip(grid_masked, 0)
        self.origin_x = -msg.info.origin.position.x*(1/self.grid_res)
        self.origin_y = self.grid_flipped.shape[0] + msg.info.origin.position.y*(1/self.grid_res)

class LaneKeeping():
    def __init__(self):
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.imagereader = ImageReader()
        self.r = rospy.Rate(100)
        self.running_mean_depth = 200
        self.lines_above_robot_a =  deque(maxlen=self.running_mean_depth)
        self.lines_above_robot_b = deque(maxlen=self.running_mean_depth)
        self.lines_below_robot_a = deque(maxlen=self.running_mean_depth)
        self.lines_below_robot_b = deque(maxlen=self.running_mean_depth)
        self.max_slope = 0.02
        self.distance_to_line_above = 0
        self.distance_to_line_below = 0

    def robot_position(self):
        exists, position, orientation, timestamp = self.get_transform("map", "base_footprint")

        if exists:
            angle = - self.euler_from_quaternion(orientation)[2] * (180 / np.pi)
            robot_x = position.x * (1 / self.imagereader.grid_res)
            robot_y = position.y * (1 / self.imagereader.grid_res)

            robot_x_centre = int(self.imagereader.origin_x + robot_x)
            robot_y_centre = int(self.imagereader.origin_y + robot_y)
            prev_width = self.imagereader.grid_flipped.shape[0]

            self.imagereader.grid_trimmed = self.imagereader.grid_flipped[(robot_y_centre - (lane_width / 2)):(robot_y_centre + (lane_width / 2)), :]
            new_origin_y = int(self.imagereader.origin_y-((prev_width-lane_width)/2))
            robot_y_centre = int(new_origin_y + robot_y)

            img_cv = cv2.resize(self.imagereader.grid_trimmed, (self.imagereader.grid_trimmed.shape[1], self.imagereader.grid_trimmed.shape[0]))  # .shape[1] defines the height, while .shape[0] defines the image width
            #img_cv = cv2.resize(grid_trimmed,(grid_trimmed.shape[1], grid_trimmed.shape[0]))
            # gray = cv2.cvtColor(grid_flipped,cv2.COLOR_BGR2GRAY)
            smooth = cv2.GaussianBlur(img_cv,(5,5),0)
            edges = cv2.Canny(smooth,0,150,apertureSize = 3)
            lines = cv2.HoughLinesP(image=edges,
                                    rho=1,
                                    theta=np.pi/180,
                                    threshold=5,
                                    minLineLength=15,
                                    maxLineGap=2)
            color_img = cv2.cvtColor(self.imagereader.grid_trimmed, cv2.COLOR_GRAY2RGB)
            function_list = []

            if lines is not None:
                for line in lines:
                    x1,y1,x2,y2 = line[0]
                    try:
                        fn = Function.from_points((x2, y2), (x1, y1))
                    except ZeroDivisionError:
                        continue
                    function_list.append(fn)
                    #Print all the lines found
                    if(abs(fn.a) < self.max_slope):
                        if fn.b > new_origin_y:
                            #fn.draw(color_img, grid_trimmed.shape[1], (100,100,255))
                            self.lines_above_robot_a.append(fn.a)
                            self.lines_above_robot_b.append(fn.b)

                        else:
                            #fn.draw(color_img, grid_trimmed.shape[1], (100,255,100))
                            self.lines_below_robot_a.append(fn.a)
                            self.lines_below_robot_b.append(fn.b)


                    offset = ((max(x2, x1) - min(x2, x1))/2, (max(y2, y1) - min(y2,y1))/2)
                    mid_point = (min(x2, x1) + offset[0], min(y2, y1) + offset[1])
                    cv2.circle(color_img, (mid_point), 1, (255, 255, 255), 1)

            if len(self.lines_above_robot_a) > 0:
                self.line_above_mean = Function(float(sum(self.lines_above_robot_a))/float(len(self.lines_above_robot_a)),float(sum(self.lines_above_robot_b))/float(len(self.lines_above_robot_b)))
                self.line_above_mean.draw(color_img, self.imagereader.grid_trimmed.shape[1], (255,0,0))
                x1,y1,x2,y2 = self.line_to_points(self.line_above_mean.a,self.line_above_mean.b)
                a,b,c = self.get_2D_line_coefficients(x1,x2,y1,y2)
                self.distance_to_line_above =  self.get_2D_distance_to_line(a, b, c, robot_x_centre, robot_y_centre)
            if len(self.lines_below_robot_b) > 0:
                self.line_below_mean = Function(float(sum(self.lines_below_robot_a))/float(len(self.lines_below_robot_a)),float(sum(self.lines_below_robot_b))/float(len(self.lines_below_robot_b)))
                self.line_below_mean.draw(color_img, self.imagereader.grid_trimmed.shape[1], (0,255,0))
                x1,y1,x2,y2 = self.line_to_points(self.line_below_mean.a,self.line_below_mean.b)
                a,b,c = self.get_2D_line_coefficients(x1,x2,y1,y2)
                self.distance_to_line_below = self.get_2D_distance_to_line(a, b, c, robot_x_centre, robot_y_centre)

            self.draw_origin(0, 2, color_img, int(self.imagereader.origin_x), int(new_origin_y))
            self.draw_robot_orientation(angle, 20, color_img, robot_x_centre, robot_y_centre)
            #print robot_x_centre, robot_y_centre
            ros_img = self.imagereader.bridge.cv2_to_imgmsg(color_img, encoding="rgb8")
            self.image_pub.publish(ros_img)
            return  self.distance_to_line_above,  self.distance_to_line_below,  abs(self.distance_to_line_above) -  abs(self.distance_to_line_below)
        return 0,0,0
        self.r.sleep()

    def get_transform(self,from_link, to_link):
        try:
            trans = self.tfBuffer.lookup_transform(from_link, to_link, rospy.Time())
            return True, trans.transform.translation, trans.transform.rotation, trans.header.stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Cannot find the transform: " + str(e))
            return False, 0, 0, 0
            #print position, quaternion

    def euler_from_quaternion(self,quaternion):
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler

    def draw_robot_orientation(self, angle, pixel_size, image, width, height):
        #p1 = (width/2, height/2)
        p1 = (width, height)
        p2 = (int(p1[0] + pixel_size * np.cos(angle * np.pi / 180)), int(p1[1] + pixel_size * np.sin(angle * np.pi / 180)))
        cv2.line(image, p1, p2, (0,255,0))

    def draw_origin(self, angle, pixel_size, image, width, height):
        #p1 = (width/2, height/2)
        p1 = (width, height)
        p2 = (int(p1[0] + pixel_size * np.cos(angle * np.pi / 180)), int(p1[1] + pixel_size * np.sin(angle * np.pi / 180)))
        cv2.line(image, p1, p2, (255,0, 0))

    def get_points(self):
        return linepoints.line1_x1, linepoints.line1_x2, linepoints.line1_y1, linepoints.line1_y2, linepoints.line2_x1, linepoints.line2_x2, linepoints.line2_y1, linepoints.line2_y2

    def line_to_points(self,a,b):
        #y=ax+b
        #if x=0
        x1 = 0
        y1 = b
        #if x=1
        x2 = 1
        y2 = a + b
        return x1,y1,x2,y2

    def get_2D_line_coefficients(self,x1,x2,y1,y2):
        # y = ax + b
        #Ax + By + C = 0
        #m = -A / B
        a = -(y2-y1)
        b =  (x2-x1)
        #using point 2
        c =-(a*x2 + b*y2)
        coefficients = [a,b,c]
        return coefficients

    def get_2D_distance_to_line(self,a, b, c, x, y):
        distance = (a*x + b*y + c)/math.sqrt(a*a + b*b)
        return distance


if __name__ == "__main__":
    rospy.init_node("lane_keeping", anonymous=False)
    lanekeeping = LaneKeeping()
    while not rospy.is_shutdown():
        try:
            #print lanekeeping.robot_position()
            lanekeeping.robot_position()
        except rospy.ROSInterruptException:
            pass
