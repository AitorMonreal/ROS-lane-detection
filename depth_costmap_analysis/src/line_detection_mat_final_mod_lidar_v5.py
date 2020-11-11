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

class LinePoints():
    def __init__(self):
        self.line_x1=0
        self.line_x2=0
        self.line_y1=0
        self.line_y2=0

linepoints = LinePoints()

global lane_width
lane_width = 80

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

    def robot_position(self):
        #print('Hello1')
        r = rospy.Rate(100)
        exists, position, orientation, timestamp = self.get_transform("map", "base_footprint")

        if exists:
            '''
            if self.imagereader.grid_res == 0:
                self.imagereader.grid_res = prev_res
            '''
            # self.imagereader.grid_res = 0.01

            angle = -self.euler_from_quaternion(orientation)[2]*(180/np.pi)
            robot_map_x = position.x*(1/self.imagereader.grid_res)
            robot_map_y = position.y*(1/self.imagereader.grid_res)

            robot_x = int(self.imagereader.origin_x+robot_map_x)
            robot_y = int(self.imagereader.origin_y+robot_map_y)
            prev_width = self.imagereader.grid_flipped.shape[0]

            vector_robot = np.array([robot_x, robot_y])

            grid_trimmed = self.imagereader.grid_flipped[robot_y-(lane_width/2):robot_y+(lane_width/2), :]

            '''
            width_removed = (prev_width-lane_width)/2
            origin_y = prev_width - width_removed + msg.info.origin.position.y*(1/grid_res)
            robot_y_centre = int(origin_y+robot_y)
            '''

            img_cv = cv2.resize(self.imagereader.grid_flipped, (self.imagereader.grid_flipped.shape[1], self.imagereader.grid_flipped.shape[0]))  # .shape[1] defines the height, while .shape[0] defines the image width
            #img_cv = cv2.resize(grid_trimmed,(grid_trimmed.shape[1], grid_trimmed.shape[0]))
            # gray = cv2.cvtColor(grid_flipped,cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(img_cv,0,150,apertureSize = 3)
            lines = cv2.HoughLinesP(image=edges,
                                    rho=1,
                                    theta=np.pi/180,
                                    threshold=5,
                                    minLineLength=15,
                                    maxLineGap=2)
            color_img = cv2.cvtColor(self.imagereader.grid_flipped, cv2.COLOR_GRAY2RGB)
            function_list = []
            if lines is not None:
                for line in lines:
                    x1,y1,x2,y2 = line[0]
                    try:
                        fn = Function.from_points((x2, y2), (x1, y1))
                    except ZeroDivisionError:
                        continue
                    function_list.append(fn)
                    #fn.draw(color_img, msg.info.height, (255,0,0))
                    # self.draw_line(a, b, color_img, msg.info.width, msg.info.height)
                    #cv2.line(color_img,(x1,y1),(x2,y2),(0,0,255),1)
                    offset = ((max(x2, x1) - min(x2, x1))/2, (max(y2, y1) - min(y2,y1))/2)
                    mid_point = (min(x2, x1) + offset[0], min(y2, y1) + offset[1])
                    #cv2.circle(color_img, (mid_point), 1, (255, 0, 0), 1)

            if len(function_list) < 2:
                # less than 2 lines, don't output anything
                return

            candidate_fns = []
            for fun1, fun2 in itertools.combinations(function_list, 2):
                a_diff = abs(fun1.a - fun2.a)
                b_diff = abs(fun1.b - fun2.b)

                if (a_diff <= 0.04 and b_diff >= 20 and b_diff <= 120):
                    #print("a diff: ", a_diff)
                    #print("b diff: ", b_diff)
                    candidate_fns.extend([fun1, fun2])

            if len(candidate_fns) > 0:
                candidate_fns.sort(key=lambda x: x.b)
                candidate_fns[0].draw(color_img, grid_trimmed.shape[1], (0, 255, 0))
                candidate_fns[-1].draw(color_img, grid_trimmed.shape[1], (0, 255, 0))
                line1 = candidate_fns[0]
                line2 = candidate_fns[-1]
                #To make both lines parallel, average their gradients:
                average_a = (line1.a+line2.a)/2
                average_b = (line1.b+line2.b)/2               
                line_average = Function(average_a, average_b)
                line_average.draw(color_img, grid_trimmed.shape[1], (255, 0, 0))
                #line1.a, line2.a = average_a
                linepoints.line_x1, linepoints.line_y1, linepoints.line_x2, linepoints.line_y2 = self.line_to_points(average_a, average_b)
            
            p1 = np.array([linepoints.line_x1, linepoints.line_y1])
            p2 = np.array([linepoints.line_x2, linepoints.line_y2])
            #vector_along_line = np.array([(linepoints.line_x2-linepoints.line_x1), (linepoints.line_y2-linepoints.line_y1)])
            vector_along_line = p2-p1
            vector_robot_to_p2 = -vector_robot+p1+vector_along_line
            angle_robot_to_line = np.arccos((np.dot(vector_robot_to_p2,vector_along_line))/(np.linalg.norm(vector_robot_to_p2)*np.linalg.norm(vector_along_line)))

            mag_robot_to_p2 = np.linalg.norm(vector_robot_to_p2)
            distance_robot_line = np.sin(angle_robot_to_line)*mag_robot_to_p2

            print(angle_robot_to_line)

            self.draw_origin(0, 2, color_img, int(self.imagereader.origin_x), int(self.imagereader.origin_y))
            self.draw_robot_orientation(angle, 20, color_img, robot_x, robot_y)

            ros_img = self.imagereader.bridge.cv2_to_imgmsg(color_img, encoding="rgb8")
            self.image_pub.publish(ros_img)
        r.sleep()

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


if __name__ == "__main__":
    rospy.init_node("lane_keeping", anonymous=False)
    lanekeeping = LaneKeeping()
    while not rospy.is_shutdown():
        try:
            lanekeeping.robot_position()
            #print(a)
            #rospy.Rate(10).sleep()
        except rospy.ROSInterruptException:
            pass

'''
if __name__ == "__main__":
    try:
        rospy.init_node("lane_keeping", anonymous=False)
        lk = LaneKeeping()
        #imagereader = ImageReader()
        lk.robot_position()
        rospy.spin()
        #print(a)
        #rospy.Rate(10).sleep()
    except rospy.ROSInterruptException:
        pass
'''
