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
        self.line1_x1=0
        self.line1_x2=0
        self.line1_y1=0
        self.line1_y2=0
        self.line2_x1=0
        self.line2_x2=0
        self.line2_y1=0
        self.line2_y2=0

linepoints = LinePoints()

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

class LaneKeeping():
    def __init__(self):
        # self.local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.grid_cb)
        self._local_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.grid_cb)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def grid_cb(self, msg):
        grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
        exists, position, orientation, timestamp = self.get_transform("map", "base_footprint")
        #exists, position, orientation, timestamp = self.get_transform("base_footprint", "map")
        grid_res = msg.info.resolution
        # change 0s to black
        grid_masked = np.ma.array(grid, mask=grid==0, fill_value=255)
        # flip the axes to match ROS representation
        grid_flipped = np.flip(grid_masked, 0)
        #grid_flipped = grid_flipped[40:80, :]
        
        #origin_x = msg.info.origin.position.x*(1/grid_res)
        #origin_y = msg.info.origin.position.y*(1/grid_res)

        origin_x = -msg.info.origin.position.x*(1/grid_res)
        origin_y = grid_flipped.shape[0] + msg.info.origin.position.y*(1/grid_res)

        angle = -self.euler_from_quaternion(orientation)[2]*(180/np.pi)
        robot_x = position.x*(1/grid_res)
        robot_y = position.y*(1/grid_res)

        robot_x_centre = int(origin_x+robot_x)
        robot_y_centre = int(origin_y+robot_y)
        print("Height " + str(grid_flipped.shape[1]))
        print("Width " + str(grid_flipped.shape[0]))
        print("X map origin " + str(origin_x))
        print("Y map origin " + str(origin_y))
        print("X robot origin " + str(robot_x))
        print("Y robot origin " + str(robot_y))
        print("X total origin " + str(robot_x_centre))
        print("Y total origin " + str(robot_y_centre))
        print("X from centre " + str((grid_flipped.shape[1]/2)+robot_x_centre))
        print("Y from centre " + str((grid_flipped.shape[0]/2)+robot_y_centre))
        #print(angle)
        print('-------')

        #img_cv = cv2.resize(grid_flipped,(msg.info.height, msg.info.width))
        img_cv = cv2.resize(grid_flipped,(grid_flipped.shape[1], grid_flipped.shape[0]))  # .shape[1] defines the height, while .shape[0] defines the image width
        #img_cv = grid_flipped
        # gray = cv2.cvtColor(grid_flipped,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(img_cv,0,150,apertureSize = 3)
        lines = cv2.HoughLinesP(image=edges,
                                rho=1,
                                theta=np.pi/180,
                                threshold=5,
                                minLineLength=15,
                                maxLineGap=2)
        color_img = cv2.cvtColor(img_cv, cv2.COLOR_GRAY2RGB)
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
            candidate_fns[0].draw(color_img, msg.info.width, (0, 255, 0))
            candidate_fns[-1].draw(color_img, msg.info.width, (0, 255, 0))         
            line1 = candidate_fns[0]
            line2 = candidate_fns[1]
            #To make both lines parallel, average their gradients:
            #average_a = (line1.a+line2.a)/2
            #line1.a, line2.a = average_a
            linepoints.line1_x1, linepoints.line1_y1, linepoints.line1_x2, linepoints.line1_y2 = self.line_to_points(line1.a, line1.b)
            linepoints.line2_x1, linepoints.line2_y1, linepoints.line2_x2, linepoints.line2_y2 = self.line_to_points(line2.a, line2.b)

        #self.draw_robot_orientation(angle, 20, color_img, grid_flipped.shape[0], grid_flipped.shape[0])
        self.draw_robot_orientation(angle, 20, color_img, robot_x_centre, robot_y_centre)
        #self.draw_robot_orientation(angle, -20, color_img, (grid_flipped.shape[0]/2)+robot_x_centre, (grid_flipped.shape[1]/2)+robot_y_centre)        self.draw_origin(0, 1, color_img, origin_x, origin_y)

        #self.draw_origin(0, 5, color_img, (grid_flipped.shape[1]/2)+int(origin_x), (grid_flipped.shape[0]/2)+int(origin_y))    
        self.draw_origin(0, 5, color_img, int(origin_x), int(origin_y))
        self.draw_robot_orientation(0, 5, color_img, grid_flipped.shape[1]+int(robot_x), grid_flipped.shape[0]+int(robot_y))


        ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding="rgb8")
        self.image_pub.publish(ros_img)

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

'''
if __name__ == "__main__":
    while True:
        try:
            rospy.init_node("lane_keeping", anonymous=False)
            lk = LaneKeeping()
            a = lk.get_points()
            print(a)
            rospy.Rate(10).sleep()
            #rospy.spin()
        except rospy.ROSInterruptException:
            pass
'''

if __name__ == "__main__":
    try:
        rospy.init_node("lane_keeping", anonymous=False)
        lk = LaneKeeping()
        a = lk.get_points()
        rospy.spin()
        #print(a)
        #rospy.Rate(10).sleep()
    except rospy.ROSInterruptException:
        pass