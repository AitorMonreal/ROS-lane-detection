#!/usr/bin/env python
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math

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
        print("pt1: ",pt1)
        pt2 = (x_max, int(self.a * x_max + self.b))
        print("pt2: ",pt2)
        cv2.line(image, pt1, pt2, color)

class LaneKeeping():
    def __init__(self):
        self.local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.grid_cb)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
    def grid_cb(self, msg):
        grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
        # change 0s to black
        grid_masked = np.ma.array(grid, mask=grid==0, fill_value=255)
        # flip the axes to match ROS representation
        grid_flipped = np.flip(grid_masked, 0)
        img_cv = cv2.resize(grid_flipped,(msg.info.height, msg.info.width))
        # gray = cv2.cvtColor(grid_flipped,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grid_flipped,0,150,apertureSize = 3)
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
                print line[0]
                try:
                    fn = Function.from_points((x2, y2), (x1, y1))
                except ZeroDivisionError:
                    continue
                function_list.append(fn)
                fn.draw(color_img, msg.info.height, (255,0,0))
                # self.draw_line(a, b, color_img, msg.info.width, msg.info.height)
                cv2.line(color_img,(x1,y1),(x2,y2),(0,0,255),1)
                offset = ((max(x2, x1) - min(x2, x1))/2, (max(y2, y1) - min(y2,y1))/2)
                mid_point = (min(x2, x1) + offset[0], min(y2, y1) + offset[1])
                cv2.circle(color_img, (mid_point), 1, (255, 0, 0), 1)
        # if len(function_list) < 2:
        #     return
        # for i in range(0, len(fn))
        ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding="rgb8")
        self.image_pub.publish(ros_img)

if __name__ == "__main__":
    try:
        rospy.init_node("lane_keeping", anonymous=False)
        lk = LaneKeeping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





