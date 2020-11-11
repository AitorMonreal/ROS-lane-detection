#!/usr/bin/env python
import rospy
import actionlib
import deutsche_base.msg
import roslib
import math
import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from simple_pid import PID
import numpy as np
from collections import deque
####
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Robot():
  def __init__(self):
    self.min_linear_speed = rospy.get_param("min_linear_speed", )
    self.max_linear_speed = rospy.get_param("max_linear_speed", )
    self.min_angular_speed = rospy.get_param("min_angular_speed", )
    self.max_angular_speed = rospy.get_param("max_angular_speed", )
    self.base_frame = rospy.get_param("base_frame". "base_footprint")
    self.cmd_vel_topic = rospy.get_param("cml_vel_topic", "")
    self.control_linear_speed = 0
    self.control_angular_speed = 0
    self.exists = False
    self.position = None
    self.orientation = None
    self.timestamp = None
    self.distance_from_line1 = None
    self.distance_from_line2 = None
'''
class Lines():
  def __init__(self, robot):
    self.costmap_topic = rospy.get_param("costmap_topic", "/move_base/localcostmap/costmap") #specify a default value if the parameter doesn't exist - in this case, "/move_base/localcostmap/costmap" is the default if "costmap_topic" doesn't exist
    self.costmap_x_midpoint = None
    self.costmap_y_midpoint = None
    self.line1_x1 = None
    self.line1_x2 = None
    self.line1_y1 = None
    self.line1_y2 = None
    self.line2_x1 = None
    self.line2_x2 = None
    self.line2_y1 = None
    self.line2_y2 = None
    self.line1_coefficients = None
    self.line2_coefficients = None
    self.line1_gradient = None
    self.line2_gradient = None
'''
class Lines():
  def __init__(self, robot):
    self.costmap_topic = rospy.get_param("costmap_topic", "/move_base/localcostmap/costmap") #specify a default value if the parameter doesn't exist - in this case, "/move_base/localcostmap/costmap" is the default if "costmap_topic" doesn't exist
    self._bridge = CvBridge()
    self._quaternion_sub = rospy.Subscriber ('/odometry/filtered', Odometry, self.get_rotation)
    self.line1_x1 = None
    self.line1_x2 = None
    self.line1_y1 = None
    self.line1_y2 = None
    self.line2_x1 = None
    self.line2_x2 = None
    self.line2_y1 = None
    self.line2_y2 = None
    self.line1_coefficients = None
    self.line2_coefficients = None
    self.line1_gradient = None
    self.line2_gradient = None
    self.local_costmap_line_sub = rospy.Subscriber(self.lines.costmap_topic, OccupancyGrid, self.get_line_cb)

  def get_rotation(self, msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #return yaw
  
  def get_line_cb(self, msg):
    grid = np.asarray(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width)
    # change 0s to black
    grid_masked = np.ma.array(grid, mask=grid==0, fill_value=255)
    # flip the axes to match ROS representation
    grid_flipped = np.flip(grid_masked, 0)
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
    self.line1_x1 = int(x0 + 1000*(-b))
    self.line1_y1 = int(y0 + 1000*(a))
    self.line1_x2 = int(x0 - 1000*(-b))
    self.line1_y2 = int(y0 - 1000*(a))

    a = np.cos(theta2)
    b = np.sin(theta2)
    x0 = a*rho2
    y0 = b*rho2
    self.line2_x1 = int(x0 + 1000*(-b))
    self.line2_y1 = int(y0 + 1000*(a))
    self.line2_x2 = int(x0 - 1000*(-b))
    self.line2_y2 = int(y0 - 1000*(a))


class ToXPosition(object):
  _goal = deutsche_base.msg.LineGoal()
  _feedback = deutsche_base.msg.LineFeedback()
  _result = deutsche_base.msg.LineResult()

  def __init__(self, robot, lines):
    self.robot = robot
    self.lines = lines
    self.reference_frame = rospy.get_param("reference_frame", "map")
    self.robot_to_x_position_distance = 0
    self.robot_central_to_lines = 0
    self._action_name = name
    self.to_goal_setpoint_angular = self.lines.costmap_y_midpoint
    self.to_goal_kp_angular =
    self.to_goal_kd_angular =
    self.to_goal_ki_angular =
    self.to_goal_setpoint_linear = self._goal
    self.to_goal_kp_linear =
    self.to_goal_kd_linear =
    self.to_goal_ki_linear =
    self.to_goal_angular_speed_pid = PID(self.to_goal_kp_angular,self.to_goal_ki_angular,self.to_goal_kd_angular, self.to_goal_setpoint_angular)
    self.to_goal_angular_speed_pid.output_limits = (-self.robot.max_angular_speed, self.robot.max_angular_speed)
    self.to_goal_linear_speed_pid = PID(self.to_goal_kp_linear,self.to_goal_ki_linear,self.to_goal_kd_linear, self.to_goal_setpoint_linear)
    self.to_goal_linear_speed_pid.output_limits = (-self.robot.max_linear_speed, self.robot.max_linear_speed)
    self.align_angle_setpoint_angular = None
    self.align_angle_kp_angular =
    self.align_angle_kd_angular =
    self.align_angle_ki_angular =
    self.align_angle_angular_speed_pid = PID(self.align_angle_kp_angular,self.align_angle_ki_angular,self.align_angle_kd_angular,self.align_angle_setpoint_angular)
    self.align_angle_angular_speed_pid.output_limits = (-self.robot.max_angular_speed, self.robot.max_angular_speed)
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.control_publisher = rospy.Publisher(self.robot.cmd_vel_topic, Twist, queue_size = 0)
    self.control_message = Twist()
    self._as = actionlib.SimpleActionServer(self._action_name, deutsche_base.msg.ToXPosition, execute_cb = self.execute_cb, auto_start = False)
    self._as.Start()

  def execute_cb(self):
    self.local_costmap_sub = rospy.Subscriber(self.lines.costmap_topic, OccupancyGrid, self.get_costmap_midpoint_cb)
    r = rospy.Rate(15)
    reached_x_position = False
    previous_timestamp = None
    distance_from_goal = None
    angle_from_goal = None
    angle_difference = None
    to_goal_distance_estimation_list = deque(maxlen=10)
    to_goal_angle_estimation_list = deque(maxlen=5)
    align_angle_angle_estimation_list = deque(maxlen=5)
    radians_to_degree_constant = 57.2958

    #*get line eq*
    self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2, self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2 = ***
    self.lines.line1_coefficients = self.get_2D_line_coefficients(self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2)
    self.lines.line2_coefficients = self.get_2D_line_coefficients(self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2)
    self.lines.line1_gradient = self.get_line_slope(self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2)
    self.lines.line2_gradient = self.get_line_slope(self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2)

    self.robot.distance_from_line1 = self.get_2D_distance_to_line(self.lines.line1_coefficients[0], self.lines.line1_coefficients[1], self.lines.line1_coefficients[2], self.lines.costmap_x_midpoint, self.lines.costmap_y_midpoint)
    self.robot.distance_from_line2 = self.get_2D_distance_to_line(self.lines.line2_coefficients[0], self.lines.line2_coefficients[1], self.lines.line2_coefficients[2], self.lines.costmap_x_midpoint, self.lines.costmap_y_midpoint)

    #*get tf from footprint to map*
    self.robot.exists, self.robot.position, self.robot.orientation, self.robot.timestamp = self.get_transform(self.reference_frame, self.robot.base_frame)

    #*check if current pose = goal*
    while self.robot.exists and not rospy.is_shutdown():
      self.robot.exists = False
      self.robot.exists, self.robot.position, self.robot.orientation, self.robot.timestamp = self.get_transform(self.reference_frame, self.robot.base_frame)

      self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2, self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2 = ***
      self.lines.line1_coefficients = self.get_2D_line_coefficients(self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2)
      self.lines.line2_coefficients = self.get_2D_line_coefficients(self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2)
      self.lines.line1_gradient = self.get_line_slope(self.lines.line1_x1, self.lines.line1_x2, self.lines.line1_y1, self.lines.line1_y2)
      self.lines.line2_gradient = self.get_line_slope(self.lines.line2_x1, self.lines.line2_x2, self.lines.line2_y1, self.lines.line2_y2)

      self.robot.distance_from_line1 = self.get_2D_distance_to_line(self.lines.line1_coefficients[0], self.lines.line1_coefficients[1], self.lines.line1_coefficients[2], self.lines.costmap_x_midpoint, self.lines.costmap_y_midpoint)
      self.robot.distance_from_line2 = self.get_2D_distance_to_line(self.lines.line2_coefficients[0], self.lines.line2_coefficients[1], self.lines.line2_coefficients[2], self.lines.costmap_x_midpoint, self.lines.costmap_y_midpoint)

      if self.robot.timestamp != previous_timestamp:
        if self.robot.position.x != self._goal:

          distance_from_goal = self_goal - self.robot.position.x

          if self.robot.distance_from_line1 < self.robot.distance_from_line2:
            angle_from_goal = -self.robot.distance_from_line1
          elif self.robot.distance_from_line2 < self.robot.distance_from_line1:
            angle_from_goal = self.robot.distance_from_line2
          else:
            angle_from_goal = 0

          to_goal_distance_estimation_list.append(distance_from_goal)
          to_goal_distance_estimation_running_mean = float(sum(to_goal_distance_estimation_list))/float(len(to_goal_distance_estimation_list))

          to_goal_angle_estimation_list.append(angle_from_goal)
          to_goal_angle_estimation_running_mean = float(sum(to_goal_angle_estimation_list))/float(len(to_goal_angle_estimation_list))

          to_goal_linear_speed = self.to_goal_linear_speed_pid(to_goal_distance_estimation_running_mean)
          to_goal_angular_speed = self.to_goal_angular_speed_pid(to_goal_angle_estimation_running_mean)

          if self._goal > self.robot.position.x:
            self.control_message.angular.z = to_goal_angular_speed
            self.control_message.linear.x = to_goal_linear_speed
          else:
            self.control_message.angular.z = to_goal_angular_speed
            self.control_message.linear.x = -to_goal_linear_speed

          self.control_publisher.publish(self.control_message)

          previous_timestamp = self.robot.timestamp

          self._feedback.distance_to_goal = distance_from_goal

        else:

          self.robot_angle = euler_from_quaternion(self.robot.orientation)[2] * radians_to_degree_constant
          self.line1_angle = math.atan(self.line1_gradient)
          self.line2_angle = math.atan(self.line2_gradient)
          self.line_angle_average = (self.line1_angle + self.line2_angle) / 2

          if self.robot_angle != self.line_angle_average:
            angle_difference = self.line_angle_average = self.robot_angle

            align_angle_angle_estimation_list.append(angle_difference)
            align_angle_estimation_running_mean = float(sum(align_angle_angle_estimation_list))/float(len(align_angle_angle_estimation_list))

            align_angle_angular_speed = align_angle_angular_speed_pid(align_angle_estimation_running_mean)

            self.control_message.angular.z = align_angle_angular_speed
            self.control_message.linear.x = 0
            self.control_publisher.publish(self.control_message)

            previous_timestamp = self.robot.timestamp

            self._feedback.angle_to_goal = angle_difference
          else:

            self.control_message.linear.x = 0
            self.control_message.angular.z = 0
            self.control_publisher.publish(self.control_message)
            reached_x_position = True
            break

      else:
          reached_x_position = False
          rospy.loginfo('%s: Cant detect a change in the TF of the robot, retrying...' % self._action_name)


    #*send succeeded*
    reached_x_position = True
    if reached_x_position:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_secceeded(True)
    else:
      rospy.loginfo('%s: Failed, could not reach goal position' % self._action_name)
      slef._as.set_secceeded(False)

  def get_costmap_midpoint_cb(self, msg):
    self.lines.costmap_x_midpoint = msg.info.width/2
    self.lines.costmap_y_midpoint = msg.info.height/2

  def get_transform(self,from_link, to_link):
    try:
        trans = self.tfBuffer.lookup_transform(from_link, to_link, rospy.Time())
        return True, trans.transform.translation, trans.transform.rotation, trans.header.stamp
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Cannot find the transform: " + str(e))
        return False, 0, 0, 0
        #print position, quaternion


def get_2D_distance(position):
  distance = math.sqrt(position.x**2 + position.y**2)
  return distance

def get_2D_line_coefficients(x1,x2,y1,y2):
  # y = mx + b
  #Ax + By + C = 0
  #m = -A / B
  a = -(y2-y1)
  b =  (x2-x1)
  #using point 2
  c =-(a*x2 + b*y2)
  coefficients = [a,b,c]
  return coefficients

def get_line_slope(x1,x2,y1,y2):
  y = y2 - y1
  x = x2 - x1
  slope = y/x
  return slope

def get_2D_distance_to_line(a, b, c, x, y):
  distance = (a*x + b*y + c)/math.sqrt(a*a + b*b)
  return distance

def euler_from_quaternion(quaternion):
  quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
  euler = tf.transformations.euler_from_quaternion(quat)
  return euler
