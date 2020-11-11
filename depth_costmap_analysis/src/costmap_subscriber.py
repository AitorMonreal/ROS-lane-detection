#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import os

def my_callback(msg):
    rospy.loginfo('Hello')
    #rospy.loginfo(msg.time.map_load_time)
    #new = np.asarray(msg.data)
    #print(new[1:10, 1:10])
    new = msg.data
    print(new)
    np_new = np.asarray(new)
    print(np_new)
    print(np_new.shape)
    print(np_new.size)
    #msg.info.width = 200
    #msg.info.height = 200
    final = np.reshape(np_new, (200,200))
    print(final)
    os.chdir('/home/aitor/ros/deutsche_ws')
    np.save('db_static_v1.npy', final)
    plt.imsave('db_static_v1.png', final)
    rospy.is_shutdown()

rospy.init_node('costmap_reader')
rate = rospy.Rate(1)
rate.sleep()
sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, my_callback)
print('Hello2')
rate.sleep()
rospy.spin()


'''
def listener():
    print('Hello3')
    rospy.init_node('costmap_reader')
    rate = rospy.Rate(0.5)
    rate.sleep()
    sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, my_callback)
    print('Hello4')
    rate.sleep()
    rospy.spin()
    print('Hello5')


if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
'''