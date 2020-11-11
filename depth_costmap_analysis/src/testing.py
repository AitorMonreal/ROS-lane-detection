#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('/home/aitor/ros/deutsche_ws/grid_flipped.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
low_threshold = 50
high_threshold = 150
edges = cv2.Canny(gray,low_threshold,high_threshold,apertureSize = 5)
plt.imsave('/home/aitor/ros/deutsche_ws/edges.png', edges)


rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 20  # minimum number of votes (intersections in Hough grid cell)

lines = cv2.HoughLines(edges,rho,theta,threshold)
# plt.imsave('/home/aitor/ros/deutsche_ws/lines.png', lines)
#lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
one = lines[0]
print(one)
chosen_lines = np.zeros((140,1,2))
i = 0
chosen_lines = lines
chosen_lines = np.empty((0,2), int)
for line in lines:
    for rho,theta in line:
        if theta > (88*(np.pi/180)) and theta < (92*(np.pi/180)):
            #chosen_lines[i,:,:].append([rho, theta])
            #chosen_lines = np.delete(chosen_lines, i, axis = 0)
            chosen_lines = np.append(chosen_lines, np.array([[rho, theta]]), axis=0)
            #chosen_lines
            i +=1
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            #cv2.line(img, (x1,y1), (x2,y2),(0,0,255),1)
            #cv2.line(img, pt1, pt2, (0,0,255), 3)
rho_mean = np.mean(chosen_lines, axis=0)[0]
rho1_arr = np.array([])
theta1_arr = np.array([])
rho2_arr = np.array([])
theta2_arr = np.array([])
for row in chosen_lines:
    print(row[0])
    if row[0] > rho_mean:
        print('Hello')
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
cv2.line(img, (x1,y1), (x2,y2),(0,0,255),1)

a = np.cos(theta2)
b = np.sin(theta2)
x0 = a*rho2
y0 = b*rho2
x1 = int(x0 + 1000*(-b))
y1 = int(y0 + 1000*(a))
x2 = int(x0 - 1000*(-b))
y2 = int(y0 - 1000*(a))
cv2.line(img, (x1,y1), (x2,y2),(0,0,255),1)

# Green color in BGR 
color = (0, 255, 0) 
theta = np.pi/180
theta = 96*(np.pi/180)
rho = 100
a = np.cos(theta)
b = np.sin(theta)
x0 = a*rho
y0 = b*rho
x1 = int(x0 + 1000*(-b))
y1 = int(y0 + 1000*(a))
x2 = int(x0 - 1000*(-b))
y2 = int(y0 - 1000*(a))
cv2.line(img, (x1,y1), (x2,y2),color,1)

cv2.imwrite('/home/aitor/ros/deutsche_ws/houghlines3.png',img)
plt.imshow(img)
plt.show()
print('done')