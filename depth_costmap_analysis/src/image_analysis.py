#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

image = np.load('/home/aitor/ros/deutsche_ws/db_static_v1.npy')

for i in range(image.shape[0]):
    for j in range(image.shape[1]):
        if image[i,j] > 40:
            image[i,j] = 1
        else:
            image[i,j] = 0

#plt.imsave('db_static_1_0_v1.png', image)

top_left = image[:100, :100]
top_right = image[:100, 100:]
bot_left = image[100:, :100]
bot_right = image[100:, 100:]



x, y = np.nonzero(image)
fig1, ax = plt.subplots()
#ax.scatter(x, y)  # to get it like the image we just have to rotate it by 90deg clockwise
#plt.show()

x_top_left, y_top_left = np.nonzero(top_left)
ax.scatter(x_top_left, y_top_left)

#m, b = np.polyfit(x_top_left, y_top_left, 1)
#plt.plot(x, m*x + b)
#plt.show()



n = 2 #number of principal components
from sklearn.decomposition import PCA
pca = PCA(n_components=n)
top_left_array = np.zeros((2, len(x_top_left)))
top_left_array[0, :] = x_top_left
top_left_array[1, :] = y_top_left
pca.fit(top_left_array)

print(pca.explained_variance_ratio_) #the amount of variance explained by each principal component
print(pca.explained_variance_ratio_.cumsum())

print('done')