import numpy as np
import matplotlib.pyplot as plt
import math

gps_xy = np.loadtxt("./gps_xy_points.txt", delimiter=',')

img1_xy = np.loadtxt("./tj_83_1/img1_xy_points.txt", delimiter=',')
img2_xy = np.loadtxt("./tj_83_2/img2_xy_points.txt", delimiter=',')
img3_xy = np.loadtxt("./tj_83_3/img3_xy_points.txt", delimiter=',')
img4_xy = np.loadtxt("./tj_83_4/img4_xy_points.txt", delimiter=',')

error1 = np.fabs(img1_xy - gps_xy)
avg_error1= np.sqrt(error1[:,0]*error1[:,0] + error1[:,1]*error1[:,1])
avg_error1 = np.sum(avg_error1)/len(avg_error1)
print("avg_error1:",avg_error1)

error2 = np.fabs(img2_xy - gps_xy)
avg_error2= np.sqrt(error2[:,0]*error2[:,0] + error2[:,1]*error2[:,1])
avg_error2 = np.sum(avg_error2)/len(avg_error2)
print("avg_error2:",avg_error2)

error3 = np.fabs(img3_xy - gps_xy)
avg_error3= np.sqrt(error3[:,0]*error3[:,0] + error3[:,1]*error3[:,1])
avg_error3 = np.sum(avg_error3)/len(avg_error3)
print("avg_error3:",avg_error3)

error4 = np.fabs(img4_xy - gps_xy)
avg_error4= np.sqrt(error4[:,0]*error4[:,0] + error4[:,1]*error4[:,1])
avg_error4 = np.sum(avg_error4)/len(avg_error4)
print("avg_error4:",avg_error4)

error12=np.fabs(img1_xy - img2_xy)
avg_error12= np.sqrt(error12[:,0]*error12[:,0] + error12[:,1]*error12[:,1])
avg_error12 = np.sum(avg_error12)/len(avg_error12)
print("avg_error12:",avg_error12)

gps_x = gps_xy[:,0]
gps_y = gps_xy[:,1]

img1_x = img1_xy[:,0]
img1_y = img1_xy[:,1]

img2_x = img2_xy[:,0]
img2_y = img2_xy[:,1]

img3_x = img3_xy[:,0]
img3_y = img3_xy[:,1]

img4_x = img4_xy[:,0]
img4_y = img4_xy[:,1]

color_black = '#000000'
color_blue = '#0000FF'
color_gold = '#FFD700'

plt.scatter(gps_x, gps_y, c=color_blue, label='gps')
plt.scatter(img1_x, img1_y, c=color_gold,label='img_1')
plt.scatter(img2_x, img2_y, label='img_2')
plt.scatter(img3_x, img3_y, label='img_3')
plt.scatter(img4_x, img4_y, label='img_4')

plt.legend()
plt.show()
