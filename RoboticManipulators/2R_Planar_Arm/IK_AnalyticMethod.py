# Code to solve the inverse kinematics problem using an analytical approach  
# A is the reference frame
# B is the joint 2 frame

import numpy as np
from math import pi, acos, atan2, cos, sin
import matplotlib.pyplot as plt

# Input desired end effector position
ef_pos = [0, 2]

# Define link lengths
l1 = 1  # Link 1 length
l2 = 1  # Link 2 length

# Calculation to determine upward or downward elbow configuration using end effector position coordinate
if ef_pos[0] >= 0: elbow_up = -1
if ef_pos[0] < 0: elbow_up = 1

xd = np.sqrt(ef_pos[0]**2 + ef_pos[1]**2)
c2 = ( xd**2 - l1**2 - l2**2 )/(2*l1*l2)
theta2 = (elbow_up)*acos(c2)
theta = atan2(ef_pos[1], ef_pos[0])
theta1 = theta - atan2(l2*sin(theta2), l1+l2*cos(theta2))

############################## Forward Kinematics ##############################

def rot_z(theta, link_array):
    """ 
    Rotation matrix for rotation about z axis in 2D space
    Inputs:
        theta - angle of rotation
        link_array - 1D array containing x and y position of link end
    Ouput:
        rotated_z - final x and y position of link end after rotation
    """
    Rot_z = np.array([[cos(theta),-sin(theta)],
                      [sin(theta), cos(theta)]])

    rotated_z = np.dot(Rot_z, link_array)
    return rotated_z

# Calculate the end position of link 1
j1_pos_A = rot_z(theta1, np.array([l1, 0]))
# Calculate the end position of link 2
j2_pos_B = rot_z(theta2, np.array([l2, 0]))
# Calculate joint 2 end position in frame A
j2_pos_A = rot_z(theta1, j2_pos_B)

# Final end effector position
ef_pos = j2_pos_A + j1_pos_A

# Plot output for visual confirmation

x = np.array([0, j1_pos_A[0], ef_pos[0]])
y = np.array([0, j1_pos_A[1], ef_pos[1]])

figure, axes = plt.subplots()
plt.scatter( x , y , s = 70, color = "green")   # Plot circle at each joint
plt.plot(x, y, color = "red", linewidth = 5)   # Plot line between all joint ends 
plt.quiver(0, 0, l1, 0, color = "blue")   # Define origin x axis
plt.quiver(0, 0, 0, l1, color = "green")   # Define origin y axis
limits = [-(l1+l2+0.1), (l1+l2+0.1), -(l1+l2+0.1), (l1+l2+0.1)]   # Define x and y plotting limits
plt.axis(limits)
plt.grid()
plt.show()
