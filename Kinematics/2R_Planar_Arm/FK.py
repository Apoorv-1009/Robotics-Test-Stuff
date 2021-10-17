# Code to deteremine end-effector position given joint angles using rotation matrices
# A is the reference frame
# B is the joint 2 frame

import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt

# Define joint angles
theta1 = np.radians(90)   # First pitch angle
theta2 = np.radians(-179)   # Second pitch angle

# Define link lengths
l1 = 1   # Link 1 length
l2 = 1   # Link 2 length

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


