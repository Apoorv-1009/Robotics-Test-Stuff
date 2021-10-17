# Code to implement velocity kinematics given initial and final end effector position
import kinematics as kin
import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt

def jacobian(q1,q2, l1=1, l2=1):
    """
    Function to calculate the elements of the jacobian matrix
    for a 2R planar manipulator
    Derived by differentiating the FK equation w.r.t time
    Inputs:
        q1 - angle of first revolute joint (degrees)
        q2 - angle of second revolute joint w.r.t first link axis (degrees)

    Outputs:
        Function returns the matrix elements
    """
    q1, q2 = np.radians(q1), np.radians(q2)

    J11 = -l1*sin(q1) - l2*sin(q1+q2)
    J12 = -l2*sin(q1+q2)
    J21 = l1*cos(q1) + l2*cos(q1+q2)
    J22 = l2*cos(q1+q2)

    return J11, J12, J21, J22

# Define link lengths
l1, l2 = 1, 1

# Define current end effector position
xi, yi = 1.0, -1.0
print("initial x: ", xi)
print("initial y: ", yi)
# Create a copy element x, y 
x, y = xi, yi

# Calculate initial joint angles given initial position
theta1, theta2 = kin.ik_solver(x, y)
print("Theta1: ", theta1)
print("Theta2: ", theta2)

# Define desired end effector position
xf, yf = 1.0, 1.0

# Calculate cartesian rate of change values
t = 10
dt = 0.1
xdot = (xf-xi)/t
ydot = (yf-yi)/t
print("xdot: ", xdot)
print("ydot: ", ydot)

# Define lists for plotting later
x_list, y_list = [], []
omega1_list, omega2_list = [], []
theta1_list, theta2_list = [], []
distance_list = []

# Calculate the distance from initial position for tracking
distance_vector = np.sqrt((xf-xi)**2 + (yf-yi)**2)
distance = 0
print("distance_vector: ", distance_vector )
i = 0
print("\n")

while distance_vector-distance > 0.01 and i<1000:

    # Calculate jacobian matrix
    J11, J12, J21, J22 = jacobian(theta1, theta2)
    # Calculate determinant of the jacobian matrix:
    det_jacobian = 1/(J11*J22 - J12*J21)
    # Calculate inverse of the jacobian elements
    J11inv = det_jacobian*J22
    J12inv = det_jacobian*(-J12)
    J21inv = det_jacobian*(-J21)
    J22inv = det_jacobian*(J11)

    # Calculate angular velocity
    theta1_dot = np.degrees(J11inv*xdot + J12inv*ydot)
    omega1_list.append(theta1_dot)
    theta2_dot = np.degrees(J21inv*xdot + J22inv*ydot)
    omega2_list.append(theta2_dot)
    # print("omega1: ", theta1_dot)
    # print("omega2: ", theta2_dot)

    # Calculate new angles
    theta1 = theta1 + theta1_dot*dt
    theta2 = theta2 + theta2_dot*dt
    theta1_list.append(theta1)
    theta2_list.append(theta2)
    # print("Theta1: ", theta1)
    # print("Theta2: ", theta2)

    # Calculate new position
    x, y = kin.fk_solver(theta1, theta2)
    x_list.append(x)
    y_list.append(y)
    # print("new x: ", x)
    # print("new y: ", y)

    # Calculate distance value
    distance = np.sqrt((xi-x)**2 + (yi-y)**2)
    distance_list.append(distance)
    # print("distance: ", distance)
    i+=1

    # Calculate new velocity based on new position
    xdot, ydot = (xf-x)/t, (yf-y)/t
    print("\n")
    if theta1 > 360 or theta2 > 360: 
        print("Values exceding the 360 degrees limit")
        break 

# fig, axs = plt.subplots(nrows=3,ncols=2, figsize=(20,20))
# fig.canvas.set_window_title('Velocity Kinematics Data')
# axs[0, 0].grid(), axs[0, 1].grid()
# axs[1, 0].grid(), axs[1, 1].grid()
# axs[2, 0].grid(), axs[2, 1].grid()

# # Plot end effector path
# limits = [-(l1+l2+0.1), (l1+l2+0.1), -(l1+l2+0.1), (l1+l2+0.1)]   # Define x and y plotting limits
# axs[0, 0].axis(limits)
# axs[0, 0].plot(x_list, y_list, 'g.', markersize=1)
# axs[0, 0].plot(xi, yi, 'r+', markersize=20)
# axs[0, 0].plot(xf, yf, 'b+', markersize=20)
# axs[0, 0].set_title("End effector path")

# # Plot distance values
# axs[0, 1].plot(distance_list)
# axs[0, 1].set_title("Distance travelled by end effector")

# # Plot theta1_dot values
# axs[1, 0].plot(omega1_list)
# axs[1, 0].set_title("Omega1 values")

# # Plot theta2_dot values
# axs[1, 1].plot(omega2_list)
# axs[1, 1].set_title("Omega2 values")

# # Plot theta1 values
# axs[2, 0].plot(theta1_list)
# axs[2, 0].set_title("Theta1 values")

# # Plot theta2 values
# axs[2, 1].plot(theta2_list)
# axs[2, 1].set_title("Theta2 values")

# fig.tight_layout()
# plt.plot()
# plt.show()

# Call the animate function to animate the motion of the planar arm
kin.animate(np.radians(theta1_list), np.radians(theta2_list), interval=10)







