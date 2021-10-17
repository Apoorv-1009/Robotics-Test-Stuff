# Code to implement Forward And Backward Repeated Inverse Kinematics 
import numpy as np
from math import sin, cos, atan2
import kinematics as kin
import matplotlib.pyplot as plt

# Input desired goal position
final_pos = np.array([1.5, 1])

# Define link lengths
l1, l2 = 1, 1

# Input current end effector position
ef_pos = np.array([2, 0])
theta1, theta2 = kin.ik_solver(ef_pos[0], ef_pos[1], l1, l2, 1)

# Define acceptable error:
error_xy = 0.000001
error = np.linalg.norm((final_pos-ef_pos))
i = 0
elbow_pos = [l1*cos(theta1), l1*sin(theta1)]
while error > error_xy and i < 20:
    # Find pseudo elbow position
    elbow_pos_dash = ((elbow_pos - final_pos)/(np.linalg.norm(elbow_pos - final_pos)))*l2 + final_pos

    # Find actual elbow position
    elbow_pos = ((elbow_pos_dash)/(np.linalg.norm(elbow_pos_dash)))*l1

    # Find new end effector position
    ef_pos = ((final_pos - elbow_pos)/(np.linalg.norm(final_pos - elbow_pos)))*l2 + elbow_pos

    error = np.linalg.norm((final_pos-ef_pos))
    i+=1

theta1 = np.degrees(atan2(elbow_pos[1], elbow_pos[0]))
theta2 = np.degrees(atan2(ef_pos[1]-elbow_pos[1], ef_pos[0]-elbow_pos[0])) - theta1

# Plot output for visual confirmation
x = np.array([0, elbow_pos[0], ef_pos[0]])
y = np.array([0, elbow_pos[1], ef_pos[1]])

figure, axes = plt.subplots()
plt.scatter( x , y , s = 70, color = "green")   # Plot circle at each joint
plt.plot(x, y, color = "red", linewidth = 5)   # Plot line between all joint ends 
plt.quiver(0, 0, l1, 0, color = "blue")   # Define origin x axis
plt.quiver(0, 0, 0, l1, color = "green")   # Define origin y axis
limits = [-(l1+l2+0.1), (l1+l2+0.1), -(l1+l2+0.1), (l1+l2+0.1)]   # Define x and y plotting limits
plt.axis(limits)
plt.grid()
plt.show()

