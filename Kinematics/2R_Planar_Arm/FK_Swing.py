# Code to swing the robotic arm given intial joint angles and rate of increment
import numpy as np
from numpy import pi, sin, cos
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Define joint angles
theta1 = np.radians(0)   # First pitch angle
theta2 = np.radians(0)   # Second pitch angle

# Define link lengths
l1 = 1   # Link 1 length
l2 = 1   # Link 2 length

increment = 0.01 # Angle increment in radian

# Create an angle array between 0 to a limiting angle we desire
theta1_list = np.arange(theta1, pi/2,increment)
theta2_list = np.arange(theta2, pi/2,increment)

# Create array of final x and y positions for each joint
X1=np.zeros(len(theta1_list))   # Array of link 1 x positions
Y1=np.zeros(len(theta1_list))   # Array of link 1 y positions
X2=np.zeros(len(theta2_list))   # Array of link 2 x positions
Y2=np.zeros(len(theta2_list))   # Array of link 2 y positions

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

indexx = 0
# Find the link 1 and link 2 end positions for each angle
for index, theta1 in enumerate(theta1_list, start=0):
    j1_pos_A = rot_z(theta1, np.array([l1, 0]))
    X1[index] = j1_pos_A[0]   # Link 1 x-position
    Y1[index] = j1_pos_A[1]   # Link 1 y-position
    j2_pos_A = rot_z(theta1_list[indexx], rot_z(theta2_list[indexx], np.array([l2, 0])))
    # ef_pos = np.array([X1[index]+j2_pos_A[0], Y1[index]+j2_pos_A[1]])
    X2[index] = X1[index]+j2_pos_A[0]   # Link 2 x-position
    Y2[index] = Y1[index]+j2_pos_A[1]   # Link 2 y-position
    indexx += 1

# Plot output for visual confirmation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=True)
limits = [-(l1+l2+0.5), (l1+l2+0.5), -(l1+l2+0.5), (l1+l2+0.5)]   # Define x and y plotting limits
ax.axis(limits)
ax.grid()
ax.set_title('Arm Sway')
line, = ax.plot([], [], 'o-', lw=5, color='#de2d26')
path = ax.plot(0, 0)[0]

x_path, y_path = [], []

# Initialization function
def init():
    line.set_data([], [])
    path.set_data(x_path, y_path)
    return line, path

# Animation function
def animate(i):
    # Plot arm
    x_points = [0, X1[i], X2[i]]
    y_points = [0, Y1[i], Y2[i]]
    line.set_data(x_points, y_points)

    # Plot path followed
    x_path.append(X2[i])
    y_path.append(Y2[i])
    path.set_xdata(x_path)
    path.set_ydata(y_path)
    return line, path

# Call the animation
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(X1), interval=40, blit=True, repeat=False)

#show the animation
plt.show()