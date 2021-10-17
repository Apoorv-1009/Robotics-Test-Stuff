# My kinematics library for the 2R planar arm
import numpy as np
from math import  acos, atan2, sin, cos
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
sys.dont_write_bytecode = True
np.set_printoptions(suppress=True)

def rot_z(theta, link_array):
    """ 
    Rotation matrix for rotation about z axis in 2D space
    Inputs:
        theta - angle of rotation (radians)
        link_array - 1D array containing x and y position [x, y]
    Ouput:
        rotated_z - final x and y position after rotation [x, y]
    """
    Rot_z = np.array([[cos(theta),-sin(theta)],
                      [sin(theta), cos(theta)]])

    rotated_z = np.dot(Rot_z, link_array)
    return rotated_z

def fk_solver(theta1=0, theta2=0, l1=1, l2=1):
    """
    Function to compute the end effector position 
    Inputs:
        theta1 - joint angle of first link (degrees)
        theta2 - joint angle of second link with respect to first link shaft (degrees)
        l1 - length of first link
        l2 - length of second link
    Outputs:
        ef_pos - end effector position [x, y]
    """
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)

    # Calculate the end position of link 1
    j1_pos_A = rot_z(theta1, np.array([l1, 0]))
    # Calculate the end position of link 2
    j2_pos_B = rot_z(theta2, np.array([l2, 0]))
    # Calculate joint 2 end position in frame A
    j2_pos_A = rot_z(theta1, j2_pos_B)

    # Final end effector position
    ef_pos = j2_pos_A + j1_pos_A
    return np.round(ef_pos, 5)

def ik_solver(x, y, l1=1, l2=1, elbow=1):
    """
    Function to return the end effector position 
    Inputs:
        x - x coordinate of the desired position
        y - y coordinate of the desired position
        l1 - length of first link
        l2 - length of second link
        elbow - enter 1 for elbow up configuration, -1 for elbow down configuration
    Outputs:
        [theta1, theta2] - joint angles for first and second links (degrees)
    """

    xd = np.sqrt(x**2 + y**2)
    c2 = ( xd**2 - l1**2 - l2**2 )/(2*l1*l2)
    theta2 = -1*(elbow)*acos(c2)
    theta = atan2(y, x)
    theta1 = theta - atan2(l2*sin(theta2), l1+l2*cos(theta2))
    return [np.degrees(np.round(theta1, 5)), np.degrees(np.round(theta2, 5))]

def fabrik_solver(xi, yi, xf, yf, l1=1, l2=1, elbow=1, precision = 0.000001):
    """
    Function to return end effector position given current arm positions
    Inputs:
        xi - x coordinate of current end-effector position
        yi - y coordinate of current end-effector position
        xf - x coordinate of the desired position
        yf - y coordinate of the desired position
        l1 - length of first link
        l2 - length of second link
        elbow - enter 1 if elbow up configuration, -1 if elbow down configuration
        precision - acceptable error range for the FABRIK solution
    Outputs:
        [theta1, theta2] - joint angles for first and second links (degrees)
    """
    theta1, theta2 = ik_solver(xf, yf, l1, l2, elbow)

    ef_pos = np.array([xi, yi])
    final_pos = np.array([xf, yf])
    # Define acceptable error:
    error_xy = precision
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
    return [theta1, theta2]

def animate(theta1_list, theta2_list, l1=1, l2=1, interval = 10):
    """
    Function to animate the robotic arm motion given a list of joint angles
    Inputs:
        theta1_list - list of joint angles of first link (radians)
        theta2_list - list of joint angles of second link with respect to first link shaft
        l1 - link 1 length
        l2 - link 2 length
    Outputs:
        Animated motion of robotic arm moving through the given angles

    """
    # Create array of final x and y positions for each joint
    X1=np.zeros(len(theta1_list))   # Array of link 1 x positions
    Y1=np.zeros(len(theta1_list))   # Array of link 1 y positions
    X2=np.zeros(len(theta2_list))   # Array of link 2 x positions
    Y2=np.zeros(len(theta2_list))   # Array of link 2 y positions

    indexx = 0
    # Find the link 1 and link 2 end positions for each angle
    for index, theta1 in enumerate(theta1_list, start=0):
        j1_pos_A = rot_z(theta1, np.array([l1, 0]))
        X1[index] = j1_pos_A[0]   # Link 1 x-position
        Y1[index] = j1_pos_A[1]   # Link 1 y-position
        j2_pos_A = rot_z(theta1_list[indexx], rot_z(theta2_list[indexx], np.array([l2, 0])))
        X2[index] = X1[index]+j2_pos_A[0]   # Link 2 x-position
        Y2[index] = Y1[index]+j2_pos_A[1]   # Link 2 y-position
        indexx += 1

    # Plot output for visual confirmation
    fig = plt.figure()
    fig.canvas.set_window_title("Animation")
    ax = fig.add_subplot(111, autoscale_on=True)
    limits = [-(l1+l2+0.5), (l1+l2+0.5), -(l1+l2+0.5), (l1+l2+0.5)]   # Define x and y plotting limits
    ax.axis(limits)
    ax.grid()
    ax.set_title('Arm Sway')
    line, = ax.plot([], [], 'o-', lw=5, color='#de2d26')
    path = ax.plot(0, 0)[0]

    x, y = np.zeros(len(theta1_list)), np.zeros(len(theta2_list))
    for i in range(len(theta1_list)):
        x[i], y[i] = fk_solver(np.degrees(theta1_list[i]), np.degrees(theta2_list[i]))

    x_path, y_path = [], []

    # Initialization function
    def init():
        line.set_data([], [])
        path.set_data(0, 0)
        return line, path

    # Animation function
    def animate(i):
        # Plot arm
        x_points = [0, X1[i], X2[i]]
        y_points = [0, Y1[i], Y2[i]]
        line.set_data(x_points, y_points)

        # Plot path followed
        x_path.append(x[i])
        y_path.append(y[i])
        path.set_xdata(x_path)
        path.set_ydata(y_path)
        return line, path

    # Call the animation
    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(theta1_list)-1, interval=interval, blit=True, repeat=False)
    plt.show()