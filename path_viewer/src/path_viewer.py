#!/usr/bin/env python
import roslib
import rospy
from nav_msgs.msg import Path

from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt

# Define globals
arrow_length = 0.1

# Function to calculate yaw angle from quaternion in Pose message
def headingcalc(pose):
    # Build quaternion from orientation
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    # Find all Euler angles from quaternion transformation
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]      # Extract yaw angle from Euler angles in radians
    return yaw

# def plotPath2D(path):
#     global arrow_length
#     # Extract path from nav_msgs/Path structure, put in array-like structure
#     for pose in path.poses:
#         # Extract 2D position
#         x = pose.pose.position.x
#         y = pose.pose.position.y
#         # Extract yaw
#         yaw = headingcalc(pose.pose)
#         # Calculate dx, dy
#         dx = arrow_length * np.cos(yaw)
#         dy = arrow_length * np.sin(yaw)
#         # Add arrow to plot
#         plt.arrow(x, y, dx, dy)
#     # Draw plot
#     plt.draw()

def plotPath2D(path):
    global arrow_length
    # Extract path from nav_msgs/Path structure, put in array-like structure
    x, y = [], [] # 2D position
    dx, dy = [], [] # tip of arrow (to show orientation)
    for pose in path.poses:
        # Extract 2D position
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)
        # Extract yaw
        yaw = headingcalc(pose.pose)
        # Calculate dx, dy
        dx.append(arrow_length * np.cos(yaw))
        dy.append(arrow_length * np.sin(yaw))
    
    fig, ax = plt.subplots()
    ax.plot(x,y,alpha=0.7)
    ax.quiver(x, y, dx, dy, units='width', scale=5, headwidth=3, headlength=5, width=0.002, color='r')
    ax.axis('equal')
    plt.show()

#################
# Main function #
#################
def main():
    # Initialize the node
    rospy.init_node('path_viewer', anonymous=True)

    # Enable interactive mode in plotting

    # Subscribe to a path topic
    path_topic = rospy.get_param("~path_topic", "/path_recorded")
    path_sub = rospy.Subscriber(path_topic, Path, plotPath2D)
    
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

##########################
# Entry point to program #
##########################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass