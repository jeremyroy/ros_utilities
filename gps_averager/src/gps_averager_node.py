#!/usr/bin/env python
import roslib
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomAverager:
    def __init__(self):
        # Initialize member variables
        self.odom_count = 0.0
        self.odom_sum = [0.0, 0.0, 0.0]
        self.yaw_sum = 0.0
        self.odom_offset = [0.0, 0.0, 0.0]

        # Subscribe to a GPS odom topic
        odom_topic = rospy.get_param("~odom_topic", "/navsat/odom")
        path_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_summer)
        stop_topic = rospy.get_param("~stop_topic", "/stop_trigger")
        stop_sub = rospy.Subscriber(stop_topic, Empty, self.stop_summing)

        # Create odom publisher with averaged output pt.
        self.pub_odom = rospy.Publisher("odom_averaged", Odometry, queue_size=1)


    # Function to calculate yaw angle from quaternion in Pose message
    def headingcalc(self, pose):
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

    def odom_summer(self, odom):
        if self.odom_count == 0.0:
            self.odom_offset[0] = odom.pose.pose.position.x
            self.odom_offset[1] = odom.pose.pose.position.y
            self.odom_offset[2] = odom.pose.pose.position.z
        self.odom_sum[0] += odom.pose.pose.position.x - self.odom_offset[0]
        self.odom_sum[1] += odom.pose.pose.position.y - self.odom_offset[1]
        self.odom_sum[2] += odom.pose.pose.position.z - self.odom_offset[2]
        self.yaw_sum += self.headingcalc(odom.pose.pose)
        self.odom_count += 1.0

    def stop_summing(self, trigger):
        odom_average = [0.0, 0.0, 0.0]
        # Calculate averages
        odom_average[0] = (self.odom_sum[0]/self.odom_count) + self.odom_offset[0]
        odom_average[1] = (self.odom_sum[1]/self.odom_count) + self.odom_offset[1]
        odom_average[2] = (self.odom_sum[2]/self.odom_count) + self.odom_offset[2]
        yaw_average = self.yaw_sum / self.odom_count
        # Print averages
        rospy.loginfo("Averages:\n\tx:\t%f\n\ty:\t%f\n\tz:\t%f\n\tyaw:\t%f", odom_average[0], odom_average[1], odom_average[2], yaw_average)
        # Publish odometry message
        odom = Odometry()
        odom.pose.pose.position.x = odom_average[0]
        odom.pose.pose.position.y = odom_average[1]
        odom.pose.pose.position.z = odom_average[2]
        quat = quaternion_from_euler(0.0, 0.0, yaw_average)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.header.frame_id = "odom_combined"
        self.pub_odom.publish(odom)
        # Reset summer
        self.odom_count = 0.0
        self.odom_sum = [0.0, 0.0, 0.0]
        self.odom_offset = [0.0, 0.0, 0.0]
        # Kill node
        rospy.signal_shutdown("Done computing average.  Exiting...")

#################
# Main function #
#################
def main():
    # Initialize the node
    rospy.init_node('gps_averager', anonymous=True)

    # Create odom averager object
    odom_agerager = OdomAverager()
    
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