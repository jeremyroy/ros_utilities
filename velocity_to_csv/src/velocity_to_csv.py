#!/usr/bin/env python

import rospy
from mining_msgs.msg import Velocity
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import csv

####################
# Controller class #
####################
class Velocity2CSV:
    # Constructor
    def __init__(self):
        # Get ros parameters
        outfile = rospy.get_param('~outfile', 'velocity.csv')
        input_topic = rospy.get_param('~input_topic', 'odom')
        input_msg_type = rospy.get_param('~input_msg_type', 'Odometry')

        # Set up subscribers
        if input_msg_type == 'Odometry':
            self.velocity_sub = rospy.Subscriber(input_topic, Odometry, self.odomCallback)
        elif input_msg_type == 'Velocity':
            self.velocity_sub = rospy.Subscriber(input_topic, Velocity, self.velocityCallback)
        elif input_msg_type == 'Float64':
            self.velocity_sub = rospy.Subscriber(input_topic, Float64, self.floatCallback)
        self.write_file = open(outfile, 'w')
        self.csv_writer = csv.writer(self.write_file)
    

    def __del__(self):
        self.write_file.close()


    def odomCallback(self, msg):
        velocity = msg.twist.twist.linear.x
        time = msg.header.stamp.secs + (msg.header.stamp.nsecs/1e9)
        self.csv_writer.writerow([str(time),str(velocity)])

    def velocityCallback(self, msg):
        velocity = msg.velocity
        time = rospy.Time.now().to_sec()
        self.csv_writer.writerow([str(time),str(velocity)])

    def floatCallback(self, msg):
        velocity = msg.data
        time = rospy.Time.now().to_sec()
        self.csv_writer.writerow([str(time),str(velocity)])


#################
# Main function #
#################
def main():
    # Initialize the node
    rospy.init_node('vel_to_csv', anonymous=True)

    # Create the csv writer
    csv_vel_writer = Velocity2CSV()
    
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