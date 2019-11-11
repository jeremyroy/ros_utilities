# GPS Averager Node

Author: Jeremy Roy

## Summary

Takes the average of GPS readings given at UTM coordinates over an Odom topic, until a stop trigger is received.  This is useful when trying to find the position of a stationary robot.

## Topics

### Input

Odom topic: varies as per `~odom_topic` parameter.  Input UTM coordinates as a nav_msgs/Odometry message.

Stop topic: varies as per `~stop_topic` parameter.  When received, the node computes the average of all received coordinates, and shutsdown.  Message type: std_msgs/Empty.

### Output

`odom_averaged`: A nav_msgs/Odometry message containing the averaged UTM coordinates.  This is only published once after the stop trigger has been received.

## Parameters:

`~odom_topic` - Name of input odom topic.  Default value: `/navsat/odom`

`~stop_topic` - Name of input stop topic.  Default value: `/stop_trigger`

## TODO

* Calculate variance of output average given position and variance of inputs.