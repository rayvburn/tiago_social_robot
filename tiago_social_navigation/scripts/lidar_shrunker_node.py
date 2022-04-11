#!/usr/bin/env python

import rospy
import copy
import sys

from sensor_msgs.msg import LaserScan
from numpy import inf

# Run `rostopic echo /scan_raw` and check the edge values (at the beginning of the array and at the end).
# You'll find the threshold that way.
LIDAR_BASE_DIST = 0.13

def callback(input):
    # create a copy
    output = copy.copy(input)
    ranges_list = list(output.ranges)

    # modify list, put infs where it's needed
    for i, range_val in enumerate(ranges_list):
        if range_val <= LIDAR_BASE_DIST:
            ranges_list[i] = float(inf)

    # update output message with the newly created list
    output.ranges = tuple(ranges_list)

    # publish
    pub.publish(output)

if __name__=="__main__":
    if len(sys.argv) < 3:
        print("usage: lidar_shrunker_node.py <input_scan_topic> <output_scan_topic>")
        exit(1)

    input_topic = rospy.resolve_name(str(sys.argv[1]))
    output_topic = rospy.resolve_name(str(sys.argv[2]))

    rospy.init_node('lidar_shrunker_node')
    rospy.loginfo("Created 'lidar_shrunker_node' node")

    pub = rospy.Publisher(output_topic, LaserScan, queue_size=5)
    rospy.Subscriber(input_topic, LaserScan, callback)
    rospy.loginfo("Created subscriber of %s and publisher of %s", input_topic, output_topic)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
