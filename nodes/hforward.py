#!/usr/bin/env python
"""
Forward LaserScan messages from hokuyod_client_node/scan, verbatim.
This is a workaround for weird network configurations.


SCL; 9 September 2012.
"""
import roslib; roslib.load_manifest("landroid_murraylab")
from sensor_msgs.msg import LaserScan
import rospy
import socket
import struct
import sys
import numpy as np


class HForward:
    def __init__(self):
        rospy.init_node("hforward")
        self.fpub = rospy.Publisher("~scan", LaserScan)
        self.fsub = rospy.Subscriber("/hokuyod_client_node/scan", LaserScan, self.callback)

    def callback(self, data):
        self.fpub.publish(data)

if __name__ == "__main__":
    hf = HForward()
    rospy.spin()
