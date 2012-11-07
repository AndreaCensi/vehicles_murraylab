#!/usr/bin/env python

import roslib; roslib.load_manifest('landroid_murraylab')
import rospy
from landroid_murraylab.msg import ldr_tracks
from ldr import *

class tracks(ldr):

    def callback(self, data):
        self.setTrackMode(TRACK_MODE_LINKED, data.left, data.right)
 
    def start(self):
        # need to monitor the motor flag and the front, left, and right wall sensors,
        ldr.start(self, reglist=[0x110, 0x111], interval=100)
        self.sub = rospy.Subscriber('track_input', ldr_tracks, self.callback) 
    
    def cleanup(self):
        self.debug(0, "cleanup")
        self.halt()
        ldr.cleanup(self)

    def update(self, d):
        pass

if __name__ == '__main__':
    d = tracks('tracks', 'localhost', 1337, verbose=0)
    d.start()
    rospy.init_node("ldr_tracks", anonymous=True)
    rospy.spin()
    d.cleanup()
