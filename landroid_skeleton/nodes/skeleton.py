#!/usr/bin/env python
"""
Skeleton code for reading laser scans and commanding motion to the Landroid

based heavily on code by Alex Jose during summer 2012.
SCL; 12 Dec 2012.
"""

import roslib; roslib.load_manifest('landroid_skeleton')
import rospy
import math
from sensor_msgs.msg import LaserScan
from landroid_murraylab.msg import ldr_tracks


class BotInterface:
    def __init__(self):
        self.ranges = []
        self.lstart = 0
        self.lend = 0
        self.lstep = 0
        self.lmax = 0
        self.lmin = 0
        self.count = 0
        rospy.init_node('bot_int', anonymous=True)
        pubbed_topics = rospy.get_published_topics()
        # Prefer forwarded topics, if available.
        if "/hforward/scan" in [tinfo[0] for tinfo in pubbed_topics]:
            self.laser_topicname = "/hforward/scan"
        else:
            self.laser_topicname = "/hokuyod_client_node/scan"
        self.mtr_cont = rospy.Publisher('/track_input', ldr_tracks)
        rospy.sleep(.1)  # apparently need to wait for >0.05 'seconds'

    def readLaser(self,data, dump=False):
        self.ranges = list(data.ranges)
        self.lstart = data.angle_min
        self.lend = data.angle_max
        self.lstep = data.angle_increment
        self.lmax = data.range_max
        self.lmin = data.range_min
        if dump:
            # write to a file, for debugging
            f = open('scandump' + str(self.count) + '.dmp', 'w+')
            for r in list(data.ranges):
                f.write("%s\n" % r)
            f.close()
            self.count += 1

    def updateLaser(self):
        try: 
            data = rospy.wait_for_message(self.laser_topicname, LaserScan, timeout=1.)
            if data: 
                self.readLaser(data)
        except Exception,e:
            print e

    def setMotors(self, l,r):
        try: self.mtr_cont.publish(ldr_tracks(left=l,right=r))
        except Exception,e: print e

    def stop(self):
        self.setMotors(0,0)

    def setLin(self,l):
        # Drive forward (roughly along a line); positive means forward.
        self.setMotors(l,l)

    def setRot(self,r):
        # Rotate in place; positive means counter-clockwise.
        self.setMotors(-r,r)
        

if __name__ == '__main__':
    verbose = 0
    bi = BotInterface()
    bi.stop()
    bi.updateLaser()

    while not rospy.is_shutdown():
        bi.updateLaser()

        # MAIN LOOP...
        #
        # E.g., to drive forward at a moderate speed, try 
        #bi.setLin(45)
