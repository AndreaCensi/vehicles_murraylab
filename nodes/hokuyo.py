#!/usr/bin/env python
import roslib; roslib.load_manifest('landroid_murraylab')
import rospy, traceback
import time
from math import pi
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from hokuyod_client import HokuyoMux as HokuyodClient
from hokuyod_client import HokuyoMuxError as HokuyodClientError

def hokuyod_client_node():   
    rospy.init_node('hokuyod_client_node')
    
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 1338)
    
    rospy.loginfo('Trying to connect to hokuyod %s:%s.' % (host, port))
    
    publisher = rospy.Publisher('~scan', LaserScan, latch=True)

    hokuyod_client = HokuyodClient(host, port)

    rospy.loginfo('Connected to hokuyod at %s:%s.' % (host, port))

    count = 0
    attempts = 0
    start = time.time()
    while not rospy.is_shutdown():
        attempts += 1
        try:
            data = hokuyod_client.read(100) # read until data within 100ms
        except HokuyodClientError as e:
            rospy.logerr('Warning, error: %s' % e) 
            continue
        if attempts % 50 == 0:
            hz = (time.time() - start)/ attempts
            rospy.loginfo('Attempts %5d; read %5d; hz=%.2f' % 
                          (attempts, count, hz))
            
        readings = data['readings']
        reading2meters = 0.001
        fields = {
            'header' : Header(stamp=rospy.Time.from_sec(data["host_time"]/1000.),
                              frame_id="base_laser"),
            'angle_min': (data["min_idx"]-data["mid_idx"])*2*pi/1024,
            'angle_max': (data["max_idx"]-data["mid_idx"])*2*pi/1024,
            'range_min': data['min_dist']*reading2meters,
            'range_max': data['max_dist']*reading2meters,
            'angle_increment': 2*pi/1024,
            'time_increment': data["deltat"],
            'scan_time': data["deltat"],
            'ranges': [r * reading2meters for r in readings]
        }
        msg = LaserScan(**fields)
        publisher.publish(msg)
        count += 1
        
if __name__ == '__main__':
    hokuyod_client_node()
