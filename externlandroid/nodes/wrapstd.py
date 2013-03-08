#!/usr/bin/env python
"""
Present data streamed from Landroid, available in messages provided
through the landroid_murraylab package, in more common ROS types.

Twist message linear velocities are in units of meters/second, and
rotational units are radians/second.


SCL; 21 Feb 2013.
"""

import roslib; roslib.load_manifest("externlandroid")
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from trackem_ros.msg import MTCalPoints
from landroid_murraylab.msg import ldr_tracks
import numpy as np
import numpy.linalg as la


class TrackForward(rospy.Publisher):
    def __init__(self, name):
        self.left_speed = 0
        self.right_speed = 0
        self.factor = 342  # Linear approximation to unit conversion
        rospy.Publisher.__init__(self, name, ldr_tracks)

    def __call__(self, data):
        # Only support nonzero "linear x" and "angular z" velocities.
        self.left_speed = self.factor*data.linear.x
        self.right_speed = self.factor*data.linear.x
        diff = (self.factor/8.)*data.angular.z
        self.left_speed -= diff
        self.right_speed += diff
        self.publish(ldr_tracks(left=int(self.left_speed),
                                right=int(self.right_speed)))

    def spin(self):
        self.publish(ldr_tracks(left=int(self.left_speed),
                                right=int(self.right_speed)))


class PoseForward(rospy.Publisher):
    def __init__(self, name):
        self.x, self.y, self.theta = None, None, None
        self.br = tf.TransformBroadcaster()
        rospy.Publisher.__init__(self, name, Odometry)

    def __call__(self, data):
        points = np.array([[p.x, p.y] for p in data.points])
        if points.shape[0] >= 3:
            now = rospy.Time.now()
            self.x, self.y = np.mean(points[:3,:], axis=0)
            self.theta = self.est_orientation(points)
            fdata = Odometry()
            fdata.pose.pose.position.x = self.x
            fdata.pose.pose.position.y = self.y
            fdata.pose.pose.orientation.x, fdata.pose.pose.orientation.y, fdata.pose.pose.orientation.z, fdata.pose.pose.orientation.w = tf.transformations.quaternion_from_euler(0., 0., self.theta)
            fdata.header.stamp.secs = now.secs
            fdata.header.stamp.nsecs = now.nsecs
            fdata.header.frame_id = "world"
            self.br.sendTransform((self.x, self.y, 0),
                                  tf.transformations.quaternion_from_euler(0., 0., self.theta),
                                  now, "base_link", "world")
            self.br.sendTransform((0., 0., 0.),
                                  tf.transformations.quaternion_from_euler(0., 0., 0.),
                                  now, "world", "odom")
            self.publish(fdata)
            

    def est_orientation(self, points):
        """Rough estimation of orientation from three points

        Given three points in the plane, find the two that are
        mutually closest, take their mean as the base point, and then
        find angle to the third point (farthest from the cluster).
        """
        # Compute by hand, generalize later if needed.
        min_dist = la.norm(points[0]-points[1])
        min_idx = [0,1]
        if la.norm(points[0]-points[2]) < min_dist:
            min_dist = la.norm(points[0]-points[2])
            min_idx = [0,2]
        if la.norm(points[1]-points[2]) < min_dist:
            min_dist = la.norm(points[1]-points[2])
            min_idx = [1,2]
        base = np.mean(points[min_idx,:],axis=0)
        for other_idx in range(3):
            if other_idx not in min_idx:
                break
        return np.arctan2(points[other_idx][1]-base[1], points[other_idx][0]-base[0])

    def get_pose(self, blocking=False):
        """Return pose, optionally blocking until one is available.
        """
        if self.x is None or self.y is None or self.theta is None:
            if not blocking:
                raise ValueError("requested pose is undefined.")
            else:
                while self.x is None or self.y is None or self.theta is None:
                    pass
        return (self.x, self.y, self.theta)


class LaserForward(rospy.Publisher):
    def __init__(self,name):
        rospy.Publisher.__init__(self, name, LaserScan)
        now = rospy.Time.now()
        self.br = tf.TransformBroadcaster()

    def __call__(self, data):
        now = rospy.Time.now()
        data.header.stamp.secs = now.secs
        data.header.stamp.nsecs = now.nsecs
        data.header.frame_id = "scanner0"
        data.time_increment = 0.0
        data.scan_time = 0.0
        self.br.sendTransform((.1, 0, 0),
                              tf.transformations.quaternion_from_euler(0.,0.,0.),
                              now, "scanner0", "base_link")
        self.publish(data)


if __name__ == "__main__":
    rospy.init_node("landroid0")
    ppub = PoseForward("~odom")
    psub = rospy.Subscriber("/trackem/calpoints", MTCalPoints, callback=ppub)
    lpub = LaserForward("~base_scan")
    lsub = rospy.Subscriber("/hokuyod_client_node/scan", LaserScan, callback=lpub)
    tpub = TrackForward("/track_input")
    tsub = rospy.Subscriber("~cmd_vel", Twist, callback=tpub)

    rate = rospy.Rate(5.)
    while not rospy.is_shutdown():
        # tpub.spin()
        rate.sleep()
