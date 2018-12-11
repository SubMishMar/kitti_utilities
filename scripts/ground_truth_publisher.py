#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node('ground_truth_publisher')

    pose_pub = rospy.Publisher('/kitti/Pose', PoseStamped, queue_size=10)

    pose = PoseStamped()

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        
        time = rospy.get_rostime()
        
        try:
            (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print([trans, rot])
        pose.header.stamp = time
        pose.header.frame_id = "world"
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        pose_pub.publish(pose)
        rate.sleep()