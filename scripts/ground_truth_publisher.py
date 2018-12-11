#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import tf
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

def publishInitTransform(time, trans_init, rot_init):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = time
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "init_frame"
  
    static_transformStamped.transform.translation.x = trans_init[0]
    static_transformStamped.transform.translation.y = trans_init[1]
    static_transformStamped.transform.translation.z = trans_init[2]
  
    static_transformStamped.transform.rotation.x = rot_init[0]
    static_transformStamped.transform.rotation.y = rot_init[1]
    static_transformStamped.transform.rotation.z = rot_init[2]
    static_transformStamped.transform.rotation.w = rot_init[3]
    broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':

    rospy.init_node('ground_truth_publisher')

    pose_pub = rospy.Publisher('/kitti/Pose', PoseStamped, queue_size=10)

    pose = PoseStamped()

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    firstTime = True
    while not rospy.is_shutdown():
        
        time = rospy.get_rostime()
        
        try:
            (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        if firstTime:
        	trans_init = trans
        	rot_init = rot
        	firstTime = False
        
        publishInitTransform(time, trans_init, rot_init)

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