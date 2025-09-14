#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher', anonymous=True)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.odom_subs = []
        for i in range(1, 6):
            sub = rospy.Subscriber('/robot{}/odom'.format(i), Odometry, 
                                 lambda msg, idx=i: self.odom_callback(msg, idx))
            self.odom_subs.append(sub)
        
        self.publish_static_transforms()
        
        rospy.loginfo("TF PUBLISHER START")

    def publish_static_transforms(self):
        transforms = []
        
        map_to_world = TransformStamped()
        map_to_world.header.stamp = rospy.Time.now()
        map_to_world.header.frame_id = "map"
        map_to_world.child_frame_id = "world"
        map_to_world.transform.translation.x = 0.0
        map_to_world.transform.translation.y = 0.0
        map_to_world.transform.translation.z = 0.0
        map_to_world.transform.rotation.x = 0.0
        map_to_world.transform.rotation.y = 0.0
        map_to_world.transform.rotation.z = 0.0
        map_to_world.transform.rotation.w = 1.0
        transforms.append(map_to_world)
        
        self.static_tf_broadcaster.sendTransform(transforms)

    def odom_callback(self, msg, robot_id):
        try:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "robot{}/odom".format(robot_id)
            
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            rospy.logerr("TFERROR robot{}: {}".format(robot_id, str(e)))

    def run(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            try:
                self.publish_static_transforms()
                rate.sleep()
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    try:
        tf_pub = TFPublisher()
        tf_pub.run()
    except rospy.ROSInterruptException:
        pass
