#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        
        self.ns = rospy.get_namespace().strip('/')
        if not self.ns:
            self.ns = 'robot1'
        
        rospy.loginfo("start: %s", self.ns)
        
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.obstacles = []
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.Subscriber('/{}/scan'.format(self.ns), LaserScan, self.laser_callback)
        rospy.Subscriber('/{}/odom'.format(self.ns), Odometry, self.odom_callback)
        
        self.data_pub = rospy.Publisher('/{}/robot_detected_data'.format(self.ns),
                                       Float32MultiArray, queue_size=1)
        
        self.marker_pub = rospy.Publisher('/{}/obstacle_markers'.format(self.ns),
                                         MarkerArray, queue_size=1)
        
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.position[2] = math.atan2(siny_cosp, cosy_cosp)
        
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y
        self.velocity[2] = msg.twist.twist.angular.z

    def laser_callback(self, msg):
        self.obstacles = []
        
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        valid_indices = np.where((ranges > msg.range_min) & (ranges < msg.range_max))[0]
        
        for i in valid_indices:
            if ranges[i] < 2.0:
                local_x = ranges[i] * math.cos(angles[i])
                local_y = ranges[i] * math.sin(angles[i])
                
                cos_theta = math.cos(self.position[2])
                sin_theta = math.sin(self.position[2])
                
                global_x = self.position[0] + local_x * cos_theta - local_y * sin_theta
                global_y = self.position[1] + local_x * sin_theta + local_y * cos_theta
                
                self.obstacles.append([global_x, global_y, 0.1])
        
        self.obstacles = self.cluster_obstacles(self.obstacles)
        
        self.publish_detected_data()
        
        self.publish_obstacle_markers()

    def cluster_obstacles(self, obstacles, cluster_distance=0.3):
        if not obstacles:
            return []
        
        clustered = []
        obstacles = np.array(obstacles)
        used = np.zeros(len(obstacles), dtype=bool)
        
        for i in range(len(obstacles)):
            if used[i]:
                continue
                
            cluster = [obstacles[i]]
            used[i] = True
            
            for j in range(i+1, len(obstacles)):
                if used[j]:
                    continue
                    
                dist = np.linalg.norm(obstacles[i][:2] - obstacles[j][:2])
                if dist < cluster_distance:
                    cluster.append(obstacles[j])
                    used[j] = True
            
            cluster = np.array(cluster)
            center_x = np.mean(cluster[:, 0])
            center_y = np.mean(cluster[:, 1])
            max_radius = np.max(cluster[:, 2])
            
            clustered.append([center_x, center_y, max_radius])
        
        return clustered

    def publish_obstacle_markers(self):
        try:
            marker_array = MarkerArray()

            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "{}_obstacles".format(self.ns)
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            
            for i, obstacle in enumerate(self.obstacles):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "{}_obstacles".format(self.ns)
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose.position.x = obstacle[0]
                marker.pose.position.y = obstacle[1]
                marker.pose.position.z = 0.0
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = obstacle[2] * 2
                marker.scale.y = obstacle[2] * 2
                marker.scale.z = 0.5
                
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.7
                
                marker.lifetime = rospy.Duration(0.5)
                
                marker_array.markers.append(marker)
            
            for i, obstacle in enumerate(self.obstacles):
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "{}_obstacle_labels".format(self.ns)
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = obstacle[0]
                text_marker.pose.position.y = obstacle[1]
                text_marker.pose.position.z = 0.8
                
                text_marker.pose.orientation.w = 1.0
                
                text_marker.text = "Obs{}".format(i+1)
                
                text_marker.scale.z = 0.2
                
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                text_marker.lifetime = rospy.Duration(0.5)
                
                marker_array.markers.append(text_marker)

            self.marker_pub.publish(marker_array)
            
            rospy.logdebug("total obstatles %d ", len(self.obstacles))
            
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def publish_detected_data(self):
        try:
            msg = Float32MultiArray()
            msg.data = []
            
            msg.data.extend(self.position.tolist())
            
            msg.data.extend(self.velocity.tolist())
            
            msg.data.append(len(self.obstacles))
            
            for obs in self.obstacles:
                msg.data.extend(obs)
            
            self.data_pub.publish(msg)
            
            rospy.loginfo("pub: pos=%s, num=%d",
                         str(self.position.tolist()), len(self.obstacles))
            
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("close...")
                break

if __name__ == '__main__':
    try:
        node = ObstacleDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("ERROR: %s", str(e))
