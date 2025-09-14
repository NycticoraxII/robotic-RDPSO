#!/usr/bin/env python

import rospy
import tf2_ros
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import traceback

class MultiRobotVisualizer:
    def __init__(self):
        rospy.init_node('multi_robot_visualizer', anonymous=True)
        
        self.target_x = rospy.get_param('~target_x', 5.0)
        self.target_y = rospy.get_param('~target_y', -5.0)
        self.target_yaw = rospy.get_param('~target_yaw', 0.0)
        
        self.center_radius = 0.1
        self.ring_radii = [0.3, 0.6, 1.0]
        self.ring_alphas = [0.8, 0.5, 0.3]
        self.ring_width = 0.05
        
        self.num_robots = 5
        self.robot_radius = 0.2
        self.robot_height = 0.1
        
        self.robot_colors = [
            (218/255.0, 47/255.0, 78/255.0),
            (0.0, 1.0, 0.0),
            (1.0, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (171/255.0, 51/255.0, 221/255.0),
        ]
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot_positions = {}
        self.robot_connected = [False] * self.num_robots
        
        self.target_marker_pub = rospy.Publisher('/target_visualization', MarkerArray, queue_size=1, latch=True)
        self.robot_marker_pub = rospy.Publisher('/robot_visualization', MarkerArray, queue_size=1)
        self.target_timer = rospy.Timer(rospy.Duration(5.0), self.publish_target_markers)
        
        self.robot_update_timer = rospy.Timer(rospy.Duration(0.1), self.update_robot_positions)
        
        self.robot_publish_timer = rospy.Timer(rospy.Duration(0.2), self.publish_robot_markers)
        
        self.publish_target_markers(None)
        
        rospy.loginfo("start")
        rospy.loginfo("pos: [%.2f, %.2f, %.2f]", self.target_x, self.target_y, self.target_yaw)
        rospy.loginfo(" %d agent TF: robot1/base_footprint ~ robot%d/base_footprint",
                     self.num_robots, self.num_robots)
    
    def publish_target_markers(self, event):
        try:
            marker_array = MarkerArray()
            
            center_marker = self.create_target_center_marker()
            marker_array.markers.append(center_marker)
            
            for i, (radius, alpha) in enumerate(zip(self.ring_radii, self.ring_alphas)):
                ring_marker = self.create_target_ring_marker(i + 1, radius, alpha)
                marker_array.markers.append(ring_marker)
            
            arrow_marker = self.create_target_arrow_marker()
            marker_array.markers.append(arrow_marker)
            
            self.target_marker_pub.publish(marker_array)
            rospy.loginfo("pub success")
            
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def create_target_center_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_center"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.01
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = self.center_radius * 2
        marker.scale.y = self.center_radius * 2
        marker.scale.z = 0.02
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0)
        
        return marker

    def create_target_ring_marker(self, ring_id, radius, alpha):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_rings"
        marker.id = ring_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.005
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = self.ring_width
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = alpha
        
        num_points = 64
        for i in range(num_points + 1):
            angle = 2.0 * math.pi * i / num_points
            point = Point()
            point.x = self.target_x + radius * math.cos(angle)
            point.y = self.target_y + radius * math.sin(angle)
            point.z = 0.005
            marker.points.append(point)
        
        marker.lifetime = rospy.Duration(0)
        
        return marker

    def create_target_arrow_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.1
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.target_yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.target_yaw / 2.0)
        
        marker.scale.x = 0.4
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0)
        
        return marker

    def update_robot_positions(self, event):
        try:
            for i in range(self.num_robots):
                robot_frame = "robot{}/base_footprint".format(i + 1)
                
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        robot_frame,
                        rospy.Time(0),
                        rospy.Duration(0.1)
                    )
                    
                    position = transform.transform.translation
                    orientation = transform.transform.rotation
                    
                    yaw = self.quaternion_to_yaw(orientation)
                    
                    self.robot_positions[i] = {
                        'x': position.x,
                        'y': position.y,
                        'z': position.z,
                        'yaw': yaw,
                        'timestamp': rospy.Time.now()
                    }
                    
                    if not self.robot_connected[i]:
                        self.robot_connected[i] = True
                        rospy.loginfo("agent%d: (%.2f, %.2f)", i+1, position.x, position.y)
                        
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    if self.robot_connected[i]:
                        rospy.logwarn("agent%d TF failed: %s", i+1, str(e))
                        self.robot_connected[i] = False
                    continue
                    
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def quaternion_to_yaw(self, quaternion):
        try:
            siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return yaw
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))
            return 0.0

    def publish_robot_markers(self, event):
        try:
            marker_array = MarkerArray()
            
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "robots"
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            
            for i in range(self.num_robots):
                if not self.robot_connected[i] or i not in self.robot_positions:
                    continue
                
                try:
                    robot_marker = self.create_robot_marker(i)
                    if robot_marker:
                        marker_array.markers.append(robot_marker)
                    
                    arrow_marker = self.create_robot_direction_marker(i)
                    if arrow_marker:
                        marker_array.markers.append(arrow_marker)
                    
                    text_marker = self.create_robot_text_marker(i)
                    if text_marker:
                        marker_array.markers.append(text_marker)
                        
                except Exception as e:
                    rospy.logerr("agent%dERROR: %s", i+1, str(e))
                    continue
            
            if len(marker_array.markers) > 1:
                self.robot_marker_pub.publish(marker_array)
                
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def create_robot_marker(self, robot_id):
        try:
            if robot_id not in self.robot_positions:
                return None
                
            pos = self.robot_positions[robot_id]
            color = self.robot_colors[robot_id % len(self.robot_colors)]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_bodies"
            marker.id = robot_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = pos['z'] + self.robot_height / 2
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = self.robot_radius * 2
            marker.scale.y = self.robot_radius * 2
            marker.scale.z = self.robot_height
            
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(1.0)
            
            return marker
            
        except Exception as e:
            rospy.logerr("agent%dERROR: %s", robot_id+1, str(e))
            return None

    def create_robot_direction_marker(self, robot_id):
        try:
            if robot_id not in self.robot_positions:
                return None
                
            pos = self.robot_positions[robot_id]
            color = self.robot_colors[robot_id % len(self.robot_colors)]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_directions"
            marker.id = robot_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = pos['z'] + self.robot_height + 0.05
            
            yaw = pos['yaw']
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            
            marker.scale.x = self.robot_radius * 1.2
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            marker.color.r = color[0] * 0.7
            marker.color.g = color[1] * 0.7
            marker.color.b = color[2] * 0.7
            marker.color.a = 1.0
            
            marker.lifetime = rospy.Duration(1.0)
            
            return marker
            
        except Exception as e:
            rospy.logerr("agent%dERROR: %s", robot_id+1, str(e))
            return None

    def create_robot_text_marker(self, robot_id):
        try:
            if robot_id not in self.robot_positions:
                return None
                
            pos = self.robot_positions[robot_id]
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_labels"
            marker.id = robot_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = pos['z'] + self.robot_height + 0.3
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.text = "R{}".format(robot_id + 1)
            
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker.lifetime = rospy.Duration(1.0)
            
            return marker
            
        except Exception as e:
            rospy.logerr("agent%dERROR: %s", robot_id+1, str(e))
            return None

    def check_robot_timeouts(self):
        try:
            current_time = rospy.Time.now()
            timeout_duration = rospy.Duration(2.0)
            
            for i in range(self.num_robots):
                if (self.robot_connected[i] and 
                    i in self.robot_positions and
                    (current_time - self.robot_positions[i]['timestamp']) > timeout_duration):
                    
                    self.robot_connected[i] = False
                    rospy.logwarn("agent%dfailed", i+1)
                    
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def run(self):
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            try:
                self.check_robot_timeouts()
                
                connected_count = sum(self.robot_connected)
                if connected_count > 0:
                    rospy.loginfo_throttle(10, "num: %d/5", connected_count)
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("ERROR: %s", str(e))
                rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = MultiRobotVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("STOP")
    except Exception as e:
        rospy.logerr("FAILED: %s", str(e))
        rospy.logerr("ERROR: %s", traceback.format_exc())
