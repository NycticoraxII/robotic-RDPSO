#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import tf2_ros
from nav_msgs.msg import Odometry

class RDPSOAgent:
    def __init__(self):
        rospy.init_node('rdpso_agent', anonymous=True)

        self.ns = rospy.get_namespace().strip('/')
        if not self.ns:
            self.ns = 'robot1'
        
        self.robot_id = int(self.ns[-1])
        
        rospy.loginfo("RDPSO Agent: %s, ID: %d", self.ns, self.robot_id)
        
        self.init_parameters()
        
        self.init_state_variables()
        
        self.setup_communication()
        
        self.rate = rospy.Rate(10)  # 10Hz
        
        self.received_global_data = False
        self.received_local_data = False

    def init_parameters(self):
        self.target_position = np.array([
            rospy.get_param('~target_x', 3.4),
            rospy.get_param('~target_y', -4.0),
            rospy.get_param('~target_yaw', 0.0)
        ]).reshape(3, 1)
        
        self.c1 = rospy.get_param('~c1', 1.5)
        self.c2 = rospy.get_param('~c2', 1.5)
        self.max_velocity = rospy.get_param('~max_velocity', 0.5)
        self.scenario_size = rospy.get_param('~scenario_size', 20.0)
        
        self.obstacle_avoidance_weight = rospy.get_param('~obstacle_weight', 2.0)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        
        rospy.loginfo("Target: [%.2f, %.2f, %.2f]",
                     self.target_position[0,0], self.target_position[1,0], self.target_position[2,0])

    def init_state_variables(self):
        self.real_position = np.zeros((3, 1))
        self.real_velocity = np.zeros((3, 1))
        self.obstacles = []
        
        self.particle_position = np.random.uniform(-self.scenario_size/2,
                                                  self.scenario_size/2, (3, 1))
        self.particle_velocity = np.zeros((3, 1))
        
        self.personal_best_position = self.particle_position.copy()
        self.personal_best_fitness = -np.inf
        
        self.global_best_position = np.zeros((3, 1))
        self.global_best_fitness = -np.inf
        self.mean_best_position = np.zeros((3, 1))
        self.all_personal_fitness = np.full(5, -np.inf)
        
        self.fitness_history = [-np.inf, -np.inf]

    def setup_communication(self):
        rospy.Subscriber('/global_pso_data', Float32MultiArray, self.global_data_callback)
        rospy.Subscriber('/{}/odom'.format(self.ns), Odometry, self.odom_callback)
        rospy.Subscriber('/{}/obstacle_info'.format(self.ns), Float32MultiArray, self.obstacle_callback)
        
        self.result_pub = rospy.Publisher('/{}/pso_result'.format(self.ns),
                                        Float32MultiArray, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.ns), 
                                          Twist, queue_size=1)

    def odom_callback(self, msg):
        self.real_position[0, 0] = msg.pose.pose.position.x
        self.real_position[1, 0] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.real_position[2, 0] = math.atan2(siny_cosp, cosy_cosp)
        
        self.real_velocity[0, 0] = msg.twist.twist.linear.x
        self.real_velocity[1, 0] = msg.twist.twist.linear.y
        self.real_velocity[2, 0] = msg.twist.twist.angular.z
        
        self.received_local_data = True

    def obstacle_callback(self, msg):
        try:
            data = msg.data
            if len(data) % 3 != 0:
                rospy.logwarn("ERROR: %d", len(data))
                return
            
            num_obstacles = len(data) // 3
            self.obstacles = []
            
            for i in range(num_obstacles):
                obs = {
                    'x': data[i*3],
                    'y': data[i*3 + 1],
                    'radius': data[i*3 + 2]
                }
                self.obstacles.append(obs)
                
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def global_data_callback(self, msg):
        try:
            data = msg.data
            if len(data) != 12:
                rospy.logwarn("ERROR: %d", len(data))
                return
            
            self.all_personal_fitness = np.array(data[:5])
            self.global_best_position = np.array(data[5:8]).reshape(3, 1)
            self.global_best_fitness = data[8]
            self.mean_best_position = np.array(data[9:12]).reshape(3, 1)
            
            self.received_global_data = True
            
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def calculate_fitness(self, position):
        target_distance = np.linalg.norm(position[:2] - self.target_position[:2])
        max_distance = math.sqrt(2 * self.scenario_size**2)
        distance_fitness = max(0, 1 - target_distance / max_distance)
        
        obstacle_penalty = 0
        for obs in self.obstacles:
            obs_pos = np.array([obs['x'], obs['y']])
            dist_to_obs = np.linalg.norm(position[:2].flatten() - obs_pos)
            if dist_to_obs < obs['radius'] + self.safe_distance:
                penalty = max(0, 1 - dist_to_obs / (obs['radius'] + self.safe_distance))
                obstacle_penalty += penalty
        
        boundary_penalty = 0
        for i in range(2):  # x, y
            if abs(position[i, 0]) > self.scenario_size / 2:
                boundary_penalty += 1
        
        fitness = distance_fitness - 0.5 * obstacle_penalty - 0.3 * boundary_penalty
        
        return max(0, fitness)

    def calculate_evolution_factor(self):
        if self.fitness_history[1] == -np.inf or self.fitness_history[0] == -np.inf:
            return 1.0
        
        current = self.fitness_history[1]
        previous = self.fitness_history[0]
        
        if max(current, previous) == 0:
            return 1.0
        
        h = 1 - min(current, previous) / max(current, previous)
        return h

    def calculate_aggregation_factor(self):
        if self.global_best_fitness <= 0:
            return 1.0
        
        mean_fitness = np.mean(self.all_personal_fitness[self.all_personal_fitness > -np.inf])
        if mean_fitness <= 0:
            return 1.0
        
        s = min(self.global_best_fitness, mean_fitness) / max(self.global_best_fitness, mean_fitness)
        return s

    def update_particle(self):
        if not (self.received_global_data and self.received_local_data):
            return
        
        current_fitness = self.calculate_fitness(self.particle_position)
        
        self.fitness_history[0] = self.fitness_history[1]
        self.fitness_history[1] = current_fitness
        
        if current_fitness > self.personal_best_fitness:
            self.personal_best_position = self.particle_position.copy()
            self.personal_best_fitness = current_fitness
        
        h = self.calculate_evolution_factor()
        s = self.calculate_aggregation_factor()
        
        w = 0.9 - 0.4 * (1 - 0.5 * h + 0.9 * s)
        w = max(0.1, min(0.9, w))  # range
        
        r1, r2, r3 = np.random.rand(3)
        
        inertia_term = w * self.particle_velocity
        cognitive_term = self.c1 * r1 * (self.personal_best_position - self.particle_position)
        social_term = self.c2 * r2 * (self.global_best_position - self.particle_position)
        
        mean_term = 0.1 * r3 * (self.mean_best_position - self.particle_position)
        
        new_velocity = inertia_term + cognitive_term + social_term + mean_term
        
        max_vel = self.max_velocity
        new_velocity = np.clip(new_velocity, -max_vel, max_vel)
        
        new_position = self.particle_position + new_velocity
        
        boundary = self.scenario_size / 2
        new_position[:2] = np.clip(new_position[:2], -boundary, boundary)
        new_position[2] = np.clip(new_position[2], -math.pi, math.pi)
        
        self.particle_velocity = new_velocity
        self.particle_position = new_position
        
        rospy.logdebug("Agent=%s, 适应度=%.3f, w=%.3f, h=%.3f, s=%.3f",
                      str(self.particle_position.flatten()), current_fitness, w, h, s)

    def calculate_control_command(self):
        if not self.received_local_data:
            return Twist()
        
        position_error = self.particle_position - self.real_position
        
        kp_linear = 1.0
        kp_angular = 2.0
        
        cmd = Twist()
        
        cmd.linear.x = kp_linear * position_error[0, 0]
        cmd.linear.y = kp_linear * position_error[1, 0]
        
        cmd.angular.z = kp_angular * position_error[2, 0]
        
        max_linear = 0.5
        max_angular = 1.0
        
        cmd.linear.x = max(-max_linear, min(max_linear, cmd.linear.x))
        cmd.linear.y = max(-max_linear, min(max_linear, cmd.linear.y))
        cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))
        
        return cmd

    def publish_result(self):
        try:
            msg = Float32MultiArray()
            msg.data = [
                float(self.robot_id),
                self.personal_best_position[0, 0],
                self.personal_best_position[1, 0],
                self.personal_best_position[2, 0],
                self.personal_best_fitness,
                self.particle_position[0, 0],
                self.particle_position[1, 0],
                self.particle_position[2, 0]
            ]
            
            self.result_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def run(self):
        rospy.loginfo("RDPSO Agent start...")
        
        while not rospy.is_shutdown():
            try:
                self.update_particle()
                
                self.publish_result()
                
                cmd = self.calculate_control_command()
                self.cmd_vel_pub.publish(cmd)
                
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                rospy.loginfo("close...")
                break
            except Exception as e:
                rospy.logerr("ERROR: %s", str(e))
                self.rate.sleep()

if __name__ == '__main__':
    try:
        agent = RDPSOAgent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("failed: %s", str(e))
