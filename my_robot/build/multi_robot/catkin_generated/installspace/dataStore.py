#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class RDPSODataStore:
    def __init__(self):
        rospy.init_node('rdpso_datastore', anonymous=True)
        
        # 初始化数据存储
        self.num_robots = 5
        self.personal_best_positions = np.zeros((3, self.num_robots))
        self.personal_best_fitness = np.full(self.num_robots, -np.inf)
        self.current_positions = np.zeros((3, self.num_robots))
        
        # 全局最佳
        self.global_best_position = np.zeros((3, 1))
        self.global_best_fitness = -np.inf
        self.global_best_robot = -1
        
        # 平均最佳位置
        self.mean_best_position = np.zeros((3, 1))
        
        # 数据接收状态
        self.data_received = np.zeros(self.num_robots, dtype=bool)
        
        # 通信设置
        self.setup_communication()
        
        self.rate = rospy.Rate(10)  # 10Hz
        rospy.loginfo("RDPSO DataStore节点已启动")

    def setup_communication(self):
        """设置通信"""
        # 订阅所有机器人的结果
        self.robot_subs = []
        for i in range(self.num_robots):
            topic = '/robot{}/pso_result'.format(i + 1)
            sub = rospy.Subscriber(topic, Float32MultiArray, 
                                 lambda msg, idx=i: self.robot_result_callback(msg, idx))
            self.robot_subs.append(sub)
        
        # 发布全局数据
        self.global_pub = rospy.Publisher('/global_pso_data', Float32MultiArray, queue_size=1)

    def robot_result_callback(self, msg, robot_idx):
        """处理机器人结果回调
        数据格式: [robot_id(1), pBest(3), F_p(1), particle_pos(3)] (共8个float)
        """
        try:
            data = msg.data
            if len(data) != 8:
                rospy.logwarn("机器人%d数据格式错误! 期望8个值，收到%d个", 
                             robot_idx + 1, len(data))
                return
            
            # 验证robot_id
            received_id = int(data[0])
            if received_id != robot_idx + 1:
                rospy.logwarn("机器人ID不匹配: 期望%d, 收到%d", 
                             robot_idx + 1, received_id)
                return
            
            # 更新个人最佳
            self.personal_best_positions[:, robot_idx] = data[1:4]
            self.personal_best_fitness[robot_idx] = data[4]
            
            # 更新当前位置
            self.current_positions[:, robot_idx] = data[5:8]
            
            # 标记数据已接收
            self.data_received[robot_idx] = True
            
            # 更新全局最佳
            if data[4] > self.global_best_fitness:
                self.global_best_fitness = data[4]
                self.global_best_position = np.array(data[1:4]).reshape(3, 1)
                self.global_best_robot = robot_idx + 1
                
                rospy.loginfo("更新全局最佳: 机器人%d, 适应度%.3f, 位置[%.2f, %.2f, %.2f]", 
                             self.global_best_robot, self.global_best_fitness,
                             self.global_best_position[0,0], 
                             self.global_best_position[1,0],
                             self.global_best_position[2,0])
            
            # 更新平均最佳位置
            self.update_mean_best()
            
        except Exception as e:
            rospy.logerr("处理机器人%d数据时出错: %s", robot_idx + 1, str(e))

    def update_mean_best(self):
        """更新平均最佳位置"""
        # 只计算已接收数据的机器人的平均值
        valid_robots = self.data_received
        if np.any(valid_robots):
            valid_positions = self.personal_best_positions[:, valid_robots]
            self.mean_best_position = np.mean(valid_positions, axis=1, keepdims=True)

    def publish_global_data(self):
        """发布全局数据
        数据格式: [all_F_p(5), gBest(3), F_gBest(1), mBest(3)] (共12个float)
        """
        try:
            msg = Float32MultiArray()
            msg.data = []
            
            # 所有机器人的个人适应度
            msg.data.extend(self.personal_best_fitness.tolist())
            
            # 全局最佳位置
            msg.data.extend(self.global_best_position.flatten().tolist())
            
            # 全局最佳适应度
            msg.data.append(self.global_best_fitness)
            
            # 平均最佳位置
            msg.data.extend(self.mean_best_position.flatten().tolist())
            
            self.global_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr("发布全局数据时出错: %s", str(e))

    def print_status(self):
        """打印状态信息"""
        received_count = np.sum(self.data_received)
        rospy.loginfo_throttle(5, 
            "状态: 接收数据 %d/%d, 全局最佳适应度: %.3f (机器人%d)", 
            received_count, self.num_robots, 
            self.global_best_fitness, self.global_best_robot)

    def run(self):
        """主运行循环"""
        while not rospy.is_shutdown():
            try:
                # 发布全局数据
                self.publish_global_data()
                
                # 打印状态
                self.print_status()
                
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                rospy.loginfo("接收到ROS中断信号，正在关闭节点...")
                break
            except Exception as e:
                rospy.logerr("主循环错误: %s", str(e))
                self.rate.sleep()

if __name__ == '__main__':
    try:
        datastore = RDPSODataStore()
        datastore.run()
    except Exception as e:
        rospy.logerr("节点初始化失败: %s", str(e))
