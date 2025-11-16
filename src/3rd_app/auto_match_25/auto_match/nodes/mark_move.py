#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import tf
import tf.transformations as tf_transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from math import radians, pi
import math
import numpy as np
import yaml
import rospkg
import os
import pickle
import matplotlib.colors as mcolors
from actionlib import SimpleActionClient

cmd = Twist()

def get_rgba(color_name,alpha=1.0):
    try:
        rgba = mcolors.to_rgba(color_name,alpha=alpha)
        return rgba
    except ValueError as e:
        print(e)


class MarkNav():
    def __init__(self):
        # 初始化节点
        rospy.init_node('MarkNav')
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(20))
        self.learning_count = 0
        print('ready')
        # 订阅标记事件
        rospy.Subscriber('mark_nav', String, self.mark_nav)
        # 定义标记地点字典
        global dict_mark
        dict_mark = {}
        # 监听TF坐标
        self.listener = tf.TransformListener()
        rospy.sleep(1) # need to delay
    
    def mark_nav(self,mark_name):
        '''
        设定标定地点，与后面导航相关，采用字典的方式存放
        建议标记地点名字,分类区、分拣区、收取区
        '''

        rospack = rospkg.RosPack() 
        mark_map_path = os.path.join(rospack.get_path('auto_match'),'config', 'mark_map.pkl')  # 标记地图yaml文件地址

        # 转化成字符型变量
        mark_name = str(mark_name)
        # 结束地点标记
        if str.find(mark_name,str("finish"))!= -1:
            # 将字典信息写入 pkl 文件
            with open(mark_map_path, 'wb') as file:
                pickle.dump(dict_mark, file)
            print("已保存pkl文件,文件保存在:",mark_map_path)
            print("当前字典内容：\n",dict_mark)


        # # 前往地点    
        # if str.find(mark_name,str("go"))!= -1:
        #     # 从文件中加载数据
        #     with open(mark_map_path, 'rb') as file:
        #         self.mark_map = pickle.load(file)
        #     # 输出加载的数据
        #     print("mark_dict=",self.mark_map)
        #     # 提取mark名称
        #     mark_name = (mark_name.split())[2]
        #     # 判断地点位置是否在地点字典中
        #     if mark_name in self.mark_map:
        #         self.navigation(mark_name)
        #     else:
        #         print("此地点尚未登陆在字典中")
         
        # if str.find(mark_name,str("arrive"))!= -1:
        #     # 从文件中加载数据
        #     with open(mark_map_path, 'rb') as file:
        #         self.mark_map = pickle.load(file)
        #     # 输出加载的数据
        #     print("mark_dict=",self.mark_map)
        #     # 提取mark名称
        #     mark_name = (mark_name.split())[2]
        #     # 判断地点位置是否在地点字典中
        #     if mark_name in self.mark_map:
        #         self.navigation1(mark_name)
        #     else:
        #         print("此地点尚未登陆在字典中")

        if str.find(mark_name, str("go")) != -1:
            # Print message indicating the start of the `go` operation
            print(f"Starting 'go' operation for target location: {mark_name}")
            
            # Load calibration data
            try:
                with open(mark_map_path, 'rb') as file:
                    self.mark_map = pickle.load(file)
                print("Successfully loaded calibration data")
            except Exception as e:
                print(f"Failed to load calibration data: {e}")
                return  # Return if data loading fails
            
            # Extract mark name
            mark_name = (mark_name.split())[2]
            
            # Check if the mark exists in the loaded calibration data
            if mark_name in self.mark_map:
                print(f"Location {mark_name} found, proceeding to navigate")
                self.navigation(mark_name)  # Navigate to the target location with orientation
            else:
                print(f"Location {mark_name} not found in calibration data, please check")
                return  # Stop if the location doesn't exist in the calibration data

        elif str.find(mark_name, str("arrive")) != -1:
            # Print message indicating the start of the `arrive` operation
            print(f"Starting 'arrive' operation for target location: {mark_name}")
            
            # Load calibration data
            try:
                with open(mark_map_path, 'rb') as file:
                    self.mark_map = pickle.load(file)
                print("Successfully loaded calibration data")
            except Exception as e:
                print(f"Failed to load calibration data: {e}")
                return  # Return if data loading fails
            
            # Extract mark name
            mark_name = (mark_name.split())[2]
            
            # Check if the mark exists in the loaded calibration data
            if mark_name in self.mark_map:
                print(f"Location {mark_name} found, proceeding to arrive at the location")
                self.navigation1(mark_name)  # Only navigate to the target location without considering orientation
            else:
                print(f"Location {mark_name} not found in calibration data, please check")
                return  # Stop if the location doesn't exist in the calibration data
    
        self.learning_count += 1
        
        if str.find(mark_name, str("learn")) != -1:
            mark_name = (mark_name.split())[2]
            
            self.currrent_position = self.get_currect_pose()

            if self.learning_count in [2, 5, 8, 11]:
                dict_mark[mark_name] = self.currrent_position[:3]
                print(f"第 {self.learning_count} 次学习，保存位置: {dict_mark}")
            else:
                dict_mark[mark_name] = self.currrent_position  
                print(f"第 {self.learning_count} 次学习，保存位姿: {dict_mark}")
        else:
            print("无法识别")

    # 获取当前位置
    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        print("target_trans:",target_trans)
        print("target_rot:",target_rot)
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))

    def navigation(self,mark_name):
        '''
        根据地点进行导航
        '''
        print("start navigation")
        # movebase初始化
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # 设定目标地点
        self.drop_position = self.mark_map[mark_name]
        tran = tf_transformations.translation_from_matrix(self.drop_position)
        quat = tf_transformations.quaternion_from_matrix(self.drop_position)
        pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        goal.target_pose.pose = pose
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)


    # def navigation1(self,mark_name):
    #     '''
    #     根据地点进行导航
    #     '''
    #     print("start navigation")
    #     # movebase初始化
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     # 设定目标地点
    #     self.drop_position = self.mark_map[mark_name]
    #     tran = tf_transformations.translation_from_matrix(self.drop_position)
    #     pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(0,0,0,1))
    #     goal.target_pose.pose = pose
    #     # 把目标位置发送给MoveBaseAction的服务器
    #     self.move_base.send_goal(goal)

    def navigation1(self, mark_name):  
        '''
        Navigate to a location based on the mark name, with no need for orientation adjustment.
        '''
        print("Start navigation to the target location")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Get the target location's position from the map
        self.drop_position = self.mark_map[mark_name]
        tran = tf_transformations.translation_from_matrix(self.drop_position)

        # Set the goal position (do not worry about orientation)
        pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(0, 0, 0, 1))  # No rotation
        goal.target_pose.pose = pose

        # Send the goal position to MoveBase for navigation
        self.move_base.send_goal(goal)

        # Wait for the result and get the robot's current position in real-time
        rate = rospy.Rate(10)  # Set the frequency to check the robot's position
        while not rospy.is_shutdown():
            try:
                # Get the current robot's position
                (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                current_position = Point(trans[0], trans[1], trans[2])
                rospy.loginfo(f"Current position: {current_position}")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get robot pose")
                continue

            # Calculate the Euclidean distance (sum of errors) between the target position and the current position
            error_x = abs(current_position.x - tran[0])
            error_y = abs(current_position.y - tran[1])
            error_z = abs(current_position.z - tran[2])

            # Calculate total error distance (Euclidean distance)
            total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)

            rospy.loginfo(f"Total position error: {total_error}")

            # Set an error threshold, assume 0.1 meters as an acceptable error range
            if total_error < 0.08:
                rospy.loginfo("Arrived at the goal, stopping the movement.")
                
                # Cancel the goal if the robot has arrived at the target
                self.move_base.cancel_goal()  # Cancels the active goal
                break  # Stop moving when the error is within the threshold

            rate.sleep()

        # Get the navigation result
        try:
            ret_status = rospy.wait_for_message('move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("Navigation timeout!!!")
            ret_status = GoalStatus.ABORTED

        if ret_status != GoalStatus.SUCCEEDED:
            rospy.logwarn("Goal not reached successfully.")
            return False
        else:
            rospy.loginfo("Goal successfully reached, proceeding to the next step.")
            return True
        
    # def navigation1(self, mark_name): 
    #     '''
    #     根据地点进行导航，不管方向
    #     '''
    #     print("start navigation to arrive at the location")
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()
        
    #     self.drop_position = self.mark_map[mark_name]
    #     tran = tf_transformations.translation_from_matrix(self.drop_position)

    #     pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(0, 0, 0, 1))
    #     goal.target_pose.pose = pose

    #     self.move_base.send_goal(goal)

    # def navigation1(self,mark_name):
    #     '''
    #     根据地点进行导航
    #     '''
    #     print("start navigation")
    #     # movebase初始化
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     # 设定目标地点
    #     self.drop_position = self.mark_map[mark_name]
    #     tran = tf_transformations.translation_from_matrix(self.drop_position)
    #     pose = Pose(Point(tran[0], tran[1], tran[2]))
    #     goal.target_pose.pose = pose
    #     # 把目标位置发送给MoveBaseAction的服务器
    #     self.move_base.send_goal(goal)

    # def navigation1(self, mark_name):
    #     '''
    #     根据地点进行导航
    #     '''
    #     print("start navigation")
    #     # movebase初始化
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()

    #     # 设定目标地点
    #     self.drop_position = self.mark_map[mark_name]
    #     tran = tf_transformations.translation_from_matrix(self.drop_position)

    #     # 获取当前机器人的方向（四元数）
    #     listener = tf.TransformListener()
    #     try:
    #         listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
    #         (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
    #         current_orientation = rot # 获取当前机器人方向的四元数
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         rospy.logerr("Failed to get current robot orientation!")
    #         current_orientation = (0, 0, 0, 1) # 如果获取失败，使用默认四元数

    #     #    创建目标位姿，保持当前方向
    #     pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(*current_orientation))
    #     goal.target_pose.pose = pose

    #     # 把目标位置发送给MoveBaseAction的服务器
    #     self.move_base.send_goal(goal)

    def create_marker(self,name,color,pose=False):
        self.mark_id += 1
        id = self.mark_id
       
        try:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'mark_nav'
            marker.id = id
            marker.type = Marker.SPHERE

            marker.scale.x=0.05
            marker.scale.y=0.05
            marker.scale.z=0.01

            r,g,b,a = get_rgba(color)
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            
            marker.action = Marker.ADD

            if pose:
                 marker.pose = pose
                 marker.pose.position.z = 0
            else:
                (trans, rot) = self.listener.lookupTransform(
            "map", "base_footprint", rospy.Time(0))
                marker.pose.position.x = trans[0]
                marker.pose.position.y = trans[1]
                marker.pose.position.z = 0
                marker.pose.orientation.x = rot[0]
                marker.pose.orientation.y = rot[1]
                marker.pose.orientation.z = rot[2]
                marker.pose.orientation.w = rot[3]


           

            marker.lifetime = rospy.Duration(0)

            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = 'mark_nav_text'
            text_marker.id = id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            if pose:
                text_marker.pose = pose
                text_marker.pose.position.z += 0.5
            else:
                (trans, rot) = self.listener.lookupTransform(
            "map", "base_footprint", rospy.Time(0))
                text_marker.pose.position.x = trans[0]
                text_marker.pose.position.y = trans[1]
                text_marker.pose.position.z = 0.5
                text_marker.pose.orientation.x = rot[0]
                text_marker.pose.orientation.y = rot[1]
                text_marker.pose.orientation.z = rot[2]
                text_marker.pose.orientation.w = rot[3]
            text_marker.scale.z = 0.1

            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            text_marker.text = name

            text_marker.lifetime = rospy.Duration(0)

            self.marker_pub.publish(marker)
            self.marker_pub.publish(text_marker)
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e:
            print(e)
        print('marked')    

if __name__ == '__main__':
    try:
        MarkNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mark_move finished.")
