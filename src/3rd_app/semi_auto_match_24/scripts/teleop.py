#!/usr/bin/env python
# -*- coding: utf-8 -*-
# written by 陈松斌 in 10.6
# rewrite by shen yining in 7.10

import os
import sys
import tty
import termios
import pygame
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

# 全局变量
cmd = Twist()
# 发布机械臂位姿

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=2)
arm_reset_pub = rospy.Publisher('arm_reset_topic', String, queue_size=1)

global can_grasp
global can_release


def grasp_status_cp(msg):
    global can_release, can_grasp
    if msg.data == '1':
        can_release = True
    if msg.data == '0' or msg.data == '-1':
        can_grasp = True


grasp_status = rospy.Subscriber(
    '/grasp_status', String, grasp_status_cp, queue_size=2)


def keyboardLoop():
    # 初始化
    rospy.init_node('teleop')
    rate = rospy.Rate(30)
    rateslow = rospy.Rate(60)
    pid = os.getpid()
    sudoPassword = 'spark'
    command = 'renice -10 %d' % pid
    str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    # 高速移动参数 线0.4-0.5 | 角2.5-2.6
    run_vel_, run_an_ = 1.0, 1.0
    # 低速移动参数
    low_vel_, low_an_ = 0.4, 1.0

    vel, an_vel = run_vel_, run_an_

    global can_release, can_grasp, can_move, allowSHUAI
    can_grasp = True
    can_release = False
    can_move = True
    allowSHUAI = True

    print(pygame.init())
    screen = pygame.display.set_mode((200, 10))
    pygame.display.set_caption("keep me on top")

    count_z = 0
    count_x = 0

    # 读取按键循环
    while not rospy.is_shutdown():
        can_move = True
        speed, turn = 0, 0
        # print(pygame.event.get())
        key_list = pygame.key.get_pressed()
        # print(key_list)
        # if key_list[pygame.K_c]: # 退出
        #     pygame.quit()
        #     exit()
        #     break
        vel, an_vel = run_vel_, run_an_
        msg = String()

############  ARM   ##############################################
        if key_list[pygame.K_SPACE]:  # 抓取0
            msg.data = 'Z'
        #    msg.data = 'G0'
       #    can_grasp = False
        elif key_list[pygame.K_j]:  # 抓取快速(j
            msg.data = 'H0'
            can_grasp = False
        # elif key_list[pygame.K_b] and can_grasp: # 抓取1
        #     msg.data = 'G1'
        #     can_grasp = False
        # elif key_list[pygame.K_g] and can_grasp: # 抓取2
        #     msg.data = 'G2'
        #     can_grasp = False
        elif key_list[pygame.K_k]:  # 直接放下(1
            msg.data = 'R0'
            # can_release = False
        # elif key_list[pygame.K_2] and can_release: # 放下并回位
        #    msg.data = 'R1'
        #    can_release = False
        elif key_list[pygame.K_2]:  # 机械臂置于2层
            msg.data = 'PLA22'
        elif key_list[pygame.K_3]:  # 机械臂置于3层
            msg.data = 'PLA33'
        # elif key_list[pygame.K_i]: # 放3层位置(3
        elif key_list[pygame.K_i]:  # 放3层位置(3
            msg.data = 'PLA3'
        # elif key_list[pygame.K_4]: # 放2层位置
        elif key_list[pygame.K_4]:  # 放2层位置
            msg.data = 'PLA2'
        # elif key_list[pygame.K_1]: # 放1层位置
        elif key_list[pygame.K_1]:  # 放1层位置
            msg.data = 'PLA1'
        # elif key_list[pygame.K_SPACE]: # 回零(q
        #    msg.data = 'Z'
        elif key_list[pygame.K_r]:  # 机械臂重置
            msg.data = 'T'
            # msg.data = '0'
            # arm_reset_pub.publish(msg)
        elif key_list[pygame.K_u]:  # 固定位置抓取  and can_grasp(j
            msg.data = 'A'
            # can_grasp = False
            # can_release = True
        elif key_list[pygame.K_9]:  # 固定位置抓取  and can_grasp(j
            msg.data = 'S'
            # can_grasp = False
            # can_release = True

        # elif key_list[pygame.K_o]: # 右绝杀  and can_grasp(k
        #    msg.data = 'AL'
        # elif key_list[pygame.K_5] and can_release: # 放侧位（无用）
        #    msg.data = 'B'

        elif key_list[pygame.K_n]:  # 降低高度
            msg.data = 'N'
        elif key_list[pygame.K_m]:  # UP
            msg.data = 'M'
        elif key_list[pygame.K_e]:  # 向前
            msg.data = 'F'
        elif key_list[pygame.K_q]:  # 向后
            msg.data = 'B'
        elif key_list[pygame.K_p]:  # pump
            msg.data = 'P'

        if msg.data:
            grasp_pub.publish(msg)

###########  BASE  #########################################################
        if key_list[pygame.K_w]:
            speed += 1
            vel = 0.6
        if key_list[pygame.K_s]:
            speed -= 1
            vel = 0.6
        if key_list[pygame.K_a]:
            turn += 1
            vel = run_vel_
        if key_list[pygame.K_d]:
            turn -= 1
            vel = run_vel_
        if key_list[pygame.K_LSHIFT]:
            vel, an_vel = low_vel_, low_an_
        # if key_list[pygame.K_e]:
            # vel = 0.6

        # 发送消息
        cmd.linear.x = speed * vel
        cmd.angular.z = turn * an_vel
        if key_list[pygame.K_LCTRL]:
            can_move = False
        #     if key_list[pygame.K_d]:
        #         allowSHUAI = False
        #         msg.data = 'SHUAI'
        #         grasp_pub.publish(msg)
        #         rate.sleep()
        #         time.sleep(0.2)
        #         allowSHUAI = True

        if can_move:
            pub.publish(cmd)
        pygame.display.update()
        pygame.event.pump()
        rate.sleep()


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
