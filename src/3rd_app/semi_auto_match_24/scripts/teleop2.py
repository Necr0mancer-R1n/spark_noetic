#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


#通信信息
arm_reset_pub = rospy.Publisher('arm_reset_topic', String, queue_size=1)

def grasp_pub(data):
    msg=String()
    msg.data = data
    grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)
    grasp_pub.publish(msg)

def cmd_pub(speed,turn,vel,an_vel):
    cmd = Twist()
    cmd.linear.x = speed * vel
    cmd.angular.z = turn * an_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub.publish(cmd)


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
    #rateslow = rospy.Rate(60)
    pid = os.getpid()
    sudoPassword = 'spark'
    command = 'renice -10 %d' % pid
    str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
 
    # 高速移动参数 线0.4-0.5 | 角2.5-2.6
    run_vel_, run_an_ = 0.5, 2.0
    # 低速移动参数
    low_vel_, low_an_ = 0.2, 0.8
    # 2key movement param
    two_vel_, two_an_ = 0.2, 0.8

    vel, an_vel = low_vel_, low_an_
    speed,turn = 0,0

    global can_release, can_grasp, can_move
    can_grasp = True
    can_release = False
    counter_clock=0
    
    #pygame init
    print(pygame.init())
    pygame.display.set_mode((200, 10))
    pygame.display.set_caption("keep me on top")
    event = pygame.event.wait()
    key_list = pygame.key.get_pressed()

########## 读取按键循环 ###############################################
    while not rospy.is_shutdown():

        #测试循环是否正常运行
        counter_clock += 1
        #print(counter_clock)
        if counter_clock%100== 0 :
            print(time.ctime())

        # 退出
        if event.type == pygame.KEYDOWN and event.key == pygame.K_q:  
            print('q')
            pygame.quit()
            exit()
            break

        event = pygame.event.wait()
############  ARM   ##############################################
        if 1 :
            if event.type == pygame.KEYDOWN:

                #调试指令集
                if 1 :
                    if event.key == pygame.K_k and 1:
                        grasp_pub('down')
                    elif event.key == pygame.K_i and 1:
                        grasp_pub('up')
                    elif event.key == pygame.K_l and 1:
                        grasp_pub('right')
                    elif event.key == pygame.K_j and 1:
                        grasp_pub('left')
                    elif event.key == pygame.K_u and 1:
                        grasp_pub('high')
                    elif event.key == pygame.K_p and 1:
                        grasp_pub('low')
                    elif event.key == pygame.K_z and 1:
                        grasp_pub('reset')
                    elif event.key == pygame.K_c and 1:
                        grasp_pub('pump_close')
                    elif event.key == pygame.K_x and 1:
                        grasp_pub('pump_on')

                #动作指令集
                if 1 :
                    if event.key == pygame.K_SEMICOLON and can_grasp:  # 抓取0
                        grasp_pub('0')
                    elif event.key == pygame.K_u:
                        grasp_pub('lift')
                    elif event.key == pygame.K_9:
                        grasp_pub('pl3')
                    elif event.key == pygame.K_8:
                        grasp_pub( 'pl2')
                    elif event.key == pygame.K_7:
                        grasp_pub('pl1')
                    elif event.key == pygame.K_1:
                        grasp_pub('auto_grasp')

                

###########  BASE  #########################################################
       
        if 1:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    print('w')
                    speed = 1
                if event.key == pygame.K_s:
                    print('s')
                    speed = -1
                if event.key == pygame.K_a:
                    print('a')
                    turn = 1
                if event.key == pygame.K_d:
                    print('d')
                    turn = -1
                if event.key==pygame.K_LSHIFT:
                    print('speed up')
                    vel, an_vel = run_vel_, run_an_
            if key_list[pygame.K_w]:
                print('w--')
                speed = 1
            if key_list[pygame.K_s]:
                print('s--')
                speed = -1     
            if key_list[pygame.K_a]:
                print('a--')
                turn = 1    
            if key_list[pygame.K_d]:
                print('d--')
                turn = -1

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w or pygame.K_s:
                    print('stop +')
                    speed = 0
                if event.key == pygame.K_a or pygame.K_d:
                    print('stop -') 
                    turn = 0
                if event.key==pygame.K_LSHIFT:
                    print('speed down')
                    vel, an_vel = low_vel_, low_an_

        if vel == run_vel_ and speed != 0 and turn != 0:
            vel,an_vel = two_vel_,two_an_
        cmd_pub(speed,turn,vel,an_vel)

  
        pygame.display.update()
        pygame.event.pump()
        rate.sleep()


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
