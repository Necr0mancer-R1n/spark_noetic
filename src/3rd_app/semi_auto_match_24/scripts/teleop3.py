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



# ȫ�ֱ���
cmd = Twist()
# ������е��λ��

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


grasp_status = rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=2)

def keyboardLoop():
	# ��ʼ��
	rospy.init_node('teleop')
	rate = rospy.Rate(30)
	rateslow = rospy.Rate(60)
	pid = os.getpid()
	sudoPassword = 'spark'
	command = 'renice -10 %d' % pid
	str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))   
	# �����ƶ����� ��0.4-0.5 | ��2.5-2.6
	run_vel_, run_an_ = 0.5, 2.0
	#run_vel_, run_an_ = 1.0, 2.6
	# �����ƶ�����
	low_vel_, low_an_ = 0.2,0.8
	#2key movement param
	two_vel_,two_an_=0.2,0.8
	
	vel, an_vel = low_vel_, low_an_

	global can_release, can_grasp , can_move,allowSHUAI
	can_grasp = True
	can_release = False
	can_move = True
	allowSHUAI = True

	print(pygame.init())
	screen = pygame.display.set_mode((200, 10))
	pygame.display.set_caption("keep me on top")
	

	count_z = 0
	count_x = 0
	

	# ��ȡ����ѭ��
	while not rospy.is_shutdown():
		can_move = True
		speed, turn = 0, 0
		#print(pygame.event.get())
		key_list = pygame.key.get_pressed()
		# print(key_list)
		if key_list[pygame.K_q]: # �˳�
			pygame.quit()
			exit()
			break
		vel, an_vel = low_vel_, low_an_
		msg = String()


############  ARM   ##############################################
		event = pygame.event.wait()
		if event.type == pygame.KEYDOWN	:
			if event.key==pygame.K_k:
				msg.data = 'down' 
			if event.key==pygame.K_i:
				msg.data = 'up' 
			if event.key==pygame.K_l: 
				msg.data = 'right' 
			if event.key==pygame.K_j: 
				msg.data = 'left'
			if event.key==pygame.K_u: 
				msg.data = 'high'
			if event.key==pygame.K_o: 
				msg.data = 'low'
			if event.key==pygame.K_p: 
				msg.data = 'lift'
			if event.key==pygame.K_z: 
				msg.data = 'reset'
			if event.key==pygame.K_SEMICOLON and can_grasp: # ץȡ0
				msg.data = '0'
			if event.key==pygame.K_c: 
				msg.data = '1'
			

		if event.key==pygame.K_9: 
					msg.data = 'pl3'
		elif event.key==pygame.K_8: 
					msg.data = 'pl2'
		elif event.key==pygame.K_7: 
					msg.data = 'pl1'
		elif event.key==pygame.K_1: 
					msg.data = 'auto_grasp'
	
		if msg.data:
			grasp_pub.publish(msg)

###########  BASE  #########################################################
	indouble_mode=True
	if key_list[pygame.K_d] and key_list[pygame.K_w] and key_list[pygame.K_LSHIFT]:
			turn -= 1     
			speed += 1
			vel, an_vel = two_vel_, two_an_
			indouble_mode=False
	elif key_list[pygame.K_a] and key_list[pygame.K_w] and key_list[pygame.K_LSHIFT]:
			turn += 1     
			speed += 1
			vel, an_vel = two_vel_, two_an_
			indouble_mode=False
	elif key_list[pygame.K_d] and key_list[pygame.K_s] and key_list[pygame.K_LSHIFT]:
			turn += 1     
			speed -= 1
			vel, an_vel = two_vel_, two_an_
			indouble_mode=False
	elif key_list[pygame.K_a] and key_list[pygame.K_s] and key_list[pygame.K_LSHIFT]:
				turn -= 1     
				speed -= 1
				vel, an_vel = two_vel_, two_an_
				indouble_mode=False
	else : 
			indouble_mode=True
	if indouble_mode:
		if event.type == pygame.KEYDOWN	:
			if event.key==pygame.K_w:
						speed = 1
			if event.key==pygame.K_s:
						speed = -1 
			if event.key==pygame.K_a:
						turn = 1
			if event.key==pygame.K_d:
						turn = -1
			#if event.key==pygame.K_LSHIFT:
			#	vel, an_vel = run_vel_, run_an_
		if key_list[pygame.K_w]:
			speed += 1
					#vel = low_vel_
			if key_list[pygame.K_s]:
					speed -= 1
					#vel = low_vel_
			if key_list[pygame.K_a]:
					turn += 1
					#an_vel = low_an_
			if key_list[pygame.K_d]:
					turn -= 1
					#an_vel = low_an_
			if key_list[pygame.K_LSHIFT]:
					vel, an_vel = run_vel_, run_an_
		else :
			vel, an_vel = low_vel_, low_an_
	
	if event.type == pygame.KEYUP	:
		if event.key==pygame.K_w:
					speed = 0 
		if event.key==pygame.K_s:
					speed = 0 
		if event.key==pygame.K_a:
					turn = 0
		if event.key==pygame.K_d:
					turn = 0
		#if event.key==pygame.K_LSHIFT:
			#	vel, an_vel = low_vel_, low_an_
	

		


##########   ������Ϣ   #########################################################33
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

