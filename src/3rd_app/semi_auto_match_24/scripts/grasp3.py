#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import os
import time
import random
import rospy
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *
from swiftpro.msg import *


class GraspObject():

    def __init__(self):
        '''
        ��ʼ������
        x & y ����ˮԴ�����ĵ�λ����Ϣ
        x_prev & y_prev ����ǰһ�ּ�¼�����ĵ�
        found_count ����ʶ�����ĵ�Ĵ���
        self.found_*** �����Ƿ�׼ȷ�ҵ�ˮԴ�����ĵ�
    x=
    y=
    z=120-170
        '''

     # ����ˮԴ����
        self.water_found_count = 0 ; self.found_water = False 
        self.water_x = 0 ; self.water_y = 0 ; self.water_x_prev = 0 ; self.water_y_prev = 0
        
        # ��ȡ�궨�ļ�����
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        rospy.logwarn('X axia k and b value: ' + str(self.x_kb))
        rospy.logwarn('X axia k and b value: ' + str(self.y_kb))
    
        # ������е��λ��
        self.pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        # ������е������
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # ����TWist��Ϣ���ƻ����˵���
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # ���Ļ�е��ץȡָ��
        self.sub2 = rospy.Subscriber('/grasp', String, self.grasp_cp, queue_size=1)
        # ������е�ۻָ�״ָ̬��
        self.arm_status_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
        # ��������ͷ����,����ͼ����Ϣ����תimage_cb���д���
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)



        # ������Ϣ�û�е�۵�ָ��λ��
        self.arm_position_reset()
        
        self.pos = position()
        self.pos.x = 230
        self.pos.y = 0
        self.pos.z = 20
        #self.pub1.publish(self.pos)

 # �Դ����һ֡ͼ����д�����ʹ��CV�������       
    def image_cb(self, data):
        # �� ROS image��Ϣ����ת����opencv����  
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
            # print(cv_image1.shape)
        except CvBridgeError as e:
            print('error')
        # ��RGB��ɫת����HSV��ɫ�ռ�
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)
        # ��ɫ������ɫ��ⷶΧ
        LowerBlue = np.array([95, 90, 80])
        UpperBlue = np.array([130, 255, 255])
        # ��ֵ����
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        # λ���㣬��ͼ�������Ĥ����
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)
        # ȡ��ά�����е�һά����������
        cv_image4 = cv_image3[:, :, 0]
        # ��ֵ�˲�����
        blurred = cv2.blur(cv_image4, (9, 9))
        # ����ֵ����
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        # ��ȡ�ṹԪ��
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        # ִ�и߼���̬�任
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # ��ʴ����
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        # ���Ŵ���
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4) 
        # �������
    
        result_cvC = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours=result_cvC[1]
        hier=result_cvC[1]
        cv2.drawContours(cv_image1, contours, 0, (0, 255, 0), 2)
        # ��ʾͼ��
  
        # ���ݼ���������Ϣ���ҵ����о��������ˮԴ
        # len(contours) ʶ�𵽵�ˮԴ�ĸ���
        if len(contours) > 0:
            dis = []    
            dis_min = 640          
            # enumerate()Ϊ�����Ķ����������к�
            for i, c in enumerate(contours):
                # ����ˮԴͼ��
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # ����ˮԴ���ĵ�
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                # w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                # h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2)
                distance =  math.sqrt((x_mid-320)**2+(480-y_mid)**2)
                # �ҳ����������ˮԴ
                dis.append(distance)
                if dis[i] < dis_min:    
                    dis_min = dis[i]
                    self.water_x = x_mid
                    self.water_y = y_mid
            
            # �����п��ܳ������ƶ�����ת������ʶ��ˮԴ�����
            # �趨found_count�����ж�ˮԴ�����ƶ�
            # ��found_count�����ﵽ30ʱ����ΪˮԴû���ƶ������Խ��ж�λ           
            if self.water_found_count >= 30:
                # ���ҵ�����Ĳ����趨ΪTruexs
                self.found_water = True                
            else:
                # �ж�ʶ���ˮԴ���ĵ�λ�������ƶ�
                if abs(self.water_x - self.water_x_prev) <= 4 and abs(self.water_y - self.water_y_prev) <= 4:
                    self.water_found_count = self.water_found_count + 1
                else:
                    # һ���ƶ�������
                    self.water_found_count = 0
        else:
            # û���ҵ��������ж�ˮԴ�ƶ���������
            self.water_found_count = 0

        self.water_x_prev = self.water_x
        self.water_y_prev = self.water_y
        
        cv2.circle(cv_image1, (int(self.water_x), int(self.water_y)), 5, (0, 0, 255), -1)

        cv2.imshow("contours",cv_image1)
        cv2.waitKey(1)





    def grasp_cp(self, msg):
        # ����˯��ʱ��
        rate = rospy.Rate(0.2)
        # ���ܵ�����ץȡˮԴ���ź�
        if msg.data == '0':              
            # ץȡ��⵽������    
            print("water grasp on!!!!!!")
            self.grasp() 
        elif msg.data == '1':
            height = int(msg.data)
            self.release_object()
            rate.sleep()
       
        if msg.data == 'auto_grasp':            
            # ��δ�ܻ�ȡˮԴ׼ȷ���ĵ�����ѭ��
            # self.arm_position_reset()
            while not self.found_water:
                rospy.logwarn('finding water' )
                rate.sleep()
            # ץȡ��⵽������    
            print("water grasp on!!!!!!")
            self.auto_grasp(self.water_x,self.water_y)
            self.found_water = False

        if msg.data == 'left':
            print("left")
            self.posy(0,15,0)
        if msg.data == 'right':
            print("right")
            self.posy(0,-15,0)
        if msg.data == 'up':
            print("up")
            self.posy(10,0,0)
        if msg.data == 'down':
            print("down")
            self.posy(-10,0,0)
        if msg.data == 'high':
            print("high")
            self.posy(0,0,10)
        if msg.data == 'low':
            print("low")
            self.posy(0,0,-10)
        if msg.data == 'lift':
            print("lift")
            self.posx(230,0,160)
        elif msg.data == 'reset':
            print('reset')
            self.arm_position_reset()
        elif msg.data == 'pl1':
            self.posx(220,0,-20)
        elif msg.data == 'pl2':
            self.posx(220,0,90)
        elif msg.data == 'pl3':
            self.posx(210,0,170)
        
        

    def grasp(self):        
        print("start to grasp\n")
        # �趨˯��ʱ��
        r1 = rospy.Rate(0.2)
        r2 = rospy.Rate(0.2)
    # ��ʼ��ȡ����
        self.pub2.publish(1)
        # ���ƻ�е�������˶� 
        self.posx(0,0,-55)

        
        rospy.sleep(1)

    def auto_grasp(self,y,x):        
        
  
        # ��������
        pos.x = 230  
        pos.y = 0
        pos.z = 160  
        self.pub1.publish(pos)
        rospy.sleep(1)

    # �ͷ�����
    def release_object(self):
        
        # stop pump
        self.pub2.publish(0)
        
    
    def arm_to_home(self):
        self.pos.x = 230
        self.pos.y = 0
        self.pos.z = 20
    def posy(self,x,y,z):
        self.pos.x += x
        self.pos.y += y 
        self.pos.z += z
        print(self.pos)
        self.pub1.publish(self.pos)
        rospy.sleep(0.2)
    def posx(self,x,y,z):
        if x !=0 : self.pos.x = x
        if y !=0 : self.pos.y = y 
        if z !=0 : self.pos.z = z
        print(self.pos)
        self.pub1.publish(self.pos)
        rospy.sleep(0.2)

    def arm_position_reset(self):
        print("arm reset\n")
        r1 = rospy.Rate(1)
        self.arm_status_pub.publish(0)
        r1.sleep()
        self.arm_status_pub.publish(1)
        r1.sleep()


        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

