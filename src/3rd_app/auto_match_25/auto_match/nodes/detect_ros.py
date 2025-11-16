#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import yolov5
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

class SparkDetect:
    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
            self.confidence = []
            self.image = None

    def __init__(self, model_path_1,model_path_2):
        '''
        初始化YOLOv5检测器
        :param model_path: YOLOv5模型文件路径
        '''
        # try:
        self.model_1 = yolov5.load(model_path_1)
        self.model_2 = yolov5.load(model_path_2)        
        # except Exception as e:
        #     rospy.logerr(f"加载模型失败:{e}, 开始下载")
        #     # 下载模型
        #     url = "https://github.com/GentsunCheng/spark_noetic/releases/latest/download/auto.pt"
        #     stat = os.system("wget " + url + " -O " + model_path)
        #     if not (os.path.exists(model_path) or stat):
        #         rospy.logerr("下载模型失败")
        #         os.remove(model_path)
        #         return
        #     rospy.loginfo("下载模型成功")
        #     self.model = yolov5.load(model_path)

    def detect(self, image):
        '''
        检测图像中的物体
        :param image: 输入图像
        :return: 结果类结构
                  result.name: 物体名称列表
                  result.x: 物体中心点x坐标列表
                  result.y: 物体中心点y坐标列表
                  result.confidence: 物体置信度列表
                  result.image: 检测后的图像
        '''
        results_1 = self.model_1(image, augment=True)
        results_2 = self.model_2(image, augment=True)
        # 存储检测结果的列表
        result = self.__results__()

        # 遍历检测结果
        try:
            for *xyxy, conf, cls in results_1.xyxy[0]:
                # 计算中心点坐标
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                # 计算大小
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                # 绘制图像
                label = f'{self.model_1.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                # 存储中心点坐标,物体名称,置信度和图像
                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(self.model_1.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

        except Exception as _:
            pass
        
        try:
            for *xyxy, conf, cls in results_2.xyxy[0]:
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                label = f'{self.model_2.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                    # 存储第二个模型的检测结果
                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(self.model_2.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))
        except Exception as _:
            pass
        
        result.image = image
        return result


class Detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24
            )
        item_file = os.path.join(rospkg.RosPack().get_path('auto_match'),'config', 'items_config.yaml')
        with open(item_file, 'r') as file:
            config_contents = yaml.safe_load(file)
            self.obj_id = config_contents['items']
        self.items = list(self.obj_id.keys())
        self.detector = SparkDetect(
            os.environ['HOME'] + "/auto.pt",
            os.environ['HOME'] + "/banana.pt"   
        )

    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        objArray.header = data.header
        try:
            results = self.detector.detect(image)
            img_bgr = results.image
            for i in range(len(results.name)):
                if (results.name[i] not in self.items) or results.confidence[i] < 0.5:
                    continue
                obj = Detection2D()
                obj.header = data.header
                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.id = int(self.obj_id[results.name[i]])
                obj_hypothesis.score = results.confidence[i]
                obj.results.append(obj_hypothesis)
                obj.bbox.size_y = int(results.size_y[i])
                obj.bbox.size_x = int(results.size_x[i])
                obj.bbox.center.x = int(results.x[i])
                obj.bbox.center.y = int(results.y[i])
                objArray.detections.append(obj)
        except:
            img_bgr = image
        self.object_pub.publish(objArray)
        img = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)
        try:
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)
        

if __name__=='__main__':
    rospy.init_node('detector_node')
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()