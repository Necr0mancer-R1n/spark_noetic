#!/usr/bin/python3

import sys
import os
import yaml
import _thread
import threading
import pickle

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray
import tf
import tf.transformations as tf_transformations

class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者


    def set_pose(self, x, y, z):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)
        rospy.sleep(1)

    def set_pump(self, enable:bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f" 设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock:bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))

class CamAction:
    def __init__(self):
        # 获取标定文件相关信息
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))          # 获取功能包路径
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')  # 获取物体标签路径
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("can't not open file")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items file empty")
            sys.exit(1)

        # 根据yaml文件，确定抓取物品的id号
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]],
            items_content["items"][items_content["objects"]["objects_b"]],
            items_content["items"][items_content["objects"]["objects_c"]],
            items_content["items"][items_content["objects"]["objects_d"]]
        ]

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][2:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            cube_list.clear()
            return cube_list
        
        # 提取
        for obj in obj_array.detections:
            obj_id = obj.results[0].id
            if obj_id in self.search_id:  # ֻ����Ŀ��ID������
                x = obj.bbox.center.x
                y = obj.bbox.center.y
                cube_list.append([obj_id, [x, y, 0]])  # ֱ�����ӵ��б�
          
            
        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        # for key, value in obj_dist.items():
        #     if key in self.search_id:
        #         cube_list.append([key, value])

        return cube_list

class ArmAction:
    def __init__(self):

        self.cam = CamAction()

        # 获取标定文件数据/一层
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]    

        # 获取标定文件数据/二层
        filename1 = os.environ['HOME'] + "/thefile2.txt"
        with open(filename1, 'r') as f1:
            s1 = f1.read()
        arr1 = s1.split()
        self.x_kb1 = [float(arr1[0]), float(arr1[1])]
        self.y_kb1 = [float(arr1[2]), float(arr1[3])]        
    
        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)


    #抓取一层代码
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 寻找物品
        rospy.sleep(1)
        cube_list = self.cam.detector()
        rospy.loginfo(f"待抓取物体: {cube_list}") 
        
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0
        
        cube_list.sort(key=lambda item: item[1][0]+5*item[1][1], reverse=True) 
        

        rospy.loginfo(f"排序后列表: {cube_list}")

        target = cube_list[0]
        x = self.x_kb[0] * target[1][1] + self.x_kb[1]  # ͼ��x �� ��е��x
        y = self.y_kb[0] * target[1][0] + self.y_kb[1]  # ͼ��y �� ��е��y
        z = -55  # �̶�ץȡ�߶�

        print(f"找到物品了！它在: {x}, {y}, {z}")

        # 打开气泵，进行吸取


        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1.2)

        self.interface.set_pump(True)
        rospy.sleep(0.3)
        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.4)
        
        r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 120)
        # rospy.sleep(2)
        # r1.sleep()

        self.grasp_status_pub.publish(String("0"))

        return cube_list[0][0]

    #抓取二层香蕉代码
    def grasp1(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 寻找物品
        rospy.sleep(1)
        cube_list = self.cam.detector()
        rospy.loginfo(f"待抓取物体: {cube_list}") 

        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0
        
        cube_list.sort(key=lambda item: item[1][1], reverse=False)  

        rospy.loginfo(f"排序后列表: {cube_list}")

        target = cube_list[0]
        x = self.x_kb1[0] * target[1][1] + self.x_kb1[1]  # ͼ��x �� ��е��x
        y = self.y_kb1[0] * target[1][0] + self.y_kb1[1]  # ͼ��y �� ��е��y
        z = 45  # �̶�ץȡ�߶�

        print(f"找到物品了！它在: {x}, {y}, {z}")




        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1)

                # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(0.3)


        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.4)

        r2.sleep()

        # 抬起目标方块
        print(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 20)
        # rospy.sleep(2)
        r1.sleep()

        self.grasp_status_pub.publish(String("0"))



        return cube_list[0][0]
    
    # def grasp(self):
    #     '''
    #     使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
    #     @return: 抓取到物品的id, 0为未识别到需要的物品
    #     '''
    #     x = 235
    #     y = 0
    #     z = -55

    #     cube_list = self.cam.detector()
    #     rospy.loginfo(f"待抓取: {cube_list}") 

    #     if len(cube_list) == 0:
    #         rospy.logwarn("没有找到物品啊。。。去下一个地方")
    #         self.grasp_status_pub.publish(String("1"))
    #         return 0
        
    #     cube_list.sort(key=lambda item: item[1][1], reverse=True)  
    #     rospy.loginfo(f"排序后列表:{cube_list}")

    #     target = cube_list[0]
    #     x = self.x_kb[0] * target[1][1] + self.x_kb[1]  
    #     y = self.y_kb[0] * target[1][0] + self.y_kb[1]  
    #     z = -55

    #     print(f"找到物品了！它在: {x}, {y}, {z}")

    #     self.interface.set_pose(x, y, z + 20)

    #     self.interface.set_pump(True)
    #     rospy.sleep(1)

    #     def lift_object():
    #         print("我把物品抬起来了")
    #         self.interface.set_pose(x, y, z + 120)

    #     def robot_step_back():
    #         print("=======向后退一点===== ")
    #         self.robot.step_back()

    #     lift_thread = threading.Thread(target=lift_object)
    #     back_thread = threading.Thread(target=robot_step_back)

    #     lift_thread.start()
    #     back_thread.start()

    #     lift_thread.join()
    #     back_thread.join()

    #     self.grasp_status_pub.publish(String("0"))

    #     return cube_list[0][0]
    
    # def grasp1(self):
    #     '''
    #     使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
    #     @return: 抓取到物品的id, 0为未识别到需要的物品
    #     '''
    #     r1 = rospy.Rate(0.25)
    #     r2 = rospy.Rate(10)
    #     x = 235
    #     y = 0
    #     z = -55

    #     # 寻找物品
    #     cube_list = self.cam.detector()
    #     rospy.loginfo(f"待抓取物体: {cube_list}") 
    #     # rospy.sleep(2)
    #     if len(cube_list) == 0:
    #         rospy.logwarn("没有找到物品啊。。。去下一个地方")
    #         self.grasp_status_pub.publish(String("1"))
    #         return 0
        
    #     cube_list.sort(key=lambda item: item[1][1], reverse=True)  

    #     rospy.loginfo(f"排序后列表: {cube_list}")

    #     target = cube_list[0]
    #     x = self.x_kb1[0] * target[1][1] + self.x_kb1[1]  # ͼ��x �� ��е��x
    #     y = self.y_kb1[0] * target[1][0] + self.y_kb1[1]  # ͼ��y �� ��е��y
    #     z = 45  # �̶�ץȡ�߶�

    #     print(f"找到物品了！它在: {x}, {y}, {z}")

    #     # 机械臂移动到目标位置上方
    #     self.interface.set_pose(x, y, z + 20)
    #     # rospy.sleep(1)

    #     # 打开气泵，进行吸取
    #     self.interface.set_pump(True)
    #     rospy.sleep(1)

    #     def lift_object():
    #         print("我把物品抬起来了")
    #         self.interface.set_pose(x, y, z + 120)

    #     def robot_step_back():
    #         print("=======向后退一点===== ")
    #         self.robot.step_back()

    #     lift_thread = threading.Thread(target=lift_object)
    #     back_thread = threading.Thread(target=robot_step_back)

    #     lift_thread.start()
    #     back_thread.start()

    #     lift_thread.join()
    #     back_thread.join()

    #     self.grasp_status_pub.publish(String("0"))

    #     return cube_list[0][0]
    
    def drop(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = -55

        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.7)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(0.3)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True

    def drop1(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = 60

        # 默认放置位置
        cube_list = self.cam.detector()
        x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
        y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
        z = 60
        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.7)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(0.3)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True
    
    def drop2(self, check=False):

        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        r1 = rospy.Rate(0.25)
        r2 = rospy.Rate(10)
        x = 235
        y = 0
        z = 160
        
        # 默认放置位置
        cube_list = self.cam.detector()
        x = self.x_kb1[0] * cube_list[0][1][1] + self.x_kb1[1]
        y = self.y_kb1[0] * cube_list[0][1][0] + self.y_kb1[1]
        z = 160

        # 默认放置位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.7)

        # 关闭气泵
        self.interface.set_pump(0)
        # r2.sleep()
        rospy.sleep(0.3)

        self.arm_grasp_ready()  # 移动机械臂到其他地方
        # rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return True

    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(0.5)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(50, 180, 160)  
        if block:
            rospy.sleep(0.5)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20)
        if block:
            rospy.sleep(0.5)

class RobotMoveAction:
    def __init__(self):
        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))


        # 创建获取spark前后距离的service客户端
        rospy.wait_for_service('/get_distance')
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)


    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go "+name)
        rospy.loginfo(f"已发送go指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True

    #不记录方向信息   
    def arriveto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("arrive "+name)
        rospy.loginfo(f"已发送arrivex指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True
        

    
    def step_back(self,dis):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.3,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_go(self,dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.3,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True


class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        print("========实例化Cam===== ")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        print("========实例化Arm===== ")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")
        
        # 物品 ID 映射 (COCO 标准)
        self.BANANA_ID = 4      # banana
        self.TEDDY_ID  = 1      # teddy bear
        self.CLOCK_ID  = 3      # clock
        self.CUP_ID    = 2      # wine glass

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        ret = False # 是否导航成功标志
        item_type = 0 
        # self.arm.arm_home()  # 移动机械臂到其他地方

        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        self.robot.step_go(0.4)

        if self.stop_flag: return

        # ==== 移动机械臂 =====
        self.arm.arm_position_reset()  # 重置机械臂坐标系

        # robot_thread = threading.Thread(target=lambda: self.robot.step_go(0.4))
        # robot_thread.start()

        # arm_thread = threading.Thread(target=lambda: [self.arm.arm_position_reset(), self.arm.arm_grasp_ready()])
        # arm_thread.start()

        # robot_thread.join()
        # arm_thread.join()

        # ===== 导航到分类区 =====
        if self.robot.goto_local("Classification_area"):
            rospy.sleep(0.2)
            # 新增：解析分类板
            self.mapping = self.detect_classification_board()
        else:
            rospy.logerr("Navigation to Classification_area failed,please run task again ")
            self.stop_flag = True
        
        # 创建任务安排字典，设定前往的抓取地点与次数
        sorting_status_times = {
            "Sorting_W":2,
            "Sorting_S":2,
            "Sorting_E":2,
            "Sorting_N":2,
            "Sorting_M":2,
        }

        drop_times = {
            "Collection_B":0,
            "Collection_C":0,
            "Collection_D":0,
            "Collection_A":0,
        }

        sorting_name = "Sorting_W"
        # pre_sorting = "Sorting_W"
        at_sorting = False
        drop_zone = 'Collection_A'
        total_drop_times = 0
        # =======开始循环运行前往中心区域抓取与放置任务======
        while True:
            # 根据任务安排逐步执行
            print("readying to sort_areas")

            if total_drop_times == 10:
                break
            else:
                sorting_name = list(sorting_status_times.keys())[0]

            # =====导航到目标地点====
            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_E"] > 0):
                            sorting_name = "Sorting_E"
                    elif "Sorting_N" in sorting_status_times:
                        if(sorting_status_times["Sorting_N"] > 0):
                            sorting_name = "Sorting_N"
                if(drop_zone == "Collection_C"):
                    if "Sorting_S" in sorting_status_times:
                        if(sorting_status_times["Sorting_S"] > 0):
                            sorting_name = "Sorting_S"
                    elif "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_E"] > 0):
                            sorting_name = "Sorting_E"
                if(drop_zone == "Collection_D"):
                    if "Sorting_W" in sorting_status_times:
                        if(sorting_status_times["Sorting_W"] > 0):
                            sorting_name = "Sorting_W"
                    elif "Sorting_S" in sorting_status_times:
                        if(sorting_status_times["Sorting_S"] > 0):
                            sorting_name = "Sorting_S"


            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_S'):
                        ret = self.robot.arriveto_local('Step_SE')
                if(drop_zone == "Collection_C"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_SW")
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local('Step_NE')
                if(drop_zone == "Collection_D"):
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_E'):
                        ret = self.robot.arriveto_local('Step_SE')

                        
            ret = self.robot.goto_local(sorting_name) # 导航到目标点
            at_sorting =True
            rospy.sleep(1) # 停稳

            if self.stop_flag: return

            # =====识别并抓取物体====
            item_type = 0
            if ret: # 判断是否成功到达目标点
                print("========扫描中，准备抓取===== ")
                if(total_drop_times == 8):
                    item_type = self.arm.grasp1()
                else: item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型

                if self.stop_flag: return

            if item_type == 0: # 如果没有识别到物体，将该地点的抓取次数归0
                sorting_status_times[sorting_name] = 0
                continue

            
            # ====放置物品====
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")

            if total_drop_times == 8:
                drop_zone = "Collection_A"   # 香蕉固定 A
            elif item_type != self.BANANA_ID:
                # 用分类板识别的结果，找对应放置区
                drop_zone = self.mapping.get(item_type, "Collection_B") 
            
            

            rospy.loginfo(f"本次物品ID: {item_type} → 放置区: {drop_zone}")

            # === 用动态放置区导航 ===
            if(drop_zone == 'Collection_B'):
                if(sorting_name == 'Sorting_S'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_NW")
            if(drop_zone == 'Collection_C'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_SW")
            if(drop_zone == 'Collection_D'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_E'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)

            ret = self.robot.goto_local(drop_zone)
            rospy.sleep(1)  # 停稳
            if self.stop_flag: return

            if ret:
                if drop_times[drop_zone] == 0:
                    self.arm.drop()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] == 1:
                    self.arm.drop1()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] == 2:
                    self.arm.drop2()  # 放下物品
                    
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                self.arm.drop()
            if self.stop_flag: return

            # 下一步
            sorting_status_times[sorting_name] = sorting_status_times[sorting_name] - 1
            if "Sorting_E" in sorting_status_times:
                if(sorting_status_times["Sorting_E"] == 0):
                    sorting_status_times.pop("Sorting_E")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_W" in sorting_status_times:
                if(sorting_status_times["Sorting_W"] == 0):
                    sorting_status_times.pop("Sorting_W")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_N" in sorting_status_times:
                if(sorting_status_times["Sorting_N"] == 0):
                    sorting_status_times.pop("Sorting_N")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            if "Sorting_S" in sorting_status_times:
                if(sorting_status_times["Sorting_S"] == 0):
                    sorting_status_times.pop("Sorting_S")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            at_sorting = False
            total_drop_times += 1


        self.arm.arm_home()
        # act.goto_local("sp")

        rospy.logwarn("***** task finished *****")
        rospy.logwarn("if you want to run task again. ")
        rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
        rospy.logwarn("Or press Ctrl+C to exit the program")
        
    def detect_classification_board(self): 
        rospy.loginfo("开始检测分类版图案...")

        zones = ["Collection_B", "Collection_C", "Collection_D"]
        mapping = {}

        while not rospy.is_shutdown():
            try:
                # 获取分类版信息
                obj_array = rospy.wait_for_message("/objects", Detection2DArray, timeout=5)
            except Exception:
                rospy.logwarn("未检测到分类版物体，继续等待...")
                continue  # 循环

            objs = []
            for obj in obj_array.detections:
                item_id = obj.results[0].id
                if item_id == (99 or 10 or 20 or 30):  
                    continue
                item_x = obj.bbox.center.x
                objs.append((item_id, item_x))

            if len(objs) < 3:
                rospy.logwarn(f"当前识别到的物体数量: {len(objs)}继续等待...")
                continue  # 未识别到三个，继续等待

            # 排序物体从左到右
            objs.sort(key=lambda x: x[1])

            # 填充映射
            for i, (item_id, _) in enumerate(objs[:3]):
                mapping[item_id] = zones[i]

            rospy.loginfo(f"分类版检测: {mapping}")
            return mapping  # 检测到三个物体后返回

    def task_cmd_cb(self,flag):
        if flag :
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task, args=())
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("waiting for thread exit...")
                self.stop_flag = True
                self.task_run_th.join()
                rospy.logwarn("thread exit success")

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                self.arm.grasp()
            elif msg.data == "0":
                self.arm.drop()
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
