#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from spark_carry_object.msg import position, status
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
import numpy as np
import math
from threading import Thread
import os

current_process = "" # 'adjustment', 'collection', 'calibration'
window_name = "Calibration"
bridge = CvBridge()
cali_w = 20
cali_h = 30
c_cnt = 0
collect_times = 200
HSV_value = [0,0,0]
lower_HSV = None
upper_HSV = None

# following two functions defined inside main
pixel2point = None
move_arm = None
reset_arm = None

point_from_cam = [0,0,0]

SAMPLE_POINTS = [
	[250, 150, -50],
	[320, 0, -50],
	[200, 150, -50],
	[250, -180, 0],
	[150, 100, 0],
	[220, 0, 0],
	[250, 180, 0],
	[100, 100, 50],
	[150, 120, 70],
	[280, 120, 70],
	[100, 100, 100],
	[150, -100, 100],
	[150, 0, 100],
	[150, -80, 100],
	[200, 100, 100],
	[250, 0, 100],
	[250, -100, 100],
	[150, 0, 150],
	[150, 100, 150],
]
MARK2EOAT_DISTANCE = 85 #for spark#3
# MARK2EOAT_DISTANCE = 77 #for spark#4

def mean_hsv(img,hsv_value):
	HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	hsv_value[0]+=np.mean(HSV[:, :, 0])
	hsv_value[1]+=np.mean(HSV[:, :, 1])
	hsv_value[2]+=np.mean(HSV[:, :, 2])
	return hsv_value

def hsv_range(hsv_value):
	H_range = 6
	S_range = 120
	V_range = 120

	lower_H = int(hsv_value[0] - H_range)
	upper_H = int(hsv_value[0] + H_range)

	lower_S = int(hsv_value[1] - S_range)
	upper_S = int(hsv_value[1] + S_range)

	lower_V = int(hsv_value[2] - V_range)
	upper_V = int(hsv_value[2] + V_range)

	if lower_H<0:
		lower_H=0
	if upper_H>180:
		upper_H=180

	if lower_S<50:
		lower_S=50
	if upper_S>255:
		upper_S=255

	if lower_V<50:
		lower_V=50
	if upper_V>255:
		upper_V=255

	lower_HSV = np.array([lower_H, lower_S, lower_V])
	upper_HSV = np.array([upper_H, upper_S, upper_V])
	return lower_HSV, upper_HSV

def start_process():
	global current_process
	reset_arm()
	move_arm(120,0,35)
	rospy.sleep(1)
	current_process = "adjustment"

def calibrate_fn():
	arm_points = []
	cam_points = []
	rospy.sleep(1)
	for xa, ya, za in SAMPLE_POINTS:
		move_arm(xa, ya, za)
		rospy.sleep(3)
		arm_points.append([xa, ya, za + MARK2EOAT_DISTANCE, 1])
		a = point_from_cam
		cam_points.append([a[0], a[1], a[2], 1])
		print('Cam Point:', a)
	arm_mat = np.matrix(arm_points)
	cam_mat = np.matrix(cam_points)
	cam_mat_pinv = np.linalg.pinv(cam_mat)
	transform_mat = np.dot(cam_mat_pinv, arm_mat)
	transform_mat = np.multiply(transform_mat, abs(transform_mat) > 1e-4)
	filename = os.environ['HOME'] + "/transform_mat.dat"
	transform_mat.dump(filename)
	print("Camera Calibration Successfull!!")
	print('arm_mat')
	print(arm_mat)
	print('cam_mat')
	print(cam_mat)
	print('transform_mat')
	print(transform_mat)
	move_arm(130, 0, 35)
	rospy.sleep(1)

def process_adjustment(cv_image_draw):
	global current_process
	if current_process != 'adjustment': return
	cv2.putText(cv_image_draw, 'please move the camera to make ', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
			(0, 255, 0), 2, cv2.LINE_AA)
	cv2.putText(cv_image_draw, 'the red color in the rectangle ', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
			(0, 255, 0), 2, cv2.LINE_AA)
	cv2.putText(cv_image_draw, 'green box!then press the \'ENTER\' ', (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
			(0, 255, 0), 2, cv2.LINE_AA)
	cv2.rectangle(cv_image_draw, (355, 310), (355 + cali_w, 310 + cali_h), (0, 255, 0), 3)
	cv2.imshow(window_name, cv_image_draw)
	k = cv2.waitKey(1) & 0xFF
	if k == ord('\n') or k == ord('\r'):
		current_process = 'collection'
	return

def process_collection(cv_image_draw):
	global c_cnt
	global HSV_value
	global current_process
	global lower_HSV, upper_HSV
	if current_process != 'collection': return
	c_cnt = c_cnt+1
	frame = cv_image_draw[315:310 + cali_h-5, 360:355 + cali_w-5]
	cv2.putText(cv_image_draw, 'collecting the hsv!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
		(0, 255, 0), 2, cv2.LINE_AA)
	cv2.putText(cv_image_draw, ' please wait for 5s.', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
			(0, 255, 0), 2, cv2.LINE_AA)
	cv2.rectangle(cv_image_draw, (355, 310), (355 + cali_w, 310 + cali_h), (0, 255, 0), 3)
	HSV_value = mean_hsv(frame, HSV_value)	
	cv2.imshow(window_name, cv_image_draw)
	cv2.waitKey(1)
	
	if(c_cnt >= collect_times):
		for i in range(len(HSV_value)):
			HSV_value[i] = HSV_value[i] / collect_times
			print(i,len(HSV_value), HSV_value[i])
		lower_HSV, upper_HSV = hsv_range(HSV_value)
		#save_hsv(name, lower_HSV, upper_HSV)
		print("HSV 'lower' & 'upper':", lower_HSV , upper_HSV)
		if current_process != 'calibration':
			Thread(target=calibrate_fn).start()
		current_process = 'calibration'
	return

def process_calibration(cv_image_cp, depth_image):
	if current_process != 'calibration': return

	global point_from_cam
	cv_image_hsv = cv2.cvtColor(cv_image_cp, cv2.COLOR_BGR2HSV)
	cv_image_gray = cv2.inRange(cv_image_hsv, lower_HSV, upper_HSV)
	# smooth and clean noise
	cv_image_gray = cv2.erode(cv_image_gray, None, iterations=2)
	cv_image_gray = cv2.dilate(cv_image_gray, None, iterations=2)
	cv_image_gray = cv2.GaussianBlur(cv_image_gray, (5,5), 0)
	# detect contour
	contours, hier = cv2.findContours(cv_image_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	size_max = 0
	if len(contours) == 0: return
	for c in contours:
		rect = cv2.minAreaRect(c)
		box = cv2.boxPoints(rect)
		box = np.intp(box)
		x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
		y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
		w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
		h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
		size = w * h
		if size > size_max:
			size_max = size
			xc = x_mid
			yc = y_mid
	xc = math.floor(xc)
	yc = math.floor(yc)
	depth = depth_image[yc, xc]

	cv2.imshow(window_name, cv_image_cp)
	cv2.imshow("Gray Masked", cv_image_gray)
	cv2.waitKey(1)
	if depth == 0:
		return
	elif abs(depth - point_from_cam[2]) < 5:
		depth = point_from_cam[2]*0.8 + depth*0.2

	point_from_cam = pixel2point(depth, [xc, yc])
	
def image_callback(data_color, data_depth):
	try:
		color_image = bridge.imgmsg_to_cv2(data_color, "bgr8")
		depth_image = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
	except CvBridgeError as e:
		print(e)
	except ValueError as e:
		return

	if current_process == 'adjustment':
		process_adjustment(color_image.copy())
	elif current_process == 'collection':
		process_collection(color_image.copy())
	elif current_process == 'calibration':
		process_calibration(color_image.copy(), depth_image.copy())

def main():
	rospy.init_node('cali_arm_cam', anonymous=True)
	global move_arm
	global reset_arm
	global pixel2point

	pub1 = rospy.Publisher('position_write_topic', position, queue_size = 1)
	pub2 = rospy.Publisher('swiftpro_status_topic', status, queue_size = 1)
	color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
	depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
	ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
	ts.registerCallback(image_callback)
	rospy.sleep(1)
	def move_arm(x,y,z):
		pos = position()
		pos.x = x
		pos.y = y
		pos.z = z
		pub1.publish(pos)
		rospy.sleep(1)
		print('Arm pos: ( %3d, %3d, %3d )' % (x,y,z))
	
	def reset_arm():
		r1 = rospy.Rate(10)
		pub2.publish(status(1))
		r1.sleep()
		pub2.publish(status(1))
		r1.sleep()

	camera_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, 5)
	try:
		intrinsics = rs2.intrinsics()
		intrinsics.width = camera_info.width
		intrinsics.height = camera_info.height
		intrinsics.ppx = camera_info.K[2]
		intrinsics.ppy = camera_info.K[5]
		intrinsics.fx = camera_info.K[0]
		intrinsics.fy = camera_info.K[4]
		if camera_info.distortion_model == 'plumb_bob':
			intrinsics.model = rs2.distortion.brown_conrady
		elif camera_info.distortion_model == 'equidistant':
			intrinsics.model = rs2.distortion.kannala_brandt4
		intrinsics.coeffs = [i for i in camera_info.D]
	except CvBridgeError as e:
		print(e)
		return
	def pixel2point(depth, pix):
		result = rs2.rs2_deproject_pixel_to_point(intrinsics, pix, depth)
		return result

	start_process()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()