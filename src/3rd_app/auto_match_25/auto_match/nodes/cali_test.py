#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from spark_carry_object.msg import position, status
from threading import Thread
from cv_bridge import CvBridgeError, CvBridge
import cv2
from auto_match.utils.coordinate_map import CoordinateMapper

bridge = CvBridge()
mapper = None
color_image = None
depth_image = None
move_arm, reset_arm = None, None

def image_callback(data_color, data_depth):
	global color_image, depth_image
	try:
		color_image = bridge.imgmsg_to_cv2(data_color, "bgr8")
		depth_image = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
	except CvBridgeError as e:
		print(e)
	except ValueError as e:
		return
	
def mouse_callback(event, x, y, flags, params):
	#right-click event value is 2
	if event == 1:
		pos = mapper.pixel2arm(depth_image, [x, y])
		print('Pix: ', [x,y])
		print('Cam', mapper.pixel2cam(depth_image, [x, y]))
		print('Arm: ', pos)
		move_arm(pos[0], pos[1], pos[2]+50)
		rospy.sleep(1)
		move_arm(*pos)
	elif event == 2:
		move_arm(50, 180, 160)
		rospy.sleep(1)
		reset_arm()

def image_thread():
	cv2.namedWindow('image', cv2.WINDOW_NORMAL)
	cv2.setMouseCallback('image', mouse_callback)
	print("Image thread running")

	while not rospy.is_shutdown():
		cv2.imshow('image', color_image.copy())
		cv2.waitKey(2)
	print("Closing image window")
	cv2.destroyAllWindows()

def main():
	rospy.init_node('test_node')
	global mapper
	global move_arm, reset_arm
	pub1 = rospy.Publisher('position_write_topic', position, queue_size = 1)
	pub2 = rospy.Publisher('swiftpro_status_topic', status, queue_size = 1)
	color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
	depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
	ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
	ts.registerCallback(image_callback)
	rospy.sleep(1)

	camera_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, 5)
	mapper = CoordinateMapper(camera_info)
	def move_arm(x,y,z):
		pos = position()
		pos.x = x
		pos.y = y
		pos.z = z
		pub1.publish(pos)
		# print('Arm pos: ( %3d, %3d, %3d )' % (x,y,z))
	def reset_arm():
		r1 = rospy.Rate(10)
		pub2.publish(status(1))
		r1.sleep()
		pub2.publish(status(1))
		r1.sleep()
	rospy.sleep(1)
	Thread(target=image_thread).start()
	rospy.spin()

if __name__ == '__main__':
	main()