#!/usr/bin/env python

import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf


class Explorer:

	image_w = 210
	image_h = 160


	def __init__(self, name):
		"""Initialization."""

		self.name = name

		# log robot name to console
		rospy.loginfo('Controlling %s' % self.name)

		# create velocity publisher
		self.velocity_publisher = rospy.Publisher(
		self.name + '/cmd_vel',  # name of the topic
		Twist,  # message type
		queue_size=10  # queue size
		)

		# create pose subscriber
		self.pose_subscriber = rospy.Subscriber(
		self.name + '/odom',  # name of the topic
		Odometry,  # message type
		self.log_odometry  # function that hanldes incoming messages
		)

		# flag to indicate whether the robot should turn or not depending on the prediction of the CNN model
		self.flag = False

		# load trained CNN model
		path = rospkg.RosPack().get_path('group_s')
		model_path = path + '/cnn_model.h5'
		self.model = tf.keras.models.load_model(model_path)

		self.bridge = CvBridge()

		# Image publisher
		self.image_pub = rospy.Publisher(self.name + '/camera/image_raw',Image, queue_size=10)

		# Image subscriber
		self.image_sub = rospy.Subscriber(self.name + '/camera/image_raw',Image,self.image_callback)

		# tell ros to call stop when the program is terminated
		rospy.on_shutdown(self.stop)

		# initialize pose to (X=0, Y=0, theta=0)
		self.pose = Pose()

		# initialize linear and angular velocities to 0
		self.velocity = Twist()

		# set node update frequency in Hz
		self.rate = rospy.Rate(10)

	def human_readable_pose2d(self, pose):
		"""Converts pose message to a human readable pose tuple."""

		# create a quaternion from the pose
		quaternion = (
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w
		)

		# convert quaternion rotation to euler rotation
		roll, pitch, yaw = euler_from_quaternion(quaternion)

		result = (
		pose.position.x,  # x position
		pose.position.y,  # y position
		yaw  # theta angle
		)

		return result

	def log_odometry(self, data):
		"""Updates robot pose and velocities, and logs pose to console."""

		self.pose = data.pose.pose
		self.velocity = data.twist.twist

		printable_pose = self.human_readable_pose2d(self.pose)

		# log robot's pose
		rospy.loginfo_throttle(
		period=1,  # log every 10 seconds
		msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
		)

	def update_proximity(self, data, sensor):
		self.proximity_distances[sensor] = data.range

	def image_callback(self, msg):
		try:
			# Convert your ROS Image message to OpenCV2
			img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

			# preprocess the image
			cv_image = cv2.resize(img, (Explorer.image_w, Explorer.image_h),interpolation = cv2.INTER_AREA)
			image = np.resize(cv_image, (1, Explorer.image_w, Explorer.image_h, 3))

			# get the image from the camera, run it through the trained CNN and obtain a prediction
			prediction = self.model.predict(image)

			# If the model predicts an obstacle we set the flag as TRUE and print the message on the image window
			if prediction == 1:
				self.flag = True
				img = cv2.putText(img,"Obstacle detected - turning...", (20,400), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))

			# If the model predicts no obstacle we set the flag as FALSE and print the message on the image window
			if prediction == 0:
				self.flag = False
				img = cv2.putText(img,"No obstacle, let's keep moving!", (20,400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 0))

		except CvBridgeError as e:
			print(e)

		cv2.imshow("Image window", img)
		cv2.waitKey(3)


	def get_control(self, v, w):
		return Twist(
		linear=Vector3(
		v,  
		.0,
		.0,
		),
		angular=Vector3(
		.0,
		.0,
		w
		)
		)


	def run(self):
		"""Controls the Thymio."""

		# while True:
		while not rospy.is_shutdown():
			# The robot will turn if there prediction says there's an obstacle, if not it will keep moving in a straight line
			if self.flag:  
				# turn and move slightly backwards while turning
				v = 0.02
				w = -0.4

			else:  
				# keep moving straight
				v = 0.09
				w = 0.0

			velocity = self.get_control(v,w)
			self.velocity_publisher.publish(velocity)
			self.rate.sleep()


	def stop(self):
		"""Stops the robot."""

		self.velocity_publisher.publish(
		Twist()  # set velocities to 0
		)

		self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('explorer_controller')
	name = 'thymio10'

	try:
		controller = Explorer(name)
		controller.run()

	except rospy.ROSInterruptException as e:
		pass
