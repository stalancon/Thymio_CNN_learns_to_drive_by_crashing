#!/usr/bin/env python

import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import numpy as np
import random

import cv2
from cv_bridge import CvBridge, CvBridgeError



class ThymioController:

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

        # Sensors
        self.sensors = ["left","center_left", "center", "center_right","right"]
        self.proximity_distances = dict()

        # Sensors subscriber
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.sensors
        ]

        self.flag = False
        self.count = 0

        self.bridge = CvBridge()

        # Image publisher
        self.image_pub = rospy.Publisher(self.name + '/camera/image_raw',Image, queue_size=10)

        # Image subscriber
        self.image_sub = rospy.Subscriber(self.name + '/camera/image_raw',Image,self.image_callback)

        # Path for saving the classified frames
        self.path = rospkg.RosPack().get_path('group_s')+'/frames/'
    
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # get the count number on the image name
            c = str(self.count)

            # save the image in the corresponding class folder
            if (self.count % 3) == 0:
                if self.flag:
                    # obstacle frame
                    img_name = self.path + '/obstacle/image_' + c + '.jpeg'
                    # cv2.imwrite(img_name, cv_image)

                else:
                    # no obstacle frame
                    img_name = self.path + '/free/image_' + c + '.jpeg'
                    # cv2.imwrite(img_name, cv_image)

            self.count+=1

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
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

        w = 0   # angular velocity
        v = 0.1     #linear velocity

        while True:
            # Sleep until the first update is received for the clock and each proximity sensor
            while not rospy.is_shutdown():
                self.rate.sleep()

                if len(self.proximity_distances) == len(self.sensors):
                    break
            
            
            while not rospy.is_shutdown():
                # The robot will drive in a constant velocity and straight until two of its sensors detect an obstacle
                if sum(self.proximity_distances[sensor] < 0.11 for sensor in self.sensors) >= 2:
                    break

                velocity = self.get_control(v,w)
                self.velocity_publisher.publish(velocity)
                self.rate.sleep()
            

            # randomly choose which way to turn (left: 0 or right: 1)
            turn = random.randint(0, 1)
            self.flag = True

            while not rospy.is_shutdown():
                # If there's an obstacle we change our linear and angular velocity,
                # the robot will be turning slowly until the obstacle is out of the way
                
                v = 0.02

                # turn to the right
                if turn == 1:
                    w=-0.4
                    
                    # turn to the left
                if turn == 0:
                    w=0.4

                # Once the obstacle is out the way we go back to our original linear velocity
                if sum(self.proximity_distances[sensor] < 0.1 for sensor in self.sensors) == 1:
                    w = 0
                    v = 0.1
                    break
                
                velocity = self.get_control(v,w)
                self.velocity_publisher.publish(velocity)
                self.rate.sleep()

            self.flag = False

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('thymio_controller')
    name = 'thymio10'

    try:
        controller = ThymioController(name)
        controller.run()

    except rospy.ROSInterruptException as e:
        pass
