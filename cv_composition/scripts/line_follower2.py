#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means
import rclpy
from rclpy.node import Node

import cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist



rclpy.init()

bridge = cv_bridge.CvBridge()
#cv2.namedWindow("window", 1)


node = Node('follower')
publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)

def image_callback( msg):
    global publisher
    global bridge

    twist = Twist()
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10, 10, 10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w, d = image.shape
    search_top = int(3*h/4)
    search_bot = int(3*h/4) + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    #The proportional controller is implemented in the following four lines which
    #is reposible of linear scaling of an error to drive the control output.
            err = cx - w/2
            twist.linear.x = 0.1
            twist.angular.z = -float(err) / 150
            publisher.publish(twist)
    #cv2.imshow("window", image)
    #cv2.waitKey(3)


subscription = node.create_subscription(Image, 'image',
                                        image_callback,
                                        rclpy.qos.qos_profile_sensor_data)

rclpy.spin(node)
#start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
#stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

