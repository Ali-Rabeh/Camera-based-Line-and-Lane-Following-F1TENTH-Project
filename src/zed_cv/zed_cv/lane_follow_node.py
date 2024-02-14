import cv2
import numpy as np

import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import find_lane, warp_img
from steering import pid_control, get_velocity
import time
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

######### LaneFollowNode #########

class LaneFollowNode(Node):

    def __init__(self):
        super().__init__('lane_follow_node')

        # camera settings:
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.flag = 0 # Flag-bit for colour change
        self.lower_yellow = np.array([20, 75, 100]) # only for yellow
        self.upper_yellow = np.array([40, 230, 255])

        # front_mount points (use this if camera is mounted above LIDAR)
        # points = np.float32([[160, 354], [800, 354], [0, 400], [960, 400]])


        # ROI points for warping (rear mount points)
        #old points
        self.points = np.float32([[101, 295], [614,295], [0, 350], [724, 350]])
        #Ali's points
        #self.points = np.float32([[216, 228], [478,214], [21, 300], [664, 281]])

        # creating a publisher for AckermannDriveStamped messages
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # create a timer to publish messages at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_drive)

    def publish_drive(self):
        
        
        ret, frame = self.cap.read()
        crop = cv2.resize(frame, (480, 270))

        # warping to the frame based on the selected points (BEV)
        bev = warp_img(frame, self.points, w=480, h=270)

        # call detect lanes to overlay lane lines, find error
        detected_lanes, error = find_lane(bev,self.lower_yellow,self.upper_yellow)

        # call pid control to find the steering angle and velocity
        self.get_logger().info('Flag: %d' % self.flag)
        if error != np.inf: 
        	steering_angle = pid_control(error, time.time())
        	velocity = get_velocity(steering_angle)
        else:
        	steering_angle = 0.0
        	velocity = 0.0
        	
        	if self.flag == 1:
        		#change colour range to orange
        		self.lower_yellow = np.array([0, 50, 110])
        		self.upper_yellow = np.array([20, 230, 255])
        		self.flag = 2
        		
        	if self.flag == 0:	 #loop one time to publish steering and velocity
        		self.flag = 1
        		
        	if self.flag == 3:
        		#change colour range to yellow
        		self.lower_yellow = np.array([20, 75, 100]) # only for yellow
        		self.upper_yellow = np.array([40, 230, 255])
        		self.flag = 0        	
        	
        	if self.flag == 2:	#loop one time to publish steering and velocity
        		self.flag = 3
        		
        	

        # publish AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = float(velocity)
        self.pub_drive.publish(drive_msg)

        # log steering angle and velocity
        self.get_logger().info('Steering angle: %f' % steering_angle)
        self.get_logger().info('Velocity: %f' % velocity)
        self.get_logger().info('Error: %f' % error)

        cv2.imshow("frame", crop)
        cv2.imshow('lane', detected_lanes)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
