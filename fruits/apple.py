import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import ImageMarker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

import numpy as np
import torch
from ultralytics import YOLO
import cv2 as cv
from cv_bridge import CvBridge	

class Apple_Detector(Node):


	def __init__(self):
		super().__init__('apple_detector')
		self.subscription = self.create_subscription(
			Image, 'video_raw', self.listener_callback, 1)
		self.marker_pub= self.create_publisher(ImageMarker,'detection/apple', 3)
		self.duration_pub= self.create_publisher(Duration,'detection/apple_duration', 1)
		
		self.bridge = CvBridge()
		self.i = 0
		
		self.model= YOLO('yolov5su.pt')

		
	def listener_callback(self, msg):
		self.get_logger().info('Got Image')
		img= self.bridge.imgmsg_to_cv2(msg, "bgr8")

		msg_list, msg_detection_time = self.detect(img)
						
		self.duration_pub.publish(msg_detection_time)
		
		for msg in msg_list:
			self.marker_pub.publish(msg)
			self.get_logger().info(f'Publishing: {self.i}')
			self.i += 1
		
	def detect(self, img) -> (list[ImageMarker], Duration):
		scale= 0.5
		ID= 47 # apple
				
		t_s= self.get_clock().now()
		
		result = self.model(img)
		
		time_msg= (self.get_clock().now()-t_s).to_msg()
		self.get_logger().info(f'[Duration] : {self.get_clock().now()-t_s}')
		self.get_logger().info(f'[Duration MSG] : {time_msg}')
		self.get_logger().info(f'[Duration MSG type] : {type(time_msg)}')
		
		apple_idx= result[0].boxes.cls == 47
		boxes= result[0].boxes.xywh[apple_idx] # center-x, center-y, width, height
		
		marker_list: list[ImageMarker]= []

		for box in boxes:
			x,y,w,h= map(int, box)
			r= int( (w+h)/2*scale )
	
			marker= ImageMarker()
			marker.id= marker.CIRCLE
			point= Point()
			point.x= float(x)
			point.y= float(y)
			point.z= 0.0
			marker.position= point
			marker.scale= float(r)
	
			marker_list.append(marker)
		
		return (marker_list, time_msg)


def main(args=None):
	rclpy.init(args= args)
	
	apple_detector= Apple_Detector()
	
	rclpy.spin(apple_detector)
	
	marker.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
