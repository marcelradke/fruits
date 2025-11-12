# following: https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import ImageMarker
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge
import time

import numpy as np
import cv2

class Marker(Node):

	def __init__(self):
		super().__init__('marker')
		self.subscription = self.create_subscription( 
			Image, 'video_raw', self.image_callback, 10)
		self.image_pub = self.create_publisher(Image,'video_mark', 3)

		self.sub_apple = self.create_subscription(ImageMarker, 'detection/apple',
			self.apple_callback, 30)
		self.sub_apple_time = self.create_subscription(Duration, 'detection/apple_duration',
			self.apple_time_callback, 3)
		self.apple_time_out= 1 #s #will be reset
						
		self.bridge = CvBridge()
		self.i = 0
		
		self.marker_buffer: list[(ImageMarker,float)]=[]
					
   
	def image_callback(self, msg):
		self.get_logger().info('Got Message')
		img= self.bridge.imgmsg_to_cv2(msg, "bgr8")
		print(type(img))
		img= self.update_image(img)
		
		msg= self.bridge.cv2_to_imgmsg(img, "bgr8")
		self.image_pub.publish(msg)
		self.get_logger().info(f'Publishing: {self.i}')
		self.i += 1
		
	def apple_time_callback(self, msg):
		self.apple_time_out= (msg.sec+(msg.nanosec/10e9)) / 2;
		print('Set apple marker time out [s]: ', self.apple_time_out )
		
	
	def apple_callback(self, msg):
		self.get_logger().info('got apple')		
		self.marker_buffer.append((msg,time.time()))
		self.update_marker_buffer()

	def update_image(self, img):
		t_now= time.time()
		for msg,t in self.marker_buffer:
				if msg.id == ImageMarker.CIRCLE:
					c= (int(msg.position.x), int(msg.position.y))
					r= int(msg.scale)
					cv2.circle(img, center= c, radius=r, color=(0, 255, 0), thickness=2)
		return img		
	
	def update_marker_buffer(self):
		# delete all markers whose timeout is reached since last received marker message
	  t_now= time.time()
	  self.marker_buffer = [(msg,t) for msg,t in self.marker_buffer if t > t_now - self.apple_time_out]
		

	   
def main(args=None):
	rclpy.init(args= args)
	
	marker= Marker()
	
	rclpy.spin(marker)

	marker.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
  main()
  

