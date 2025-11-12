# following: https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2 as cv

class WebCamPub(Node):

	def __init__(self):
		super().__init__('webcam')
		self.image_pub = self.create_publisher(Image,'video_raw', 3)
		
		timer_period = 0.100
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
		self.cam = cv.VideoCapture(0)
		if not self.cam.isOpened():
			print('Can not open camera')
			exit()
		self.bridge = CvBridge()
   
	def timer_callback(self):
		msg = Image()
				
		flag, img = self.cam.read()
		if not flag:
			print('Stream ended')
			
		msg= self.bridge.cv2_to_imgmsg(img, "bgr8")
		self.image_pub.publish(msg)
		self.get_logger().info(f'Publishing: {self.i}')
		self.i += 1
   
def main(args=None):
	rclpy.init(args= args)
	
	webcam= WebCamPub()
	
	rclpy.spin(webcam)

	webcam.cam.release()
	webcam.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
  main()
  

