# following: https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

class Display(Node):

	def __init__(self):
		super().__init__('display')
		self.subscription = self.create_subscription( 
			Image, 'video_mark', self.listener_callback, 10)
		self.bridge = CvBridge()
	
	def listener_callback(self, msg):
		self.get_logger().info('Got Message')
		img= self.bridge.imgmsg_to_cv2(msg, "bgr8")
		cv2.imshow('Display', img)
		cv2.waitKey(5)

		
def main(args=None):
	rclpy.init(args=args)
	display= Display()
	rclpy.spin(display)
	
	cv2.destroyAllWindows()
	display.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
		
