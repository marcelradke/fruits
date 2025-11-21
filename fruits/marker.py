# following: https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import ImageMarker
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge
import time

from matplotlib import pyplot as plt
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

		# marker time out will be reset with next marker message
		self.apple_time_out= 1 #s 
		
		# Plotting
		cmap= plt.get_cmap('jet')
		apple_colors= cmap(np.linspace(0,1,101))
		self.apple_colors= (apple_colors[::-1,0:3]*255)
						
		self.bridge = CvBridge()
		self.i = 0
		
		self.marker_buffer: list[(ImageMarker,float)]=[]
					
   
	def image_callback(self, msg):
		self.get_logger().info('Got Message')
		img= self.bridge.imgmsg_to_cv2(msg, "bgr8")
		
		# draw markers
		img= self.update_image(img)
		
		# publish
		msg= self.bridge.cv2_to_imgmsg(img, "bgr8")
		self.image_pub.publish(msg)
		self.get_logger().info(f'Publishing: {self.i}')
		self.i += 1
		
	def apple_callback(self, msg):
		self.get_logger().info('got apple')
		
		# set marker duration
		self.apple_time_out= msg.lifetime.sec+(msg.lifetime.nanosec/10e9) / 2;
		self.get_logger().info(f'type duration {type(msg.lifetime)}')
		s= f'Set apple marker time out [s]: {self.apple_time_out}'
		self.get_logger().info(s)
		
		# add message
		self.marker_buffer.append((msg,time.time()))
		self.update_marker_buffer()

	def update_image(self, img):
		t_now= time.time()
		for msg,t in self.marker_buffer:
				if msg.id == ImageMarker.CIRCLE:
					# Postion
					x= int(msg.position.x)
					y= int(msg.position.y)
					c= (x, y)
					r= int(msg.scale)
					# Color is defined by prediction confidence
					conf= msg.outline_color.a
					self.apple_colors[int(conf*100)]
					rgb= self.apple_colors[int(conf*100)]
					cv2.circle(img, center= c, radius=r, color= rgb, thickness=2)
					img = cv2.putText(img, str(int(conf*100)), (int(x-10),int(y-r-2)),
					                 cv2.FONT_HERSHEY_SIMPLEX, fontScale= 0.7, color= rgb, 
					                 thickness= 2)
		return img		
	
	def update_marker_buffer(self):
		# delete all markers whose timeout is reached 
		# since last received marker message
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
  

