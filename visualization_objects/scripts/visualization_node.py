#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Calculation of the object orientation and publishing of Markers
#LIBARIES: | rospy | math | quaternion_from_euler |
#MESSAGE TYPES: | objects_msg (costum) | Marker | Point |
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************
import rospy
import math
from classification_segments.msg import objects_msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

class Sub_Pub(object):
	#Initialize an object of the class "Sub_Pub"
	def __init__(self):
		self.lines_start_x_received = []
		self.lines_start_y_received = []
		self.lines_stop_x_received = []
		self.lines_stop_y_received = []
		self.lines_closest_point_x_received = []
		self.lines_closest_point_y_received = []
		self.lines_middle_point_x_received = []
		self.lines_middle_point_y_received = []
		self.circles_center_x_received = []
		self.circles_center_y_received = []
		self.circles_radius_received = []
		self.rectangles_corner_start_x_received = []
		self.rectangles_corner_start_y_received = []
		self.rectangles_corner_max_distance_x_received = []
		self.rectangles_corner_max_distance_y_received = []
		self.rectangles_corner_stop_x_received = []
		self.rectangles_corner_stop_y_received = []
		self.rectangles_corner_max_distance_mirror_x_received = []
		self.rectangles_corner_max_distance_mirror_y_received = []
		self.rectangles_center_x_received = []
		self.rectangles_center_y_received = []

		self.pub = rospy.Publisher('marker', Marker, queue_size=10) 
		self.msg_to_publish = Marker()
		
	#Receiving the subscribtion-data, and publication of the object-Marker
	def callback(self, msg):
		self.lines_start_x_received = msg.lines_start_x
		self.lines_start_y_received = msg.lines_start_y
		self.lines_stop_x_received = msg.lines_stop_x
		self.lines_stop_y_received = msg.lines_stop_y
		self.lines_closest_point_x_received = msg.lines_closest_point_x
		self.lines_closest_point_y_received = msg.lines_closest_point_y
		self.lines_middle_point_x_received = msg.lines_middle_point_x
		self.lines_middle_point_y_received = msg.lines_middle_point_y
		self.circles_center_x_received = msg.circles_center_x
		self.circles_center_y_received = msg.circles_center_y
		self.circles_radius_received = msg.circles_radius
		self.rectangles_corner_start_x_received = msg.rectangles_corner_start_x
		self.rectangles_corner_start_y_received = msg.rectangles_corner_start_y
		self.rectangles_corner_max_distance_x_received = msg.rectangles_corner_max_distance_x
		self.rectangles_corner_max_distance_y_received = msg.rectangles_corner_max_distance_y
		self.rectangles_corner_stop_x_received = msg.rectangles_corner_stop_x
		self.rectangles_corner_stop_y_received = msg.rectangles_corner_stop_y
		self.rectangles_corner_max_distance_mirror_x_received = msg.rectangles_corner_max_distance_mirror_x
		self.rectangles_corner_max_distance_mirror_y_received = msg.rectangles_corner_max_distance_mirror_y
		self.rectangles_center_x_received = msg.rectangles_center_x
		self.rectangles_center_y_received = msg.rectangles_center_y
		self.publish_marker()
		
	#calculation of objectorientation and Marker publication
	def publish_marker(self):
		
		lines_start_x = self.lines_start_x_received
		lines_start_y = self.lines_start_y_received
		lines_stop_x = self.lines_stop_x_received
		lines_stop_y = self.lines_stop_y_received
		lines_closest_point_x = self.lines_closest_point_x_received
		lines_closest_point_y = self.lines_closest_point_y_received
		lines_middle_point_x = self.lines_middle_point_x_received
		lines_middle_point_y = self.lines_middle_point_y_received
		circles_center_x = self.circles_center_x_received
		circles_center_y = self.circles_center_y_received
		circles_radius = self.circles_radius_received
		rectangles_corner_start_x = self.rectangles_corner_start_x_received 
		rectangles_corner_start_y = self.rectangles_corner_start_y_received 
		rectangles_corner_max_distance_x = self.rectangles_corner_max_distance_x_received
		rectangles_corner_max_distance_y = self.rectangles_corner_max_distance_y_received
		rectangles_corner_stop_x = self.rectangles_corner_stop_x_received 
		rectangles_corner_stop_y = self.rectangles_corner_stop_y_received 
		rectangles_corner_max_distance_mirror_x = self.rectangles_corner_max_distance_mirror_x_received
		rectangles_corner_max_distance_mirror_y = self.rectangles_corner_max_distance_mirror_y_received
		rectangles_center_x = self.rectangles_center_x_received
		rectangles_center_y = self.rectangles_center_y_received
		i = 0
		j = 0
		n = 0

		#publication of the classified rectangles
		for i in range(0, len(rectangles_center_x)):
			#calculation of the rectangles length and width
			scale_x_rectangle=(math.sqrt(((rectangles_corner_stop_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_stop_y[i]-rectangles_corner_max_distance_y[i])**2)))
			scale_y_rectangle=(math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))

			#calculation if the rectangle is located in the first quadrant 	
			if (rectangles_center_x[i] >= 0 and rectangles_center_y[i] >= 0):
				#calculation of the rectangle orientation in rad
				rad_around_z = math.acos((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))

			#calculation if the rectangle is located in the third quadrant
			if (rectangles_center_x[i] <= 0 and rectangles_center_y[i] <= 0):
				rad_around_z = -math.acos((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))

			#calculation if the rectangle is located in the fourth quadrant
			if (rectangles_center_x[i] >= 0 and rectangles_center_y[i] <= 0 ):
				#calculation depending on the rectangle alignment to the sensor
				if (rectangles_corner_start_x[i] >= rectangles_corner_max_distance_x[i]):	
					rad_around_z = -math.acos((rectangles_corner_max_distance_y[i]-rectangles_corner_start_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))
				if (rectangles_corner_start_x[i] < rectangles_corner_max_distance_x[i]):
					rad_around_z = -math.acos((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))

			#calculation if the rectangle is located in the second quadrant
			if (rectangles_center_x[i] < 0 and rectangles_center_y[i] > 0):
				if (rectangles_corner_start_x[i] >= rectangles_corner_max_distance_x[i]):	
					rad_around_z = -math.acos((rectangles_corner_max_distance_y[i]-rectangles_corner_start_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))
				if (rectangles_corner_start_x[i] < rectangles_corner_max_distance_x[i]):	
					rad_around_z = -math.acos((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])/((math.sqrt(((rectangles_corner_start_x[i]-rectangles_corner_max_distance_x[i])**2)+((rectangles_corner_start_y[i]-rectangles_corner_max_distance_y[i])**2)))))

			#calculation of the quaternion objectorientation
			(x, y, z, w) = quaternion_from_euler(0, 0, rad_around_z)
			#update of the calculated rectangle data
			self.msg_to_publish.header.frame_id = '/laser'
			self.msg_to_publish.type = self.msg_to_publish.CUBE
			self.msg_to_publish.id = i
			self.msg_to_publish.action = self.msg_to_publish.ADD
			self.msg_to_publish.scale.x = scale_x_rectangle
			self.msg_to_publish.scale.y = scale_y_rectangle
			self.msg_to_publish.scale.z = 0.075/2
			self.msg_to_publish.color.a = 1
			self.msg_to_publish.color.r = 1
			self.msg_to_publish.color.g = 1
			self.msg_to_publish.color.b = 0
			self.msg_to_publish.pose.orientation.x = x
			self.msg_to_publish.pose.orientation.y = y
			self.msg_to_publish.pose.orientation.z = -z
			self.msg_to_publish.pose.orientation.w = w
			my_point = Point()
			#taking the negativ coordinates, because the inter rviz coordinate system is the opposite of the sensor coordinate system
			my_point.x = -rectangles_center_x[i]
			my_point.y = -rectangles_center_y[i]
			my_point.z = 0.0
			self.msg_to_publish.pose.position = my_point
			self.msg_to_publish.lifetime = rospy.Duration(0.5)

			self.pub.publish(self.msg_to_publish)
			
		#publication of the classified circles
		for j in range(0, len(circles_center_x)):
			#calculation of the quaternion objectorientation
			(x, y, z, w) = quaternion_from_euler(0, 0, 0)
			#update of the calculated circle data
			self.msg_to_publish.header.frame_id = '/laser'
			self.msg_to_publish.type = self.msg_to_publish.CYLINDER
			self.msg_to_publish.id = j + i + 1
			self.msg_to_publish.action = self.msg_to_publish.ADD
			self.msg_to_publish.scale.x = circles_radius[j]*2
			self.msg_to_publish.scale.y = circles_radius[j]*2
			self.msg_to_publish.scale.z = 0.075/2
			self.msg_to_publish.color.a = 1
			self.msg_to_publish.color.r = 0
			self.msg_to_publish.color.g = 1
			self.msg_to_publish.color.b = 0
			self.msg_to_publish.pose.orientation.x = x
			self.msg_to_publish.pose.orientation.y = y
			self.msg_to_publish.pose.orientation.z = z
			self.msg_to_publish.pose.orientation.w = w
			my_point = Point()
			my_point.x = -circles_center_x[j]
			my_point.y = -circles_center_y[j]
			my_point.z = 0.0
			self.msg_to_publish.pose.position = my_point
			self.msg_to_publish.lifetime = rospy.Duration(0.5)
			
			self.pub.publish(self.msg_to_publish)
			
		#publication of the classificated lines
		for n in range(0, len(lines_closest_point_x)):
			#calculation of the length of the line
			scale_lines = (math.sqrt(((lines_start_x[n]-lines_stop_x[n])**2)+((lines_start_y[n]-lines_stop_y[n])**2)))
			#calculation if the line is located in the first or fourth quadrant
			if (lines_middle_point_x[n] >= 0):
				#calculation of orientation depending on the rectangle alignment to the sensor
				if (lines_start_x[n]**2 > lines_stop_x[n]**2):
					rad_around_z = math.acos((lines_start_y[n]-lines_stop_y[n])/((math.sqrt(((lines_stop_x[n]-lines_start_x[n])**2)+((lines_stop_y[n]-lines_start_y[n])**2)))))
				if (lines_start_x[n]**2 <= lines_stop_x[n]**2):
					rad_around_z = math.acos((lines_stop_y[n]-lines_start_y[n])/((math.sqrt(((lines_stop_x[n]-lines_start_x[n])**2)+((lines_stop_y[n]-lines_start_y[n])**2)))))
			#calculation if the line is located in the second or thrid quadrant
			if (lines_middle_point_x[n] < 0):
				if (lines_start_x[n]**2 > lines_stop_x[n]**2):
					rad_around_z = -math.acos((lines_start_y[n]-lines_stop_y[n])/((math.sqrt(((lines_stop_x[n]-lines_start_x[n])**2)+((lines_stop_y[n]-lines_start_y[n])**2)))))
				if (lines_start_x[n]**2 <= lines_stop_x[n]**2):
					rad_around_z = -math.acos((lines_stop_y[n]-lines_start_y[n])/((math.sqrt(((lines_stop_x[n]-lines_start_x[n])**2)+((lines_stop_y[n]-lines_start_y[n])**2)))))
				
			#calculation of the quaternion objectorientation
			(x, y, z, w) = quaternion_from_euler(0, 0, rad_around_z)
			#update of the calculated line data
			self.msg_to_publish.header.frame_id = '/laser'
			self.msg_to_publish.type = self.msg_to_publish.CUBE
			self.msg_to_publish.id = n + i + j + 2
			self.msg_to_publish.action = self.msg_to_publish.ADD
			self.msg_to_publish.scale.x = 0.005
			self.msg_to_publish.scale.y = scale_lines
			self.msg_to_publish.scale.z = 0.075/2
			self.msg_to_publish.color.a = 1
			self.msg_to_publish.color.r = 0
			self.msg_to_publish.color.g = 0
			self.msg_to_publish.color.b = 1
			self.msg_to_publish.pose.orientation.x = x
			self.msg_to_publish.pose.orientation.y = y
			self.msg_to_publish.pose.orientation.z = - z
			self.msg_to_publish.pose.orientation.w = w
			my_point = Point()
			my_point.x = -lines_middle_point_x[n]
			my_point.y = -lines_middle_point_y[n]
			my_point.z = 0.0
			self.msg_to_publish.pose.position = my_point
			self.msg_to_publish.lifetime = rospy.Duration(0.5)

			self.pub.publish(self.msg_to_publish)	

def main():
	#creates an Objekt of the class Sub_Pub
	sub_pub = Sub_Pub()
	rospy.init_node('visualization_objects', anonymous=True)
	rospy.Subscriber("/objects", objects_msg, sub_pub.callback)	
	
	rospy.spin()


if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
