#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Unit Test of the visualization function publish_marker
#LIBARIES: | rospy | math | unittest
#MESSAGE TYPES: | Marker | point |
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************

import unittest
from visualization_node_unit_test import Sub_Pub
import rospy
from visualization_msgs.msg import Marker
import math
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

class test_visualization(unittest.TestCase):

	def test_publish_marker_circle(self):
		subpub1 = Sub_Pub()
		subpub1.lines_start_x_received = []
		subpub1.lines_start_y_received = []
		subpub1.lines_stop_x_received = []
		subpub1.lines_stop_y_received = []
		subpub1.lines_closest_point_x_received = []
		subpub1.lines_closest_point_y_received = []
		subpub1.lines_middle_point_x_received = []
		subpub1.lines_middle_point_y_received = []
		subpub1.circles_center_x_received = [0.3176278971]
		subpub1.circles_center_y_received = [0.03338403723]
		subpub1.circles_radius_received = [0.04437748038]
		subpub1.rectangles_corner_start_x_received = []
		subpub1.rectangles_corner_start_y_received = []
		subpub1.rectangles_corner_max_distance_x_received = []
		subpub1.rectangles_corner_max_distance_y_received = []
		subpub1.rectangles_corner_stop_x_received = []
		subpub1.rectangles_corner_stop_y_received = []
		subpub1.rectangles_corner_max_distance_mirror_x_received = []
		subpub1.rectangles_corner_max_distance_mirror_y_received = []
		subpub1.rectangles_center_x_received = []
		subpub1.rectangles_center_y_received = []
		
		subpub1.msg_to_publish = Marker()

		subpub1.publish_marker()

		assert subpub1.msg_to_publish.header.frame_id == '/laser'
		assert subpub1.msg_to_publish.type == subpub1.msg_to_publish.CYLINDER
		assert subpub1.msg_to_publish.id == 1
		assert subpub1.msg_to_publish.action == subpub1.msg_to_publish.ADD
		assert subpub1.msg_to_publish.scale.x == subpub1.circles_radius_received[0]*2
		assert subpub1.msg_to_publish.scale.y == subpub1.circles_radius_received[0]*2
		assert subpub1.msg_to_publish.scale.z == 0.075/2
		assert subpub1.msg_to_publish.color.a == 1
		assert subpub1.msg_to_publish.color.r == 0
		assert subpub1.msg_to_publish.color.g == 1
		assert subpub1.msg_to_publish.color.b == 0
		my_point = Point()
		my_point.x = -subpub1.circles_center_x_received[0]
		my_point.y = -subpub1.circles_center_y_received[0]
		my_point.z = 0.0
		assert subpub1.msg_to_publish.pose.position == my_point
		
	def test_publish_marker_rectangle(self):
		subpub2 = Sub_Pub()
		subpub2.lines_start_x_received = []
		subpub2.lines_start_y_received = []
		subpub2.lines_stop_x_received = []
		subpub2.lines_stop_y_received = []
		subpub2.lines_closest_point_x_received = []
		subpub2.lines_closest_point_y_received = []
		subpub2.lines_middle_point_x_received = []
		subpub2.lines_middle_point_y_received = []
		subpub2.circles_center_x_received = []
		subpub2.circles_center_y_received = []
		subpub2.circles_radius_received = []
		subpub2.rectangles_corner_start_x_received = [0.3399705238]
		subpub2.rectangles_corner_start_y_received = [0.2852691411]
		subpub2.rectangles_corner_max_distance_x_received = [0.2954328559]
		subpub2.rectangles_corner_max_distance_y_received = [0.2852961927]
		subpub2.rectangles_corner_stop_x_received = [0.2969601631]
		subpub2.rectangles_corner_stop_y_received = [0.3298076735]
		subpub2.rectangles_corner_max_distance_mirror_x_received = [0.3414978309]
		subpub2.rectangles_corner_max_distance_mirror_y_received = [0.3297806219]
		subpub2.rectangles_center_x_received = [0.3184653434]
		subpub2.rectangles_center_y_received = [0.3075384073]
		 
		subpub2.msg_to_publish = Marker()

		subpub2.publish_marker()

		assert subpub2.msg_to_publish.header.frame_id == '/laser'
		assert subpub2.msg_to_publish.type == subpub2.msg_to_publish.CUBE
		assert subpub2.msg_to_publish.id == 0
		assert subpub2.msg_to_publish.action == subpub2.msg_to_publish.ADD

		rad_around_z = math.pi/2
		(x, y, z, w) = quaternion_from_euler(0, 0, rad_around_z)

		difference_scale_x_rectangle = abs(subpub2.msg_to_publish.scale.x-0.044537676)
		difference_scale_y_rectangle = abs(subpub2.msg_to_publish.scale.y-0.044537676)

		assert difference_scale_x_rectangle < 0.001
		assert difference_scale_y_rectangle < 0.001
		assert subpub2.msg_to_publish.scale.z == 0.075/2
		assert subpub2.msg_to_publish.color.a == 1
		assert subpub2.msg_to_publish.color.r == 1
		assert subpub2.msg_to_publish.color.g == 1
		assert subpub2.msg_to_publish.color.b == 0
		assert subpub2.msg_to_publish.pose.orientation.x == x
		assert subpub2.msg_to_publish.pose.orientation.y == y

		difference_orientation_z = abs(-z-subpub2.msg_to_publish.pose.orientation.z)
		assert difference_orientation_z < 0.001

		difference_orientation_w = abs(w-subpub2.msg_to_publish.pose.orientation.w)
		assert difference_orientation_w < 0.001

		my_point = Point()
		my_point.x = -subpub2.rectangles_center_x_received[0]
		my_point.y = -subpub2.rectangles_center_y_received[0]
		my_point.z = 0.0
		
		difference_point_x = abs(my_point.x-subpub2.msg_to_publish.pose.position.x)
		assert difference_point_x < 0.001
		difference_point_y = abs(my_point.y-subpub2.msg_to_publish.pose.position.y)
		print(difference_point_y)
		assert difference_point_y < 0.001
		difference_point_z = abs(my_point.z-subpub2.msg_to_publish.pose.position.z)
		assert difference_point_z < 0.001

	def test_publish_marker_line(self):
		subpub3 = Sub_Pub()
		subpub3.lines_start_x_received = [0.3399705238]
		subpub3.lines_start_y_received = [0.2852691411]
		subpub3.lines_stop_x_received = [0.2954328559]
		subpub3.lines_stop_y_received = [0.2852961927]
		subpub3.lines_closest_point_x_received = [0.2954328559]
		subpub3.lines_closest_point_y_received = [0.2852961927]
		subpub3.lines_middle_point_x_received = [0.3177016899]
		subpub3.lines_middle_point_y_received = [0.2852826669]
		subpub3.circles_center_x_received = []
		subpub3.circles_center_y_received = []
		subpub3.circles_radius_received = []
		subpub3.rectangles_corner_start_x_received = []
		subpub3.rectangles_corner_start_y_received = []
		subpub3.rectangles_corner_max_distance_x_received = []
		subpub3.rectangles_corner_max_distance_y_received = []
		subpub3.rectangles_corner_stop_x_received = []
		subpub3.rectangles_corner_stop_y_received = []
		subpub3.rectangles_corner_max_distance_mirror_x_received = []
		subpub3.rectangles_corner_max_distance_mirror_y_received = []
		subpub3.rectangles_center_x_received = []
		subpub3.rectangles_center_y_received = []
		
		subpub3.msg_to_publish = Marker()

		subpub3.publish_marker()

		assert subpub3.msg_to_publish.header.frame_id == '/laser'
		assert subpub3.msg_to_publish.type == subpub3.msg_to_publish.CUBE
		assert subpub3.msg_to_publish.id == 2
		assert subpub3.msg_to_publish.action == subpub3.msg_to_publish.ADD
		assert subpub3.msg_to_publish.scale.x == 0.005

		rad_around_z = math.pi/2
		(x, y, z, w) = quaternion_from_euler(0, 0, rad_around_z)

		difference_scale_y_line = abs(subpub3.msg_to_publish.scale.y-0.044537676)
		
		assert difference_scale_y_line < 0.001
		assert subpub3.msg_to_publish.scale.z == 0.075/2
		assert subpub3.msg_to_publish.color.a == 1
		assert subpub3.msg_to_publish.color.r == 0
		assert subpub3.msg_to_publish.color.g == 0
		assert subpub3.msg_to_publish.color.b == 1
		assert subpub3.msg_to_publish.pose.orientation.x == x
		assert subpub3.msg_to_publish.pose.orientation.y == y

		difference_orientation_z = abs(-z-subpub3.msg_to_publish.pose.orientation.z)
		assert difference_orientation_z < 0.001

		difference_orientation_w = abs(w-subpub3.msg_to_publish.pose.orientation.w)
		assert difference_orientation_w < 0.001
		
		my_point = Point()
		my_point.x = -subpub3.lines_middle_point_x_received[0]
		my_point.y = -subpub3.lines_middle_point_y_received[0]
		my_point.z = 0.0
		assert subpub3.msg_to_publish.pose.position == my_point

		difference_point_x = abs(my_point.x-subpub3.msg_to_publish.pose.position.x)
		assert difference_point_x < 0.001
		difference_point_y = abs(my_point.y-subpub3.msg_to_publish.pose.position.y)
		assert difference_point_y < 0.001
		difference_point_z = abs(my_point.z-subpub3.msg_to_publish.pose.position.z)
		assert difference_point_z < 0.001

def main():
	unittest.main()

if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
