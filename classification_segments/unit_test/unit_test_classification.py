#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Unit Test of the classifcation function calculate_classifications
#LIBARIES: | rospy | math |
#MESSAGE TYPES: | LaserScan | segmentation_msg (costum) |
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************

import unittest
from classification_node_unit_test import Sub_Pub
from classification_segments.msg import objects_msg
import rospy
import math

class test_classification(unittest.TestCase):
	#Unittest for a circle-shaped segment
	def test_calculate_classification_circle(self):
		subpub1 = Sub_Pub()
		subpub1.ranges_received = [0.3,0.29,0.285,0.281,0.278,0.276,0.275,0.276,0.278,0.281,0.285,0.29,0.3 ]
		subpub1.angle_increment_received = math.radians(1.0)
		subpub1.amount_segments_received = 1
		subpub1.start_points_received = [0]
		subpub1.stop_points_received = [12]
		subpub1.msg_to_publish = objects_msg()

		subpub1.calculate_classification()

		assert subpub1.msg_to_publish.lines_start_x == []
		assert subpub1.msg_to_publish.lines_start_y == []
		assert subpub1.msg_to_publish.lines_stop_x == []
		assert subpub1.msg_to_publish.lines_stop_y == []
		assert subpub1.msg_to_publish.lines_closest_point_x == []
		assert subpub1.msg_to_publish.lines_closest_point_y == []
		assert subpub1.msg_to_publish.lines_middle_point_x == []
		assert subpub1.msg_to_publish.lines_middle_point_y == []
		
		difference_center_x = abs(subpub1.msg_to_publish.circles_center_x[0]-0.3176278971)
		assert difference_center_x < 0.001
		difference_center_y = abs(subpub1.msg_to_publish.circles_center_y[0]-0.03338403723)
		assert difference_center_y < 0.001
		difference_circle_radius = abs(subpub1.msg_to_publish.circles_radius[0]-0.04437748038)
		assert difference_circle_radius < 0.001
		assert subpub1.msg_to_publish.rectangles_corner_start_x == []
		assert subpub1.msg_to_publish.rectangles_corner_start_y == []
		assert subpub1.msg_to_publish.rectangles_corner_max_distance_x == []
		assert subpub1.msg_to_publish.rectangles_corner_max_distance_y == []
		assert subpub1.msg_to_publish.rectangles_corner_stop_x == []
		assert subpub1.msg_to_publish.rectangles_corner_stop_y == []
		assert subpub1.msg_to_publish.rectangles_corner_max_distance_mirror_x == []
		assert subpub1.msg_to_publish.rectangles_corner_max_distance_mirror_y == []
		assert subpub1.msg_to_publish.rectangles_center_x == []
		assert subpub1.msg_to_publish.rectangles_center_y == []
		
	#Unittest for a rectangle shaped segment
	def test_calculate_classification_rectangle(self):
		subpub2 = Sub_Pub()
		subpub2.ranges_received = [6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,0.4438,0.4386,0.4264,0.4183,0.4107,0.4183,0.4264,0.4386,0.4438,6.0,6.0,6.0]
		subpub2.angle_increment_received = math.radians(1)
		subpub2.amount_segments_received = 1
		subpub2.start_points_received = [40]
		subpub2.stop_points_received = [48]
		subpub2.msg_to_publish = objects_msg()

		subpub2.calculate_classification()

		assert subpub2.msg_to_publish.lines_start_x == []
		assert subpub2.msg_to_publish.lines_start_y == []
		assert subpub2.msg_to_publish.lines_stop_x == []
		assert subpub2.msg_to_publish.lines_stop_y == []
		assert subpub2.msg_to_publish.lines_closest_point_x == []
		assert subpub2.msg_to_publish.lines_closest_point_y == []
		assert subpub2.msg_to_publish.lines_middle_point_x == []
		assert subpub2.msg_to_publish.lines_middle_point_y == []
		assert subpub2.msg_to_publish.circles_center_x == []
		assert subpub2.msg_to_publish.circles_center_y == []
		assert subpub2.msg_to_publish.circles_radius == []
		difference_rectangles_corner_start_x = abs(subpub2.msg_to_publish.rectangles_corner_start_x[0]-0.3399705238)
		assert difference_rectangles_corner_start_x < 0.001
		difference_rectangles_corner_start_y = abs(subpub2.msg_to_publish.rectangles_corner_start_y[0]-0.2852691411)
		assert difference_rectangles_corner_start_y < 0.001
		difference_rectangles_corner_max_distance_x = abs(subpub2.msg_to_publish.rectangles_corner_max_distance_x[0]-0.2954328559)
		assert difference_rectangles_corner_max_distance_x < 0.001
		difference_rectangles_corner_max_distance_y = abs(subpub2.msg_to_publish.rectangles_corner_max_distance_y[0]-0.2852961927)
		assert difference_rectangles_corner_max_distance_y < 0.001
		difference_rectangles_corner_stop_x = abs(subpub2.msg_to_publish.rectangles_corner_stop_x[0]-0.2969601631)
		assert difference_rectangles_corner_stop_x < 0.001
		difference_rectangles_corner_stop_y = abs(subpub2.msg_to_publish.rectangles_corner_stop_y[0]-0.3298076735)
		assert difference_rectangles_corner_stop_y < 0.001
		difference_rectangles_corner_max_distance_mirror_x= abs(subpub2.msg_to_publish.rectangles_corner_max_distance_mirror_x[0]-0.3414978309)
		assert difference_rectangles_corner_max_distance_mirror_x < 0.001
		difference_rectangles_corner_max_distance_mirror_y = abs(subpub2.msg_to_publish.rectangles_corner_max_distance_mirror_y[0]-0.3297806219)
		assert difference_rectangles_corner_max_distance_mirror_y < 0.001
		difference_rectangles_center_x = abs(subpub2.msg_to_publish.rectangles_center_x[0]-0.3184653434)
		assert difference_rectangles_center_x < 0.001
		difference_rectangles_center_y = abs(subpub2.msg_to_publish.rectangles_center_y[0]-0.3075384073)
		assert difference_rectangles_center_y < 0.001

	#Unittest for a line shaped segment
	def test_calculate_classification_line(self):
		subpub3 = Sub_Pub()
		subpub3.ranges_received = [6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0,0.4438,0.4386,0.4264,0.4183,0.4107,6.0,6.0,6.0]
		subpub3.angle_increment_received = math.radians(1)
		subpub3.amount_segments_received = 1
		subpub3.start_points_received = [40]
		subpub3.stop_points_received = [44]
		subpub3.msg_to_publish = objects_msg()

		subpub3.calculate_classification()

		difference_lines_start_x = abs(subpub3.msg_to_publish.lines_start_x[0]-0.3399705238)
		assert difference_lines_start_x < 0.001
		difference_lines_start_y = abs(subpub3.msg_to_publish.lines_start_y[0]-0.2852691411)
		assert difference_lines_start_y < 0.001
		difference_lines_stop_x = abs(subpub3.msg_to_publish.lines_stop_x[0]-0.2954328559)
		assert difference_lines_stop_x < 0.001
		difference_lines_stop_y = abs(subpub3.msg_to_publish.lines_stop_y[0]-0.2852961927)
		assert difference_lines_stop_y < 0.001
		difference_lines_closest_point_x = abs(subpub3.msg_to_publish.lines_closest_point_x[0]-0.2954328559)
		assert difference_lines_closest_point_x < 0.001
		difference_lines_closest_point_y= abs(subpub3.msg_to_publish.lines_closest_point_y[0]-0.2852961927)
		assert difference_lines_closest_point_y < 0.001
		difference_lines_middle_point_x = abs(subpub3.msg_to_publish.lines_middle_point_x[0]-0.3177016899)
		assert difference_lines_middle_point_x < 0.001
		difference_lines_middle_point_y = abs(subpub3.msg_to_publish.lines_middle_point_y[0]-0.2852826669)
		assert difference_lines_middle_point_y < 0.001
		assert subpub3.msg_to_publish.circles_center_x == []
		assert subpub3.msg_to_publish.circles_center_y == []
		assert subpub3.msg_to_publish.circles_radius == []
		assert subpub3.msg_to_publish.rectangles_corner_start_x == []
		assert subpub3.msg_to_publish.rectangles_corner_start_y == []
		assert subpub3.msg_to_publish.rectangles_corner_max_distance_x == []
		assert subpub3.msg_to_publish.rectangles_corner_max_distance_y == []
		assert subpub3.msg_to_publish.rectangles_corner_stop_x == []
		assert subpub3.msg_to_publish.rectangles_corner_stop_y == []
		assert subpub3.msg_to_publish.rectangles_corner_max_distance_mirror_x == []
		assert subpub3.msg_to_publish.rectangles_corner_max_distance_mirror_y == []
		assert subpub3.msg_to_publish.rectangles_center_x == []
		assert subpub3.msg_to_publish.rectangles_center_y == []


def main():
	unittest.main()

if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
