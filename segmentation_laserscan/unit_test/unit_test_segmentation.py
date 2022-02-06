#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Unit Test of the segmentation function calculate_segments
#LIBARIES: | rospy | math |
#MESSAGE TYPES: | LaserScan | segmentation_msg (costum) |
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************

import unittest
from segmentation_node_unit_test import Sub_Pub
import rospy
import math
from segmentation_laserscan.msg import segmentation_msg

class test_segmentation(unittest.TestCase):

	def test_calculate_segments1(self):
		subpub1 = Sub_Pub()
		subpub1.ranges_received = [2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0]
		subpub1.angle_increment_received = math.radians(36.0)
		subpub1.msg_to_publish = segmentation_msg()

		subpub1.calculate_segments()

		assert subpub1.msg_to_publish.ranges == [2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0]
		assert subpub1.msg_to_publish.angle_increment == math.radians(36.0)
		assert subpub1.msg_to_publish.start_points == []
		assert subpub1.msg_to_publish.stop_points == []
		assert subpub1.msg_to_publish.amount_segments == 0
		
		
	def test_calculate_segments2(self):
		subpub2 = Sub_Pub()
		subpub2.ranges_received = [2.0, 2.0, 2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
		subpub2.angle_increment_received = math.radians(1.0)
		subpub2.msg_to_publish = segmentation_msg()

		subpub2.calculate_segments()

		assert subpub2.msg_to_publish.ranges == [2.0, 2.0, 2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
		assert subpub2.msg_to_publish.angle_increment == math.radians(1.0)
		assert subpub2.msg_to_publish.start_points == [0,10]
		assert subpub2.msg_to_publish.stop_points == [9,19]
		assert subpub2.msg_to_publish.amount_segments == 2

	def test_calculate_segments3(self):
		subpub3 = Sub_Pub()
		subpub3.ranges_received = [2.0, 2.0, 2.0,2.0,2.0,5.0,5.0,5.0,2.0, 2.0, 2.0,2.0,2.0]
		subpub3.angle_increment_received = math.radians(1.0)
		subpub3.msg_to_publish = segmentation_msg()

		subpub3.calculate_segments()

		assert subpub3.msg_to_publish.ranges == [2.0, 2.0, 2.0,2.0,2.0,2.0,2.0,2.0,2.0, 2.0, 2.0,2.0,2.0]
		assert subpub3.msg_to_publish.angle_increment == math.radians(1.0)
		assert subpub3.msg_to_publish.start_points == [0]
		assert subpub3.msg_to_publish.stop_points == [12]
		assert subpub3.msg_to_publish.amount_segments == 1


def main():
	unittest.main()


if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
