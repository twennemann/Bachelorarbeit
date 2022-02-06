#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Segmentation of the already filtered LaserScan from the RPLIDAR 
#LIBARIES: | rospy | math |
#MESSAGE TYPES: | LaserScan | segmentation_msg (costum) |
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************

import rospy
import math
from sensor_msgs.msg import LaserScan
from segmentation_laserscan.msg import segmentation_msg


class Sub_Pub(object):
	#Initialize an object of the class "Sub_Pub"
	def __init__(self):
		self.ranges_received = []
		self.angle_increment_received = 0
		self.pub = rospy.Publisher('segmentation', segmentation_msg, queue_size=10) 
		self.msg_to_publish = segmentation_msg()

	#Receiving the subscribtion-data, and publication of the calculated segments 
	def callback(self, msg):
		self.ranges_received = msg.ranges 
		self.angle_increment_received = msg.angle_increment # 0.017453
		self.calculate_segments()
		self.pub.publish(self.msg_to_publish)

	#Update data to publish
	def update_data_to_publish(self, start_points, stop_points, amount_segments):
		self.msg_to_publish.ranges = self.ranges
		self.msg_to_publish.angle_increment = self.angle_increment
		self.msg_to_publish.start_points = start_points
		self.msg_to_publish.stop_points = stop_points
		self.msg_to_publish.amount_segments = amount_segments

	#Calculation of the segments
	def calculate_segments(self):
		#Creating List out of Tuple, to be able to transform the List
		self.ranges = []
		for i in range (0, len(self.ranges_received)):
			self.ranges.append(self.ranges_received[i])

		self.angle_increment = self.angle_increment_received

		#Parameter:
		threshold_range = 4.0 #max measurement distance to consider in the calculation 
		min_segment_points = 5 #min number of possible segmentpoints -> supposed to be 3 or greater, because of the following classifcation
	
		start_points_block = []
		stop_points_block = []
		start_points_block_remove = []
		stop_points_block_remove = []
		start_points_block_add = []
		stop_points_block_add = []
		small_start_blocks_remove = []
		small_stop_blocks_remove = []
		start_points_block_remove_outliers = []
		stop_points_block_remove_outliers = []
		euclidian_distance = 0.0
		last_range=len(self.ranges)-1 #Last measurement in the ranges list

		#verify, if the first and the last value of "ranges" is part of a segment
		if (self.ranges[0]<=threshold_range):
			start_points_block.append(0)

		if (self.ranges[last_range]<=threshold_range):
			stop_points_block.append(last_range)

		#split up the measurments closer then "treshhold_range" in blocks
		for i in range (0, len(self.ranges)-1):
			if (self.ranges[i]>threshold_range and self.ranges[i+1]<=threshold_range):
				start_points_block.append(i+1)
			if (self.ranges[i]<=threshold_range and self.ranges[i+1]>threshold_range):
				stop_points_block.append(i)
		
		#verify if any blocks were found in the measurment
		if (len(start_points_block) > 0):
			start_points_block.sort()
			stop_points_block.sort()

			#verify the eukilian distance between stop- and startpoints of the blocks
			for i in range (0, len(start_points_block)-1):
				#calculation of the euclidian_distance of the stop- and startpoints to compare
				euclidian_distance_stop_start = math.sqrt((self.ranges[start_points_block[i+1]]**2)+(self.ranges[stop_points_block[i]]**2)-(2*self.ranges[start_points_block[i+1]]*self.ranges[stop_points_block[i]]*math.cos((start_points_block[i+1]-stop_points_block[i])*self.angle_increment)))
				#calculation of the average distance of the stop- and startpoints to compare
				average_distance_start_stop = (self.ranges[start_points_block[i+1]]+self.ranges[stop_points_block[i]])/2.0
				#calculation of the parameter "threshold_connect" depending on the average distance
				if (average_distance_start_stop <= 1.0):
					threshold_connect = 0.1
				elif (average_distance_start_stop > 1.0):
					threshold_connect = (0.05*average_distance_start_stop)+0.05
				
				#save the start- and stoppoints to remove
				if (euclidian_distance_stop_start <= threshold_connect):
					start_points_block_remove.append(start_points_block[i+1])
					stop_points_block_remove.append(stop_points_block[i])
					
					#change the value of the points inbeween stop and start 
					points_inbetween = (start_points_block[i+1]-1)-stop_points_block[i]			
					if (points_inbetween >= 1):
						point_number_inbetween = 1
						for j in range ((stop_points_block[i]+1), (start_points_block[i+1])):
							self.ranges[j] = self.ranges[stop_points_block[i]]+(point_number_inbetween*((self.ranges[start_points_block[i+1]]-self.ranges[stop_points_block[i]])/(points_inbetween+1)))
							point_number_inbetween = point_number_inbetween + 1
						
			start_points_block.sort()
			stop_points_block.sort()
			
			#remove start and stop points which are part of the same segment
			for i in range (0,len(start_points_block_remove)):
				start_points_block.remove(start_points_block_remove[i])
				stop_points_block.remove(stop_points_block_remove[i])

			#verify the euclidean distances between every point i and i+1 (neighbourpoints) of each segment 
			for i in range (0,len(start_points_block)):
				for j in range (start_points_block[i], stop_points_block[i]):
					#calculation of the euclidian distance of the neighbourpoints
					euclidian_distance_neighbours = math.sqrt((self.ranges[j]**2)+(self.ranges[j+1]**2)-(2*self.ranges[j]*self.ranges[j+1]*math.cos(self.angle_increment)))
					#calculation of the average distance of the neighbourpoints
					average_distance_neighbours = (self.ranges[j]+self.ranges[j+1])/2.0
					#calculation of the parameter "threshold_divide" depending on the average distance
					if (average_distance_neighbours <= 1.0):
						threshold_divide = 0.1
					elif (average_distance_neighbours > 1.0):
						threshold_divide = (0.05*average_distance_neighbours)+0.05
					#safe the new start- and stop points
					if (euclidian_distance_neighbours > threshold_divide):
						stop_points_block_add.append(j)
						start_points_block_add.append(j+1)
				
			#add new start- and stop points to the lists
			for i in range (0,len(start_points_block_add)):
				start_points_block.append(start_points_block_add[i])
				stop_points_block.append(stop_points_block_add[i])
			
			start_points_block.sort()
			stop_points_block.sort()	

			#verify the distance between the first startpoint and the last stoppoint
			if (len(start_points_block) > 1):
				#calculation of the euclidian distance of first start- and the last stoppoint
				euclidian_distance_last_first = math.sqrt((self.ranges[start_points_block[0]]**2)+(self.ranges[stop_points_block[len(stop_points_block)-1]]**2)-(2*self.ranges[start_points_block[0]]*self.ranges[stop_points_block[len(stop_points_block)-1]]*math.cos(self.angle_increment*((start_points_block[0]+360)-(stop_points_block[len(stop_points_block)-1])))))
				#calculation of the parameter "threshold_connect" depending on the average distance of the first start- and the last stoppoint
				average_distance_last_first = (self.ranges[start_points_block[0]]+self.ranges[stop_points_block[len(stop_points_block)-1]])/2.0
				if (average_distance_last_first <= 1.0):
					threshold_connect = 0.1
				elif (average_distance_last_first > 1.0):
					threshold_connect = (0.05*average_distance_last_first)+0.05

				#chance the value of the points inbetween the first start- and the last stoppoint
				point_number_inbetween_first_start_last_stop = 1
				if(euclidian_distance_last_first <= threshold_connect):
					points_inbetween = (start_points_block[0]+last_range)-(stop_points_block[len(stop_points_block)-1])
					if (points_inbetween >= 1  and stop_points_block[len(stop_points_block)-1] != last_range):
						for j in range ((stop_points_block[len(stop_points_block)-1]+1), last_range+1):
							self.ranges[j] = self.ranges[stop_points_block[len(stop_points_block)-1]]+(point_number_inbetween_first_start_last_stop*((self.ranges[start_points_block[0]]-self.ranges[stop_points_block[len(stop_points_block)-1]])/(points_inbetween+1)))
							point_number_inbetween_first_start_last_stop = point_number_inbetween_first_start_last_stop + 1

					if (points_inbetween >= 1  and start_points_block[0] != 0):

						for j in range (0, start_points_block[0]):

							self.ranges[j] = self.ranges[stop_points_block[len(stop_points_block)-1]]+(point_number_inbetween_first_start_last_stop*((self.ranges[start_points_block[0]]-self.ranges[stop_points_block[len(stop_points_block)-1]])/(points_inbetween+1)))
							point_number_inbetween_first_start_last_stop = point_number_inbetween_first_start_last_stop + 1

					#connect the last stoppoint and first startpoint, if they are part of the same segment
					last_first_connect = stop_points_block[0]
					start_points_block.remove(start_points_block[0])
					stop_points_block.remove(stop_points_block[len(stop_points_block)-1])
					stop_points_block.remove(stop_points_block[0])
					stop_points_block.append(last_first_connect)
				
			#removing segments with less points then the parameter "min_segment_points"...
			#...in case, that the last stoppoint and the first startpoint got connected 
			if (start_points_block[len(start_points_block)-1] > stop_points_block[len(stop_points_block)-1]):
				for i in range (0,len(start_points_block)-1):
					number_of_points = abs(start_points_block[i]-stop_points_block[i])
					if (number_of_points < min_segment_points):
						small_start_blocks_remove.append(start_points_block[i])
						small_stop_blocks_remove.append(stop_points_block[i])

				number_of_points = abs(start_points_block[len(start_points_block)-1]-(stop_points_block[len(stop_points_block)-1]+360))
				if (number_of_points < min_segment_points):		
					small_start_blocks_remove.append(start_points_block[len(start_points_block)-1])
					small_stop_blocks_remove.append(stop_points_block[len(stop_points_block)-1])

			#...in case, that the last stoppoint and the first startpoint got not connected
			else:
				for i in range (0,len(start_points_block)):
					number_of_points = abs(start_points_block[i]-stop_points_block[i])
					if (number_of_points < min_segment_points):
						small_start_blocks_remove.append(start_points_block[i])
						small_stop_blocks_remove.append(stop_points_block[i])

			for i in range (0,len(small_start_blocks_remove)):
				start_points_block.remove(small_start_blocks_remove[i])
				stop_points_block.remove(small_stop_blocks_remove[i])
			
		#update the calculated data to publish
		self.update_data_to_publish(start_points_block, stop_points_block, len(start_points_block))


def main():
	#creates an Objekt of the class Sub_Pub
	sub_pub = Sub_Pub()
	rospy.init_node('segmentation_laserscan', anonymous=True)
	rospy.Subscriber("/scan_filtered", LaserScan, sub_pub.callback)	
	
	rospy.spin()


if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
