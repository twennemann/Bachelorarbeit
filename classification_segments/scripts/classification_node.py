#!/usr/bin/env python
#***************************************************************************

#PURPOSE: Classification of the segmentated Data of an RPLIDAR scan 
#LIBARIES: | rospy | math |
#MESSAGE TYPES: | segmentation_msg (costum) | objects_msg (costum)
#LAST MODIFIED: Tim Wennemann 30.07.2020

#***************************************************************************
import rospy
import math
from segmentation_laserscan.msg import segmentation_msg
from classification_segments.msg import objects_msg

class Sub_Pub(object):
	#Initialize an object of the class "Sub_Pub"
	def __init__(self):
		self.ranges_received = []
		self.angle_increment_received = 0
		self.amount_segments_received = 0
		self.start_points_received = []
		self.stop_points_received = []
		#self.amount_objects = 0
		self.pub = rospy.Publisher('objects', objects_msg, queue_size=10) 
		self.msg_to_publish = objects_msg()
		
	#Receiving the subscribtion-data, and publication of the calculated objects
	def callback(self, msg):
		self.ranges_received = msg.ranges 
		self.angle_increment_received = msg.angle_increment # 0.017453
		self.amount_segments_received = msg.amount_segments
		self.start_points_received = msg.start_points
		self.stop_points_received = msg.stop_points
		self.calculate_classification()
		self.pub.publish(self.msg_to_publish)
		
	#Update data to publish
	def update_data_to_publish(self, lines_start_x, lines_start_y, lines_stop_x, lines_stop_y, lines_closest_point_x, lines_closest_point_y, lines_middle_point_x, lines_middle_point_y, circles_center_x, circles_center_y, circles_radius, rectangles_corner_start_x, rectangles_corner_start_y, rectangles_corner_max_distance_x, rectangles_corner_max_distance_y, rectangles_corner_stop_x, rectangles_corner_stop_y, rectangles_corner_max_distance_mirror_x, rectangles_corner_max_distance_mirror_y, rectangles_center_x, rectangles_center_y):
		self.msg_to_publish.lines_start_x = lines_start_x
		self.msg_to_publish.lines_start_y = lines_start_y
		self.msg_to_publish.lines_stop_x = lines_stop_x
		self.msg_to_publish.lines_stop_y = lines_stop_y
		self.msg_to_publish.lines_closest_point_x = lines_closest_point_x
		self.msg_to_publish.lines_closest_point_y = lines_closest_point_y
		self.msg_to_publish.lines_middle_point_x = lines_middle_point_x
		self.msg_to_publish.lines_middle_point_y = lines_middle_point_y
		self.msg_to_publish.circles_center_x = circles_center_x
		self.msg_to_publish.circles_center_y = circles_center_y
		self.msg_to_publish.circles_radius = circles_radius
		self.msg_to_publish.rectangles_corner_start_x = rectangles_corner_start_x
		self.msg_to_publish.rectangles_corner_start_y = rectangles_corner_start_y
		self.msg_to_publish.rectangles_corner_max_distance_x = rectangles_corner_max_distance_x
		self.msg_to_publish.rectangles_corner_max_distance_y = rectangles_corner_max_distance_y
		self.msg_to_publish.rectangles_corner_stop_x = rectangles_corner_stop_x
		self.msg_to_publish.rectangles_corner_stop_y = rectangles_corner_stop_y
		self.msg_to_publish.rectangles_corner_max_distance_mirror_x = rectangles_corner_max_distance_mirror_x
		self.msg_to_publish.rectangles_corner_max_distance_mirror_y = rectangles_corner_max_distance_mirror_y
		self.msg_to_publish.rectangles_center_x = rectangles_center_x
		self.msg_to_publish.rectangles_center_y = rectangles_center_y

	#Calculation of the classification
	def calculate_classification(self):
		self.ranges = self.ranges_received
		self.angle_increment = self.angle_increment_received
		self.amount_segments = self.amount_segments_received
		self.start_points = self.start_points_received
		self.stop_points = self.stop_points_received
		start_points_block = self.start_points
		stop_points_block = self.stop_points
		ranges = self.ranges
		last_range=len(self.ranges)-1
		
		#Parameter:
		min_angle_maxstart_maxstop_rectangle = 80.0 #min angle to be classified as a rectangle
		max_angle_maxstart_maxstop_rectangle = 100.0 #max angle to be classified as a rectangle
		circle_center_faktor = 1.9 #factor, to calculade the position of the circle_center_point
		percent_greater_angle_circle = 0.80 #percent of greater angles to be classified as a circle

		segments_x_values = []
		segments_y_values = []
		segments_ranges = []
		lines_start_x = []
		lines_start_y = []
		lines_stop_x = []
		lines_stop_y = []
		lines_closest_point_x = []
		lines_closest_point_y = []
		lines_middle_point_x = []
		lines_middle_point_y = []
		circles_center_x = []
		circles_center_y = []
		circles_radius = []
		rectangles_corner_start_x = []
		rectangles_corner_start_y = []
		rectangles_corner_max_distance_x = []
		rectangles_corner_max_distance_y = []
		rectangles_corner_stop_x = []
		rectangles_corner_stop_y = []
		rectangles_corner_max_distance_mirror_x = []
		rectangles_corner_max_distance_mirror_y = []
		rectangles_center_x = []
		rectangles_center_y = []
		
		#verify, if any segments got measured
		if (self.amount_segments > 0):
			#create segmentlists and convert the measurmentpoints into cartesian coordinates...
			#...in case, that there is a segment on the x-axis
			if (start_points_block[len(start_points_block)-1] > stop_points_block[len(stop_points_block)-1]):
				for i in range (0,len(start_points_block)-1):
					segment_points_x = []
					segment_points_y = []
					segment_points_range =[]
					for j in range (start_points_block[i], stop_points_block[i]+1):
						x_value = ranges[j]*math.cos(j*self.angle_increment)
						y_value = ranges[j]*math.sin(j*self.angle_increment)
						segment_points_x.append(x_value)
						segment_points_y.append(y_value)
						segment_points_range.append (ranges[j])
					segments_x_values.append(segment_points_x)
					segments_y_values.append(segment_points_y)
					segments_ranges.append(segment_points_range)
				segment_points_x = []
				segment_points_y = []
				segment_points_range =[]

				for i in range (start_points_block[len(start_points_block)-1], last_range+1):
					x_value = ranges[i]*math.cos(i*self.angle_increment)
					y_value = ranges[i]*math.sin(i*self.angle_increment)
					segment_points_x.append(x_value)
					segment_points_y.append(y_value)
					segment_points_range.append (ranges[i])

				for i in range (0, stop_points_block[len(stop_points_block)-1]+1):
					x_value = ranges[i]*math.cos(i*self.angle_increment)
					y_value = ranges[i]*math.sin(i*self.angle_increment)
					segment_points_x.append(x_value)
					segment_points_y.append(y_value)
					segment_points_range.append (ranges[i])
				segments_x_values.append(segment_points_x)
				segments_y_values.append(segment_points_y)
				segments_ranges.append(segment_points_range)

			#... in case, that there is no segment on the x-axis
			else:	
				for i in range (0,len(start_points_block)):
					segment_points_x = []
					segment_points_y = []
					segment_points_range =[]

					for j in range (start_points_block[i], stop_points_block[i]+1):
						x_value = ranges[j]*math.cos(j*self.angle_increment)
						y_value = ranges[j]*math.sin(j*self.angle_increment)
						segment_points_x.append(x_value)
						segment_points_y.append(y_value)
						segment_points_range.append (ranges[j])
					segments_x_values.append(segment_points_x)
					segments_y_values.append(segment_points_y)
					segments_ranges.append(segment_points_range)

			#classification of each segment i
			for i in range (0,len(segments_x_values)):

				x_value_start = segments_x_values[i][0]					
				x_value_stop = segments_x_values[i][len(segments_x_values[i])-1]				
				y_value_start = segments_y_values[i][0]				
				y_value_stop = segments_y_values[i][len(segments_y_values[i])-1]
				max_distance_start_stop_line = 0.0
				x_value_max_distance_point = 0.0
				y_value_max_distance_point = 0.0
				point_number = 0

				#calculation of the distance of each segmentpoint j to the pq-vector 
				for j in range (1, len(segments_x_values[i])-1):					
					x_value_j = segments_x_values[i][j]					
					y_value_j = segments_y_values[i][j]					
					lambda_value = (-((x_value_start-x_value_j)*(x_value_stop-x_value_start))-((y_value_start-y_value_j)*(y_value_stop-y_value_start))) / (((x_value_stop-x_value_start)**2)+((y_value_stop-y_value_start)**2))					
					point_distance_to_line = math.sqrt(((x_value_start-x_value_j+(lambda_value*x_value_stop)-(lambda_value*x_value_start))**2)+((y_value_start-y_value_j+(lambda_value*y_value_stop)-(lambda_value*y_value_start))**2))

					#Saving the max_distance_point
					if (max_distance_start_stop_line < point_distance_to_line):
						max_distance_start_stop_line = point_distance_to_line
						x_value_max_distance_point = x_value_j
						y_value_max_distance_point = y_value_j	
						point_number = j
				
				#calculation of the parameter "threshold_lines" depending on the distance of the middle point inbetween start- and stoppoint
				distance_middle_point = (math.sqrt((((x_value_start+x_value_stop)/2)**2)+(((y_value_start+y_value_stop)/2)**2)))
				if (distance_middle_point <= 1.0):
					threshold_lines = 0.02
				elif (distance_middle_point > 1.0):
					threshold_lines = (0.01*distance_middle_point)+0.01	

				#condition for the classification as a line
				if (max_distance_start_stop_line < threshold_lines):
					line_middle_point_x = (x_value_start+x_value_stop)/2
					line_middle_point_y = (y_value_start+y_value_stop)/2
					closest_sensor_line_point_distance = 12.0
					for j in range (0, len(segments_x_values[i])):
						x_value_j = segments_x_values[i][j]
						y_value_j = segments_y_values[i][j]
						distance_sensor_line_point = math.sqrt((x_value_j**2)+(y_value_j**2))
						if(closest_sensor_line_point_distance > distance_sensor_line_point):
							closest_sensor_line_point_distance = distance_sensor_line_point
							line_closest_point_x = x_value_j
							line_closest_point_y = y_value_j
					
					#saving the line-coordinades to publish
					lines_start_x.append(x_value_start)
					lines_start_y.append(y_value_start)
					lines_stop_x.append(x_value_stop)
					lines_stop_y.append(y_value_stop)
					lines_middle_point_x.append(line_middle_point_x)
					lines_middle_point_y.append(line_middle_point_y)
					lines_closest_point_x.append(line_closest_point_x)
					lines_closest_point_y.append(line_closest_point_y)

				else:
					
					#calculation of the angle between the vector ->(max_distance_point,startpoint) and the vector ->(max_distance_point,stop)Berechnung des Winkels zwischen Vector max_distance_point start und Vector max_distance_point stop
					angle_maxstart_maxstop = math.degrees(math.acos((((x_value_start-x_value_max_distance_point)*(x_value_stop-x_value_max_distance_point))+((y_value_start-y_value_max_distance_point)*(y_value_stop-y_value_max_distance_point)))/((math.sqrt(((x_value_start-x_value_max_distance_point)**2)+((y_value_start-y_value_max_distance_point)**2)))*(math.sqrt(((x_value_stop-x_value_max_distance_point)**2)+((y_value_stop-y_value_max_distance_point)**2))))))

					#calculation of the angle between the vector ->(startpoint,stoppoint) and the vector ->(startpoint, max_distance_point)
					angle_startstop_startmax = math.degrees(math.acos((((x_value_stop-x_value_start)*(x_value_max_distance_point-x_value_start))+((y_value_stop-y_value_start)*(y_value_max_distance_point-y_value_start)))/((math.sqrt(((x_value_stop-x_value_start)**2)+((y_value_stop-y_value_start)**2)))*(math.sqrt(((x_value_max_distance_point-x_value_start)**2)+((y_value_max_distance_point-y_value_start)**2))))))
				
					amount_points_in_between = len(segments_x_values[i])-3					
					amount_points_greater_angle = 0.0
					max_distance_pmax_line = 0.0
					x_value_pmax_distance_point = 0.0
					y_value_pmax_distance_point = 0.0

					#verify the segmentpoints inbetween the startpoint and max_distance_point
					for j in range (1, point_number):
						x_value_j = segments_x_values[i][j]						
						y_value_j = segments_y_values[i][j]
					
						#calculation of the angle between the vector ->(startpoint,stoppoint) and the vector ->(startpoint, j)
						angle_startstop_startj = math.degrees(math.acos((((x_value_stop-x_value_start)*(x_value_j-x_value_start))+((y_value_stop-y_value_start)*(y_value_j-y_value_start)))/((math.sqrt(((x_value_stop-x_value_start)**2)+((y_value_stop-y_value_start)**2)))*(math.sqrt(((x_value_j-x_value_start)**2)+((y_value_j-y_value_start)**2))))))

						#calculate the distances for rectangle classification
						lambda_value = (-((x_value_start-x_value_j)*(x_value_max_distance_point-x_value_start))-((y_value_start-y_value_j)*(y_value_max_distance_point-y_value_start))) / (((x_value_max_distance_point-x_value_start)**2)+((y_value_max_distance_point-y_value_start)**2))
						pmax_point_distance_to_line = math.sqrt(((x_value_start-x_value_j+(lambda_value*x_value_max_distance_point)-(lambda_value*x_value_start))**2)+((y_value_start-y_value_j+(lambda_value*y_value_max_distance_point)-(lambda_value*y_value_start))**2))
						
						#count the number of greater angles for circle classificaiton
						if (angle_startstop_startmax+2 < angle_startstop_startj):
							amount_points_greater_angle = amount_points_greater_angle + 1

						#save up the point which has the biggest distance to the start-max_distance_point line
						if (max_distance_pmax_line < pmax_point_distance_to_line):
							max_distance_pmax_line = pmax_point_distance_to_line
							x_value_pmax_distance_point = x_value_j
							y_value_pmax_distance_point = y_value_j	
							
					#calculation of the angle between the vector ->(stoppoint,startpoint) and the vector ->(stoppoint, max_distance_point)
					angle_stopstart_stopmax = math.degrees(math.acos((((x_value_start-x_value_stop)*(x_value_max_distance_point-x_value_stop))+((y_value_start-y_value_stop)*(y_value_max_distance_point-y_value_stop)))/((math.sqrt(((x_value_start-x_value_stop)**2)+((y_value_start-y_value_stop)**2)))*(math.sqrt(((x_value_max_distance_point-x_value_stop)**2)+((y_value_max_distance_point-y_value_stop)**2))))))

					#verify the segmentpoints inbetween the max_distance_point and the stop_point
					for j in range (point_number+1, len(segments_x_values[i])-1):
						x_value_j = segments_x_values[i][j]
						y_value_j = segments_y_values[i][j]
						
						#calculation of the angle between the vector ->(stoppoint,startpoint) and the vector ->(stoppoint, j)
						angle_stopstart_stopj = math.degrees(math.acos((((x_value_start-x_value_stop)*(x_value_j-x_value_stop))+((y_value_start-y_value_stop)*(y_value_j-y_value_stop)))/((math.sqrt(((x_value_start-x_value_stop)**2)+((y_value_start-y_value_stop)**2)))*(math.sqrt(((x_value_j-x_value_stop)**2)+((y_value_j-y_value_stop)**2))))))
						
						#calculate the distances for rectangle classification
						lambda_value = (-((x_value_max_distance_point-x_value_j)*(x_value_stop-x_value_max_distance_point))-((y_value_max_distance_point-y_value_j)*(y_value_stop-y_value_max_distance_point))) / (((x_value_stop-x_value_max_distance_point)**2)+((y_value_stop-y_value_max_distance_point)**2))
						pmax_point_distance_to_line = math.sqrt(((x_value_max_distance_point-x_value_j+(lambda_value*x_value_stop)-(lambda_value*x_value_max_distance_point))**2)+((y_value_max_distance_point-y_value_j+(lambda_value*y_value_stop)-(lambda_value*y_value_max_distance_point))**2))

						#count the number of greater angles for circle classificaiton
						if (angle_stopstart_stopmax+2 < angle_stopstart_stopj):
							amount_points_greater_angle = amount_points_greater_angle + 1	

						#save the point pmax, which has the biggest distance to the Speichern des Punktes Pmax, welcher den groessten Abstand zwischen Geraden und Punkt besitzt
						if (max_distance_pmax_line < pmax_point_distance_to_line):
							max_distance_pmax_line = pmax_point_distance_to_line
							x_value_pmax_distance_point = x_value_j
							y_value_pmax_distance_point = y_value_j	

					#calculation of the parameter "threshold_rectangle" depending on the distance of the middle point inbetween start- and stoppoint
					if (distance_middle_point <= 1.0):
						threshold_rectangle = 0.02
					elif (distance_middle_point > 1.0):
						threshold_rectangle = 0.02*distance_middle_point
						
					#condition for the classification as circle
					if (amount_points_greater_angle >= percent_greater_angle_circle*amount_points_in_between):
						circle_center_x = x_value_max_distance_point+(circle_center_faktor*(((x_value_start+x_value_stop)/2)-x_value_max_distance_point))
						circle_center_y = y_value_max_distance_point+(circle_center_faktor*(((y_value_start+y_value_stop)/2)-y_value_max_distance_point))
						circle_radius = math.sqrt(((circle_center_x-x_value_max_distance_point)**2)+((circle_center_y-y_value_max_distance_point)**2))
						#calculation for the room/object classification
						distance_sensor_circle_center = math.sqrt((circle_center_x**2)+(circle_center_y**2))
						distance_sensor_max_distance_point = math.sqrt((x_value_max_distance_point**2)+(y_value_max_distance_point**2))

						#save up the circle values to publish
						if(distance_sensor_circle_center > distance_sensor_max_distance_point):
							circles_center_x.append(circle_center_x)
							circles_center_y.append(circle_center_y)
							circles_radius.append(circle_radius)
					
					#condition for the classification as circle
					elif(max_distance_pmax_line < threshold_rectangle and min_angle_maxstart_maxstop_rectangle < angle_maxstart_maxstop < max_angle_maxstart_maxstop_rectangle):
						rectangle_center_x = (x_value_start+x_value_stop)/2	
						rectangle_center_y = (y_value_start+y_value_stop)/2	
						rectangle_corner_4_x = x_value_start + x_value_stop - x_value_max_distance_point
						rectangle_corner_4_y = y_value_start + y_value_stop - y_value_max_distance_point
						#calculation for the room/object classification
						distance_sensor_rectangle_center = math.sqrt((rectangle_center_x**2)+(rectangle_center_y**2))
						distance_sensor_max_distance_point = math.sqrt((x_value_max_distance_point**2)+(y_value_max_distance_point**2))

						#save up the rectangle values to publish
						if(distance_sensor_rectangle_center > distance_sensor_max_distance_point):
							rectangles_corner_start_x.append(x_value_start)
							rectangles_corner_start_y.append(y_value_start)
							rectangles_corner_max_distance_x.append(x_value_max_distance_point)
							rectangles_corner_max_distance_y.append(y_value_max_distance_point)
							rectangles_corner_stop_x.append(x_value_stop)
							rectangles_corner_stop_y.append(y_value_stop)
							rectangles_corner_max_distance_mirror_x.append(rectangle_corner_4_x)
							rectangles_corner_max_distance_mirror_y.append(rectangle_corner_4_y)
							rectangles_center_x.append(rectangle_center_x)
							rectangles_center_y.append(rectangle_center_y)
		#update the calculated data to publish
		self.update_data_to_publish(lines_start_x, lines_start_y, lines_stop_x, lines_stop_y, lines_closest_point_x, lines_closest_point_y, lines_middle_point_x, lines_middle_point_y, circles_center_x, circles_center_y, circles_radius, rectangles_corner_start_x, rectangles_corner_start_y, rectangles_corner_max_distance_x, rectangles_corner_max_distance_y, rectangles_corner_stop_x, rectangles_corner_stop_y, rectangles_corner_max_distance_mirror_x, rectangles_corner_max_distance_mirror_y, rectangles_center_x, rectangles_center_y)
	

def main():
	#creates an Objekt of the class Sub_Pub
	sub_pub = Sub_Pub()
	rospy.init_node('classification_segments', anonymous=True)
	rospy.Subscriber("/segmentation", segmentation_msg, sub_pub.callback)	
	
	rospy.spin()


if __name__ == '__main__':
    try:
       	main()
    except rospy.ROSInterruptException:
        pass
