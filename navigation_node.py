#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import String
import math


speed = Twist()

goal = [0,0]
goal_latitude = 0
goal_longitude = 0
goal_sector = 0
distance_to_goal = 0
angle_to_goal = 0

current_sector = 0
current_latitude = 0.1
previous_latitude = 0
current_longitude = 0.1
previous_longitude = 0
current_angle = 0

current_target = [0,0,0]
current_target_longitude = 0
current_target_latitude = 0
angle_to_current_target = 0
current_target_sector = 0
distance_to_current_target = 0

next_landmark = [0,0,0]
next_landmark_longitude = 0
next_landmark_latitude = 0
landmark_longitude = 0
landmark_latitude = 0

next_target = [0,0,0]

landmark_number = 0

turning_speed = 0.2

obstacle = "false"


#This is where all landmarks are registered. Each landmark contains three numbers. The first number is the longitude, the second one is the latitude. The third one can be either 0 or 1 and it indicates whether the landmark has already been visited (0 means that it hasn't been visited, whereas 1 means that this landmark has already been reached)
landmarks = [[31.9274479,-24.7299579,0],[31.9276686,-24.7306168,0],[31.9276650,-24.7310060,0],[31.9275000,-24.7310094,0],[31.9273904,-24.7310110,0],[31.9270345,-24.7309452,0],[31.9267524,-24.7308552,0],[31.9267387,-24.7308486,0],[31.9265179,-24.7306816,0],[31.9261697,-24.7306427,0],[31.9259927,-24.7304856,0],[31.9258447,-24.7302813,0],[31.9257250,-24.7301056,0],[31.9253106,-24.7298811,0],[31.9248905,-24.7296668,0],[31.9243954,-24.7297847,0],[31.9239940,-24.7299331,0],[31.9236519,-24.7303116,0],[31.9228232,-24.7305890,0],[31.9219353,-24.7306262,0],[31.9206414,-24.7310321,0],[31.9198807,-24.7304280,0],[31.9189338,-24.7313462,0],[31.9183208,-24.7314191,0],[31.9179148,-24.7314614,0],[31.9194823,-24.7309685,0],[31.9178805,-24.7316399,0],[31.926180,-24.7321502,0],[31.9267149,-24.7327142,0],[31.9251874,-24.7320158,0],[0,0,0]]

#This function selects the next landmark to drive to among all the landmarks included in the matrix.
def select_next_landmark():
	global landmarks
	global goal
	global goal_longitude
	global goal_latitude
	global next_landmark_longitude
	global next_landmark_latitude
	global distance_to_goal
	global landmark_longitude
	global landmark_latitude
	global landmark_number
	global next_landmark

	counter = 0
	distance_to_landmark = 1000
	smallest_distance = 1000
	

	for landmark in landmarks:
		landmark_longitude = landmark[0]
		landmark_latitude = landmark[1]

		counter = counter + 1			

		distance_to_landmark = math.sqrt((landmark_longitude-current_longitude)*(landmark_longitude-current_longitude)+(landmark_latitude-current_latitude)*(landmark_latitude-current_latitude))

		distance_from_landmark_to_goal = math.sqrt((goal_longitude-landmark_longitude)*(goal_longitude-landmark_longitude)+(goal_latitude-landmark_latitude)*(goal_latitude-landmark_latitude))

		if 7*distance_to_landmark + distance_from_landmark_to_goal < smallest_distance and landmark[2] == 0:
			smallest_distance = 7*distance_to_landmark + distance_from_landmark_to_goal
			next_landmark = landmark
			landmark_number = counter - 1			

	next_landmark_longitude = next_landmark[0]
	next_landmark_latitude = next_landmark[1]

	if landmark_number == len(landmarks) - 1:
		print "The last landmark has been reached. Driving to the goal"
	
	print "The next landmark is landmark number %d: [%f,%f]" %(landmark_number, next_landmark_longitude, next_landmark_latitude)

	return next_landmark

#This function is the responsible of driving the Husky
def drive_to_next_target(current_target):
	global current_longitude
	global current_latitude
	global angle_to_current_target
	global current_angle
	global current_sector
	global current_target_sector
	global distance_to_current_target
	global next_target
	global landmark_number

	current_target_longitude = current_target[0]
	current_target_latitude = current_target[1]


	#Calculate the angle_to_current_target

	#First angular sector
	if current_target_longitude > current_longitude and current_target_latitude > current_latitude:
		angle_to_current_target = math.atan(abs(current_target_latitude - current_latitude)/abs(current_target_longitude - current_longitude))
		#print "angle to current_target (first sector) = %f" %(angle_to_current_target)
		current_target_sector = 1
	

	#Second angular sector
	elif current_target_longitude < current_longitude and current_target_latitude > current_latitude:
		angle_to_current_target = 3.1416 - math.atan(abs(current_target_latitude - current_latitude)/abs(current_target_longitude - current_longitude))
		#print "angle to current_target (second sector) = %f" %(angle_to_current_target)
		current_target_sector = 2
	

	#Third angular sector
	elif current_target_longitude < current_longitude  and current_target_latitude < current_latitude :
		angle_to_current_target = 3.1416 + math.atan(abs(current_target_latitude - current_latitude)/abs(current_target_longitude - current_longitude))
		#print "angle to current_target (third sector) = %f" %(angle_to_current_target)
		current_target_sector = 3
		

	#Fourth angular sector
	elif current_target_longitude > current_longitude and current_target_latitude < current_latitude:
		angle_to_current_target = 6.2832 - math.atan(abs(current_target_latitude - current_latitude)/abs(current_target_longitude - current_longitude))
		#print "angle to current_target (fourth sector) = %f" %(angle_to_current_target)
		current_target_sector = 4



	#Drive to next_target 
	distance_to_current_target = math.sqrt((current_target_longitude-current_longitude)*(current_target_longitude-current_longitude)+(current_target_latitude-current_latitude)*(current_target_latitude-current_latitude))
	print "distance to current target = %f" %(distance_to_current_target)



	if (distance_to_current_target > 0.00001):
		
		#Change orientation to match path to current_target

		if (abs(angle_to_current_target - current_angle) > 0.2 or current_sector != current_target_sector) and obstacle == "false":

			print "Driving to target [%f, %f]" %(current_target_longitude, current_target_latitude)

			if current_sector == 1:
				if current_target_sector == 1:
					if angle_to_current_target < current_angle:
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 2:
					#turn counterclockwise 
					speed.angular.z = turning_speed
				if current_target_sector == 3:
					if angle_to_current_target > (current_angle + 3.1416):
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 4:
					#turn clockwise
					speed.angular.z = -turning_speed
			if current_sector == 2:
				if current_target_sector == 1:
					#turn clockwise
					speed.angular.z = -turning_speed
				if current_target_sector == 2:
					if angle_to_current_target < current_angle:
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 3:
					#turn counterclockwise 
					speed.angular.z = turning_speed
				if current_target_sector == 4:
					if angle_to_current_target < (current_angle + 3.1416):
						#turn counterclockwise 
						speed.angular.z = turning_speed
					else:
						#turn clockwise
						speed.angular.z = -turning_speed
			if current_sector == 3:
				if current_target_sector == 1:
					if angle_to_current_target > (current_angle - 3.1416):
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 2:
					#turn clockwise
					speed.angular.z = -turning_speed
				if current_target_sector == 3:
					if angle_to_current_target < current_angle:
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 4:
					#turn counterclockwise 
					speed.angular.z = turning_speed
			if current_sector == 4:
				if current_target_sector == 1:
					#turn counterclockwise 
					speed.angular.z = turning_speed
				if current_target_sector == 2:
					if angle_to_current_target > (current_angle - 3.1416):
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed
				if current_target_sector == 3:
					#turn clockwise
					speed.angular.z = -turning_speed
				if current_target_sector == 4:
					if angle_to_current_target < current_angle:
						#turn clockwise
						speed.angular.z = -turning_speed
					else:
						#turn counterclockwise 
						speed.angular.z = turning_speed


		else:
			#do not turn
			speed.angular.z = 0
	


		#obstacle avoidance
		if obstacle == "left":
			speed.linear.x = 0
			speed.angular.z = -turning_speed
		elif obstacle == "right":
			speed.linear.x = 0
			speed.angular.z = turning_speed

		elif obstacle == "critical":
			speed.linear.x = 0
			speed.angular.z = 0
				
		else:
			speed.linear.x = 200

	else:
		speed.linear.x = 0
		print "Target reached"
		landmarks[landmark_number][2] = 1
		if landmark_number == len(landmarks) - 1:
			print "Goal reached"
			#The node stops when the goal is reached
			reason = 1
			rospy.signal_shutdown(reason)
		else:
			next_target = select_next_landmark()		

#This function receives the goal from the goal publisher
def callback_goal(msg):
	global goal
	global goal_longitude
	global goal_latitude
	global landmarks

	goal_longitude = msg.longitude
	goal_latitude = msg.latitude

	landmarks[len(landmarks)-1][0] = goal_longitude
	landmarks[len(landmarks)-1][1] = goal_latitude	


#This function calls the navigation functions and finds out about the orientation of the Husky
def callback_navigation(msg):
	global current_latitude
	global previous_latitude
	global current_longitude
	global previous_longitude
	global current_angle
	global current_sector
	global angle_to_goal
	global goal_latitude
	global goal_longitude
	global goal_sector
	global turning_speed
	global distance_to_goal	
	global landmarks
	global goal
	global next_landmark
	global current_target
	global current_target_longitude
	global current_target_latitude
	global angle_to_current_target
	global current_target_sector
	global distance_to_current_target

	previous_latitude = current_latitude
	previous_longitude = current_longitude

	current_latitude = msg.latitude
	current_longitude = msg.longitude

	next_landmark = select_next_landmark()

	drive_to_next_target(next_landmark)

	
	#Calculate the current angle 

	#First sector
	if current_longitude > previous_longitude and current_latitude > previous_latitude:
		current_angle = math.atan(abs(current_latitude - previous_latitude)/abs(current_longitude - previous_longitude))
		#print "current angle (first sector) = %f" %(current_angle)
		current_sector = 1


	#Second sector
	elif current_longitude < previous_longitude and current_latitude > previous_latitude:
		current_angle = 3.1416 - math.atan(abs(current_latitude - previous_latitude)/abs(current_longitude - previous_longitude))
		#print "current angle (second sector) = %f" %(current_angle)
		current_sector = 2
	

	#Third sector
	elif current_longitude < previous_longitude and current_latitude < previous_latitude:
		current_angle = 3.1416 + math.atan(abs(current_latitude - previous_latitude)/abs(current_longitude - previous_longitude))
		#print "current angle (third sector) = %f" %(current_angle)
		current_sector = 3
	

	#Fourth sector
	elif current_longitude > previous_longitude and current_latitude < previous_latitude:
		current_angle = 6.2832 - math.atan(abs(current_latitude - previous_latitude)/abs(current_longitude - previous_longitude))
		#print "current angle (fourth sector) = %f" %(current_angle)
		current_sector = 4


#This function receives the readings from the sensors used to avoid obstacles
def callback_obstacle_avoidance(msg):
	global obstacle
	global next_landmark
	global landmarks
	global landmark_number

	if msg.ranges[320] < 2.5 or msg.ranges[250] < 2.5 or msg.ranges[510] < 2.5:
		if msg.ranges[470] <= msg.ranges[250]:
			obstacle = "left"
		else:
			obstacle = "right"
		
	else:
		obstacle = "false"

#This function receives the readings from the gyroscope and accelerometer
def callback_imu(msg):
	global obstacle

	if abs(msg.orientation.x) > 0.2 or abs(msg.orientation.y) > 0.15 or abs(msg.linear_acceleration.x) > 10:
		obstacle = "critical"
		pub_imu.publish("Help! I am in trouble!")
	
	
	
#Initialization of the node
rospy.init_node("navigation_node")


#Subscribers
sub_navigation = rospy.Subscriber("/navsat/fix", NavSatFix, callback_navigation)

sub_goal = rospy.Subscriber("/navigation_goal", NavSatFix, callback_goal)

sub_obstacle_avoidance = rospy.Subscriber("/rrbot/laser/scan", LaserScan, callback_obstacle_avoidance)

sub_imu = rospy.Subscriber("/imu/data", Imu, callback_imu)


#Publishers
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

pub_imu = rospy.Publisher("/emergency_call", String, queue_size = 1)

rate =rospy.Rate(2)


while not rospy.is_shutdown():
	pub.publish(speed)
	rate.sleep()
