#! /usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

rospy.init_node("navigation_goal")
pub = rospy.Publisher("/navigation_goal", NavSatFix, queue_size=1)
rate =rospy.Rate(2)
goal = NavSatFix()
goal.latitude = -24.7317075
goal.longitude = 31.9179577

while not rospy.is_shutdown():
	pub.publish(goal)
	print goal
	rate.sleep()
