#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

estado = 0
cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
motion = Twist()

def callbackPose(msg):
	global estado
	position = msg.pose.position
	if position.x < 5.5 and estado == 0:
		estado = 1
		motion.linear.x = +0.5
	if position.x > 6.15 and estado == 1:
		estado = 2
		motion.linear.x = -0.3
		motion.angular.z = -0.2
	if position.y >= 1.1 and estado == 2:
		estado = 3
		motion.angular.z = 0.3
	if position.y >= 1.6 and estado == 3:
		estado = 4
		motion.linear.x = 0.15
		motion.angular.z = 0.3
	if position.y <= 1.5 and estado == 4:
		estado = 5
		motion.linear.x = 0
		motion.angular.z = 0
	cmd.publish(motion)


def callbackSick(msg):
	global estado
	if estado == 5:
		estado = 6

rospy.init_node("cliente")

rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
rospy.Subscriber("/atrv/sick", LaserScan, callbackSick)

rospy.spin() # this will block untill you hit Ctrl+C
