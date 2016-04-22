#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


def callbackPose(msg):
	global estado
	global posX
	global ultX

	position = msg.pose.position
	posX = position.x
	
#	if estado == 0:
#		estado = 1
#		motion.linear.x = +0.5
#	if ultX > 1.7 and estado == 1:
#		estado = 2
#		motion.linear.x = -0.3
#		motion.angular.z = -0.2
#	if position.y >= 1.1 and estado == 2:
#		estado = 3
#		motion.angular.z = 0.3
#	if position.y >= 1.6 and estado == 3:
#		estado = 4
#		motion.linear.x = 0.15
#		motion.angular.z = 0.3
#	if position.y <= 1.5 and estado == 4:
#		estado = 5
#		motion.linear.x = 0
#		motion.angular.z = 0

	cmd.publish(motion)


def callbackScan(scan):
	global iniX
	global distX
	global ultX
	global estadoPos

	point_cloud = laser_projector.projectLaser(scan)
	minY = 0 
	maxY = 0 

	for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
		if p[1] < minY:
			minY = p[1] 
		if p[1] > maxY:
			maxY = p[1] 

	if (minY+maxY)/2 < -0.1 and estadoPos == 0:
		estadoPos = 1	
		iniX = posX
	elif (minY+maxY)/2 > 0.1 and estadoPos == 1:
		estadoPos = 0
		ultX = distX
	elif estadoPos == 1:
		distX = posX - iniX
		
	#rospy.loginfo("posX: %.1f y: %.1f x: %.1f - %.1f" %(posX, distY, distX, ultX))
	rospy.loginfo("min: %.1f	max: %.1f	%.1f	%.1f" %(minY, maxY, distX, ultX))


cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
pos = rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
sca = rospy.Subscriber("/atrv/sick", LaserScan, callbackScan)

estado = 0
estadoPos = 0
posX = 0
iniX = 0
distX = 0
ultX = 0

laser_projector = LaserProjection()
motion = Twist()

rospy.init_node("cliente")
rospy.spin() # this will block untill you hit Ctrl+C

