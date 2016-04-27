#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


def callbackPose(poseStamped):
	global estado
	global posX
	global oriZ
	global espacoVazio

	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	
	posX = position.x
	oriZ = orientation.z
	
#	if estado == 0:
#		estado = 1
#		motion.linear.x = +0.5
#	if espacoVazio > 2 and estado == 1:
#		estado = 2
#		motion.linear.x = -0.3
#		motion.angular.z = -0.2
#	if position.y >= 13.1 and estado == 2:
#		estado = 3
#		motion.angular.z = 0.3
#	if position.y >= 13.6 and estado == 3:
#		estado = 4
#		motion.linear.x = 0.15
#		motion.angular.z = 0.3
#	if oriZ >= -0.05 and oriZ <= 0.05 and estado == 4:
#		estado = 5
#		motion.linear.x = 0
#		motion.angular.z = 0

	cmd.publish(motion)


def callbackScan(laserScan):
	global iniX
	global distX
	global espacoVazio
	global estadoCont

	point_cloud = laser_projector.projectLaser(laserScan)
	minY = 0 
	maxY = 0 
	minX = 0 
	maxX = 0 
	somaX = 0
	somaY = 0
	pontos = 0

	# calcula os pontos mínimo e máximo de detecção do laser
	for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
		x = p[0]
		y = p[1]
		if x < 0:
			x = 0
		if y < 0:
			y = 0
		if minX > x:
			minX = x 
		if maxX < x:
			maxX = x 
		if minY > y:
			minY = y 
		if maxY < y:
			maxY = y 
		somaX = somaX + x
		somaY = somaY + y
		pontos = pontos + 1

		rospy.loginfo("x: %.2f y: %.2f" %(x, y))		

	mediaY = (somaY)/pontos	
	mediaX = (somaX)/pontos	
	pond = mediaX-mediaY

	rospy.loginfo("minY: %.1f maxY: %.1f mediaY: %.1f" %(minY, maxY, mediaY))	
	rospy.loginfo("minX: %.1f maxX: %.1f mediaX: %.1f" %(minX, maxX, mediaX))	
	rospy.loginfo("mediaX-mediaY: %.2f" %(pond))	

	# Se o laser não detectar obstáculo, inicia a contagem do espaço vazio
	if mediaY < -0.1 and estadoCont == 0:
		estadoCont = 1	
		iniX = posX
	# Se o laser detectar obstáculo, finaliza a contagem do espaço vazio
	elif mediaY > 0.1 and estadoCont == 1:
		estadoCont = 0
		espacoVazio = distX
	# Se o estadoCont == 1, há um espaço vazio sendo mensurado
	elif estadoCont == 1:
		distX = posX - iniX
		
	#rospy.loginfo("oriZ: %.1f media: %.1f dist: %.1f esp: %.1f" %(oriZ, media, distX, espacoVazio))


cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
pos = rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
sca = rospy.Subscriber("/atrv/sick", LaserScan, callbackScan)


estado = 0
estadoCont = 0
posX = 0
iniX = 0
distX = 0
oriZ = 0
espacoVazio = 0

laser_projector = LaserProjection()
motion = Twist()

rospy.init_node("cliente")
rospy.spin() # this will block untill you hit Ctrl+C

