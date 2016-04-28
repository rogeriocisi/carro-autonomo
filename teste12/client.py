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
	global obstEsq
	global obstTraz

	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	
	posX = position.x
	oriZ = orientation.z
	
	if estado == 0:
		rospy.loginfo("entrou - estado: %d" %(estado))
		estado = 1
		motion.linear.x = +0.5
	if espacoVazio > 2 and estado == 1:
		rospy.loginfo("entrou - estado: %d" %(estado))
		estado = 2
		motion.linear.x = -0.3
		motion.angular.z = -0.2
	if obstTraz < 0.2 and estado == 2:
		rospy.loginfo("entrou - estado: %d" %(estado))
		estado = 3
		motion.angular.z = 0.3
	if obstTraz < 0.1 and estado == 3:
		rospy.loginfo("entrou - estado: %d" %(estado))
		estado = 4
		motion.linear.x = 0.15
		motion.angular.z = 0.3
	if oriZ >= -0.05 and oriZ <= 0.05 and estado == 4:
		rospy.loginfo("entrou - estado: %d" %(estado))
		estado = 5
		motion.linear.x = 0
		motion.angular.z = 0

	rospy.loginfo("estado: %d" %(estado))

	cmd.publish(motion)


def callbackScan(laserScan, tipo):
	global iniX
	global distX
	global espacoVazio
	global estadoCont
	global obstEsq
	global obstTraz

	point_cloud = laser_projector.projectLaser(laserScan)
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
		somaX = somaX + x
		somaY = somaY + y
		pontos = pontos + 1

	if tipo == 'esquerda' and pontos > 0:
		mediaX = (somaX)/pontos	
		mediaY = (somaY)/pontos	
		obstEsq = mediaX-mediaY

		# Se o laser não detectar obstáculo, inicia a contagem do espaço vazio
		if obstEsq > 0.15 and estadoCont == 0:
			estadoCont = 1	
			iniX = posX
		# Se o laser detectar obstáculo, finaliza a contagem do espaço vazio
		elif obstEsq < 0.15 and estadoCont == 1:
			estadoCont = 0
			espacoVazio = distX
		# Se o estadoCont == 1, há um espaço vazio sendo mensurado
		elif estadoCont == 1:
			distX = posX - iniX
		
		#rospy.loginfo("obst-%s: %.1f oriZ: %.1f dist: %.1f esp: %.1f" %(tipo, obstEsq, oriZ, distX, espacoVazio))

	elif tipo == 'trazeira' and pontos > 0:
		mediaX = (somaX)/pontos	
		mediaY = (somaY)/pontos	
		obstTraz = mediaX-mediaY
		
		rospy.loginfo("obst-%s: %.1f oriZ: %.1f dist: %.1f esp: %.1f" %(tipo, obstTraz, oriZ, distX, espacoVazio))


def callbackScan1(laserScan):
	callbackScan(laserScan, 'esquerda')


def callbackScan2(laserScan):
	callbackScan(laserScan, 'trazeira')


cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
pos = rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
si1 = rospy.Subscriber("/atrv/sick1", LaserScan, callbackScan1)
si2 = rospy.Subscriber("/atrv/sick2", LaserScan, callbackScan2)


estado = 0
estadoCont = 0
posX = 0
iniX = 0
distX = 0
oriZ = 0
obstEsq = 0
obstTraz = 0
espacoVazio = 0

laser_projector = LaserProjection()
motion = Twist()

rospy.init_node("cliente")
rospy.spin() # this will block untill you hit Ctrl+C

