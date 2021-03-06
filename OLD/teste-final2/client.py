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
	global obstFrente

	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	
	posX = position.x
	oriZ = orientation.z
	
	if estado == 0:
		estado = 1
		motion.linear.x = 1
		motion.angular.z = 0
	if estado == 1:
		if espacoVazio > 2:
			estado = 2
			motion.linear.x = -0.3
			motion.angular.z = -0.2
	if estado == 2:
		if obstTraz < 0.15:
			estado = 3
			motion.linear.x = -0.3
			motion.angular.z = 0.3
	if estado == 3:
		if obstTraz < 0.1:
			estado = 4
			motion.linear.x = 0.15
			motion.angular.z = 0.3
	if estado == 4:
		if obstFrente < 0.15:
			estado = 5
			motion.linear.x = 0.15
			motion.angular.z = -0.3
	if estado == 5:
		if oriZ >= -0.02 and oriZ <= 0.02:
			estado = 6
			motion.linear.x = 0
			motion.angular.z = 0

	cmd.publish(motion)


def callbackScan(laserScan, tipo):
	global iniX
	global distX
	global espacoVazio
	global estadoCont
	global obstEsq
	global obstTraz
	global obstFrente

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

	if tipo == 'esq' and pontos > 0:
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

	elif (tipo == 'tr1' or tipo == 'tr2') and pontos > 0:
		mediaX = (somaX)/pontos	
		mediaY = (somaY)/pontos	
		obstTraz = mediaX-mediaY
		
		#rospy.loginfo("obst-%s: %.1f oriZ: %.1f esp: %.1f" %(tipo, obstTraz, oriZ, espacoVazio))

	elif tipo == 'fre' and pontos > 0:
		mediaX = (somaX)/pontos	
		mediaY = (somaY)/pontos	
		obstFrente = mediaX-mediaY


def callbackScanEsquerda(laserScan):
	callbackScan(laserScan, 'esq')


def callbackScanFrente(laserScan):
	callbackScan(laserScan, 'fre')


def callbackScanTraz1(laserScan):
	callbackScan(laserScan, 'tr1')


def callbackScanTraz2(laserScan):
	callbackScan(laserScan, 'tr2')


cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
pos = rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
si1 = rospy.Subscriber("/atrv/sickEsq", LaserScan, callbackScanEsquerda)
si2 = rospy.Subscriber("/atrv/sickFrente", LaserScan, callbackScanFrente)
si3 = rospy.Subscriber("/atrv/sickTraz1", LaserScan, callbackScanTraz1)
si4 = rospy.Subscriber("/atrv/sickTraz2", LaserScan, callbackScanTraz2)


estado = 0
estadoCont = 0
posX = 0
iniX = 0
distX = 0
oriZ = 0
obstEsq = 0
obstTraz = 0
obstFrente = 0
espacoVazio = 0

laser_projector = LaserProjection()
motion = Twist()

rospy.init_node("cliente")
rospy.spin() # this will block untill you hit Ctrl+C

