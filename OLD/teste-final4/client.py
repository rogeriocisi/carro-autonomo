#!/usr/bin/env python3
import rospy
import math
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from laser_geometry import LaserProjection


def callbackPose(poseStamped):
	global estado
	global posX
	global espacoVazio
	global obstEs
	global obstDi
	global obstTr
	global obstFr

	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	
	posX = position.x
	oriZ = orientation.z
	oriX = orientation.x

	# Obtem as distancias minimas dos 4 sensores da frente, traz, esquerda e direita
	obstFrMin = min(obstFr)
	obstTrMin = min(obstTr)
	obstEsMin = min(obstEs)
	obstDiMin = min(obstDi)

	rospy.loginfo("%d fr: %.1f tr: %.1f es: %.1f di: %.1f oriX: %.1f" %(estado, obstFrMin, obstTrMin, obstEsMin, obstDiMin, oriX))

	if obstFrMin < 0.3 and motion.linear.x > 0:
		motion.linear.x = motion.linear.x/2

	if estado == 0:
		estado = 1
		motion.linear.x = 1
		motion.angular.z = 0
	if estado == 1:
		if espacoVazio > 2.2:
			estado = 2
			motion.linear.x = -0.3
			motion.angular.z = -0.2
	if estado == 2:
		if obstTrMin < 0.6:
			estado = 3
			motion.linear.x = -0.3
			motion.angular.z = 0.3
	if estado == 3:
		if obstTrMin < 0.4:
			estado = 4
			motion.linear.x = 0.15
			motion.angular.z = 0.3
	if estado == 4:
		if obstFrMin < 0.4 or obstEsMin < 0.2:
			estado = 5
			motion.linear.x = 0.15
			motion.angular.z = -0.3
	if estado == 5:
		if obstFrMin < 0.9 and oriZ >= -0.02 and oriZ <= 0.02:
			estado = 6
			motion.linear.x = 0
			motion.angular.z = 0

	cmd.publish(motion)


def calculaEspacoParkAssist():
	global iniX
	global distX
	global espacoVazio
	global estadoCont
	global obstEs

	# Utiliza o sensor obstEs[1] para o park assist
	# Se o laser não detectar obstáculo, inicia a contagem do espaço vazio
	if estadoCont == 0 and obstEs[0] > 1.3:
		estadoCont = 1	
		iniX = posX
	# Se o laser detectar obstáculo, finaliza a contagem do espaço vazio
	elif estadoCont == 1 and obstEs[0] < 0.7:
		estadoCont = 0
		espacoVazio = distX
	# Se o estadoCont == 1, há um espaço vazio sendo mensurado
	elif estadoCont == 1:
		distX = posX - iniX
	
	#rospy.loginfo("obst: %.1f dist: %.1f esp: %.1f" %(obstEsq, distX, espacoVazio))


def callbackIR(infrared, tipo):
	global obstEs
	global obstTr
	global obstFr
	global obstDi
	global MAX_IR
	closest = 0

	# http://people.cornellcollege.edu/smikell15/MAX/code/follower.py.html        
	# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
	depths = []
	for dist in infrared.ranges:
		if not np.isnan(dist):
			depths.append(dist)

	#scan.ranges is a tuple, and we want an array.
	fullDepthsArray = infrared.ranges[:]

	#If depths is empty that means we're way too close to an object to get a reading.
	#thus establish our distance/position to nearest object as "0".
	if len(depths) == 0:
		closest = 0
	else:
		closest = min(depths)


	#rospy.loginfo("closest: %.1f position: %.1f" %(closest, position))


	if tipo == 'es1':
		obstEs[0] = closest
		calculaEspacoParkAssist()
	elif tipo == 'es2':
		obstEs[1] = closest
	elif tipo == 'es3':
		obstEs[2] = closest
	elif tipo == 'fr1':
		obstFr[0] = closest
	elif tipo == 'fr2':
		obstFr[1] = closest
	elif tipo == 'fr3':
		obstFr[2] = closest
	elif tipo == 'tr1':
		obstTr[0] = closest
	elif tipo == 'tr2':
		obstTr[1] = closest
	elif tipo == 'tr3':
		obstTr[2] = closest
	elif tipo == 'di1':
		obstDi[0] = closest
	elif tipo == 'di2':
		obstDi[1] = closest
	elif tipo == 'di3':
		obstDi[2] = closest



def callbackIREs1(infrared):
	callbackIR(infrared, 'es1')

def callbackIREs2(infrared):
	callbackIR(infrared, 'es2')

def callbackIREs3(infrared):
	callbackIR(infrared, 'es3')

def callbackIRFr1(infrared):
	callbackIR(infrared, 'fr1')

def callbackIRFr2(infrared):
	callbackIR(infrared, 'fr2')

def callbackIRFr3(infrared):
	callbackIR(infrared, 'fr3')

def callbackIRTr1(infrared):
	callbackIR(infrared, 'tr1')

def callbackIRTr2(infrared):
	callbackIR(infrared, 'tr2')

def callbackIRTr3(infrared):
	callbackIR(infrared, 'tr3')

def callbackIRDi1(infrared):
	callbackIR(infrared, 'di1')

def callbackIRDi2(infrared):
	callbackIR(infrared, 'di2')

def callbackIRDi3(infrared):
	callbackIR(infrared, 'di3')

'''
def callbackSick(laserScan):
	point_cloud = laser_projector.projectLaser(laserScan)
	minLaser = 0 
	maxLaser = 0 

	# calcula os pontos mínimo e máximo de detecção do laser
	for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
		if p[0] < minLaser:
			minLaser = p[0] 
		elif p[0] > maxLaser:
			maxLaser = p[0] 
		if p[1] < minLaser:
			minLaser = p[1] 
		elif p[1] > maxLaser:
			maxLaser = p[1] 
		
	rospy.loginfo("minLaser: %.1f maxLaser: %.1f" %(minLaser, maxLaser))
'''

cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
pos = rospy.Subscriber("/atrv/pose", PoseStamped, callbackPose)
ie1 = rospy.Subscriber("/atrv/infraredEs1", LaserScan, callbackIREs1)
ie2 = rospy.Subscriber("/atrv/infraredEs2", LaserScan, callbackIREs2)
ie3 = rospy.Subscriber("/atrv/infraredEs3", LaserScan, callbackIREs3)
if1 = rospy.Subscriber("/atrv/infraredFr1", LaserScan, callbackIRFr1)
if2 = rospy.Subscriber("/atrv/infraredFr2", LaserScan, callbackIRFr2)
if3 = rospy.Subscriber("/atrv/infraredFr3", LaserScan, callbackIRFr3)
it1 = rospy.Subscriber("/atrv/infraredTr1", LaserScan, callbackIRTr1)
it2 = rospy.Subscriber("/atrv/infraredTr2", LaserScan, callbackIRTr2)
it3 = rospy.Subscriber("/atrv/infraredTr3", LaserScan, callbackIRTr3)
di1 = rospy.Subscriber("/atrv/infraredDi1", LaserScan, callbackIRDi1)
di2 = rospy.Subscriber("/atrv/infraredDi2", LaserScan, callbackIRDi2)
di3 = rospy.Subscriber("/atrv/infraredDi3", LaserScan, callbackIRDi3)
#sic = rospy.Subscriber("/atrv/sick", LaserScan, callbackSick)


estado = 0
estadoCont = 0
MAX_IR = 3
posX = 0
iniX = 0
distX = 0
obstEs = [MAX_IR, MAX_IR, MAX_IR]
obstDi = [MAX_IR, MAX_IR, MAX_IR]
obstFr = [MAX_IR, MAX_IR, MAX_IR]
obstTr = [MAX_IR, MAX_IR, MAX_IR]
espacoVazio = 0

laser_projector = LaserProjection()
motion = Twist()

rospy.init_node("cliente")
rospy.spin() # this will block untill you hit Ctrl+C

