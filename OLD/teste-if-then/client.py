#!/usr/bin/env python3

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


class Controle:

	def __init__(self):
		self.cmd = rospy.Publisher("/atrv/motion", Twist, queue_size=10)
		self.motion = Twist()

		MAX_IR = 3
		self.obstEs = [MAX_IR, MAX_IR, MAX_IR]
		self.obstDi = [MAX_IR, MAX_IR, MAX_IR]
		self.obstFr = [MAX_IR, MAX_IR, MAX_IR]
		self.obstTr = [MAX_IR, MAX_IR, MAX_IR]
		self.estado = 0
		self.estadoCont = 0
		self.espacoVazio = 0
		self.posX = 0
		self.oriZ = 0
		self.iniX = 0
		self.distX = 0
		self.distLidar = 0
		self.direcao = 1


	def mover(self):
		# Obtem as distancias minimas dos 4 sensores da frente, traz, esquerda e direita
		obstFrMin = min(self.obstFr)
		obstTrMin = min(self.obstTr)
		obstEsMin = min(self.obstEs)
		obstDiMin = min(self.obstDi)

		rospy.loginfo("%d fr: %.1f tr: %.1f es: %.1f di: %.1f oriZ: %.3f" %(self.estado, obstFrMin, obstTrMin, obstEsMin, obstDiMin, self.oriZ))

		# Algoritmo do park assist
		if self.estado == 0:
			self.estado = 1
			self.motion.linear.x = 1
			self.motion.angular.z = 0
		if self.estado == 1:
			# Se estiver no sentido negativo do eixo X, inverte a variavel de direcao
			if abs(self.oriZ) > 0.9:
				self.direcao = -1

			if self.espacoVazio > 2.2:
				self.estado = 2
				self.motion.linear.x = -0.3
				self.motion.angular.z = -0.2 * self.direcao
		if self.estado == 2:
			if obstTrMin < 0.6:
				self.estado = 3
				self.motion.linear.x = -0.3
				self.motion.angular.z = 0.3 * self.direcao
		if self.estado == 3:
			if obstTrMin < 0.3:
				self.estado = 4
				self.motion.linear.x = 0.15
				self.motion.angular.z = 0.3 * self.direcao
		if self.estado == 4:
			if (abs(self.oriZ) < 0.02 or abs(self.oriZ) > 0.9999) and ((self.direcao == 1 and obstEsMin < 0.6) or (self.direcao == -1 and obstDiMin < 0.6)):
				self.estado = 6
				self.motion.linear.x = 0
				self.motion.angular.z = 0

		self.cmd.publish(self.motion)


	def calculaEspacoParkAssist(self, distObst):
		# Utiliza a medida distObst para o park assist
		# Se o sensor não detectar obstáculo, inicia a contagem do espaço vazio
		if self.estadoCont == 0 and distObst > 1.3:
			self.estadoCont = 1	
			self.iniX = self.posX
		# Se o sensor detectar obstáculo, finaliza a contagem do espaço vazio
		elif self.estadoCont == 1 and distObst < 0.8:
			self.estadoCont = 0
			self.espacoVazio = self.distX
		# Se o estadoCont == 1, há um espaço vazio sendo mensurado
		elif self.estadoCont == 1:
			self.distX = abs(self.posX - self.iniX)


	def callbackPose(self, poseStamped):
		position = poseStamped.pose.position
		orientation = poseStamped.pose.orientation
		self.posX = position.x
		self.oriZ = orientation.z

		self.mover()


	def callbackSick(self, laserScan):
		depths = []
		for dist in laserScan.ranges:
			if not np.isnan(dist):
				depths.append(dist)

		if len(depths) == 0:
			self.distLidar = 0
		else:
			self.distLidar = min(depths)


	def callbackIR(self, infrared, tipo):
		# http://people.cornellcollege.edu/smikell15/MAX        
		# Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
		closest = 0
		depths = []
		for dist in infrared.ranges:
			if not np.isnan(dist):
				depths.append(dist)

		#If depths is empty that means we're way too close to an object to get a reading.
		#thus establish our distance to nearest object as "0".
		if len(depths) == 0:
			closest = 0
		else:
			closest = min(depths)

		#rospy.loginfo("closest: %.1f position: %.1f" %(closest, position))

		if tipo == 'es1':
			self.obstEs[0] = closest
			# Se estiver trafegando no sentido positivo do eixo X
			if abs(self.oriZ) < 0.1 and self.estado <= 1:
				self.calculaEspacoParkAssist(self.obstEs[0])
		elif tipo == 'di1':
			self.obstDi[0] = closest
			# Se estiver trafegando no sentido negativo do eixo X
			if abs(self.oriZ) > 0.9 and self.estado <= 1:
				self.calculaEspacoParkAssist(self.obstDi[0])
		elif tipo == 'es2':
			self.obstEs[1] = closest
		elif tipo == 'es3':
			self.obstEs[2] = closest
		elif tipo == 'di2':
			self.obstDi[1] = closest
		elif tipo == 'di3':
			self.obstDi[2] = closest
		elif tipo == 'fr1':
			self.obstFr[0] = closest
		elif tipo == 'fr2':
			self.obstFr[1] = closest
		elif tipo == 'fr3':
			self.obstFr[2] = closest
		elif tipo == 'tr1':
			self.obstTr[0] = closest
		elif tipo == 'tr2':
			self.obstTr[1] = closest
		elif tipo == 'tr3':
			self.obstTr[2] = closest


def callbackPose(poseStamped):
	controle.callbackPose(poseStamped)

def callbackIREs1(infrared):
	controle.callbackIR(infrared, 'es1')

def callbackIREs2(infrared):
	controle.callbackIR(infrared, 'es2')

def callbackIREs3(infrared):
	controle.callbackIR(infrared, 'es3')

def callbackIRFr1(infrared):
	controle.callbackIR(infrared, 'fr1')

def callbackIRFr2(infrared):
	controle.callbackIR(infrared, 'fr2')

def callbackIRFr3(infrared):
	controle.callbackIR(infrared, 'fr3')

def callbackIRTr1(infrared):
	controle.callbackIR(infrared, 'tr1')

def callbackIRTr2(infrared):
	controle.callbackIR(infrared, 'tr2')

def callbackIRTr3(infrared):
	controle.callbackIR(infrared, 'tr3')

def callbackIRDi1(infrared):
	controle.callbackIR(infrared, 'di1')

def callbackIRDi2(infrared):
	controle.callbackIR(infrared, 'di2')

def callbackIRDi3(infrared):
	controle.callbackIR(infrared, 'di3')

def callbackSick(laserScan):
	controle.callbackSick(laserScan)

def listener():
	# Initializes node, creates subscriber, and states callback functions
	rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")

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
	sic = rospy.Subscriber("/atrv/sick", LaserScan, callbackSick)

	rospy.spin()

if __name__ == "__main__":
	controle = Controle()
	listener()


