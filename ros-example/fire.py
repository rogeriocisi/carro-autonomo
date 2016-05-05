#!/usr/bin/python

"""Base code provided by Harvey Mudd College, modified extensivly by Peter Tran.
Stripped down to code to the basic components so that the robot will simply fire
without any extra input from the user via the GUI. It also has been intergrated
with bark.py."""

import os
import sys
import time
import pygame
import usb.core
import rospy
from Tkinter import *
from PIL import Image, ImageTk

class launchControl():
	def __init__(self):
		self.dev = usb.core.find(idVendor=0x2123, idProduct=0x1010)
		if self.dev is None:
			raise ValueError('Launcher not found.')
		if self.dev.is_kernel_driver_active(0) is True:
			self.dev.detach_kernel_driver(0)
		self.dev.set_configuration()
		self.listener()

	def callback(data):
		"""If data sent from bark.py indicates that the robot should shoot, then it
		will do so. And it will constantly shoot."""
		if str(data) == "data: Shoot":
			self.turretFire

	def turretFire(self, event):
		self.dev.ctrl_transfer(0x21,0x09,0,0,[0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00])

	def listener(self):
		"""Subscribes to bark.py."""
		print "I am listening"
		rospy.init_node('fire', anonymous=True)
		rospy.Subscriber("Fire", String, callback)
		rospy.spin()

"""This script must be run as root in order to work, it also must be run using rosrun
in order for rospy to correctly import."""
if __name__ == '__main__':
	if not os.geteuid() == 0:
		sys.exit("Script must be run as root.")
	launchControl()


