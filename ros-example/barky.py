#!/usr/bin/env python

"""Code developed by Rocky Mazorow and Peter Tran. It is intergrated with follow.py
and fire.py."""

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

"""If follow.py sends in "Bark", then this node will play the sound, as well as publish
to fire.py. As of currently, it will not work properly because fire.py cannot be
run. In order to be run, you have to use the command sudo rosrun, which does not work
unless the entire package is inside the root folder.

The function also publishes to fire immediatly when it barks, ideally we would like for
it to bark for several seconds as a warning before firing."""
def callback(data):
	if str(data) == "data: Bark":
		soundhandle = SoundClient()
		rospy.sleep(1)
		soundhandle.play(1)
		pubshoot = rospy.Publisher('bark', String)
		pubshoot.publish(String("Fire"))

"""Subscribes to follow.py"""
def listener():
	print "I am listening"
	rospy.init_node('bark', anonymous=True)
	rospy.Subscriber("follow", String, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()


