#!/usr/bin/python

import commands
import subprocess
import os
import sys

pr = subprocess.Popen( "/usr/bin/git pull" , cwd = os.path.dirname( os.getcwd() + "/" ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "Error : " + str(error) 
print "out : " + str(out)
