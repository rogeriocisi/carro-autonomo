#!/usr/bin/python

import commands
import subprocess
import os
import sys

pr = subprocess.Popen( "/usr/bin/git pull" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()


print "Error : " + str(error) 
print "out : " + str(out)
