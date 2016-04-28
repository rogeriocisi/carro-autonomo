#!/usr/bin/python

import commands
import subprocess
import os
import sys

pr = subprocess.Popen( "/usr/bin/git add *" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "Error1 : " + str(error) 
print "out1 : " + str(out)

pr = subprocess.Popen( "/usr/bin/git commit -m 'teste'" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "Error2 : " + str(error) 
print "out2 : " + str(out)

pr = subprocess.Popen( "/usr/bin/git push origin master" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "Error3 : " + str(error) 
print "out3 : " + str(out)
