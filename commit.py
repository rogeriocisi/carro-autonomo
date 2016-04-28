#!/usr/bin/python

# Este programa faz o add, commit e push no Github

import commands
import subprocess
import os
import sys
import sys

comentario = ''
i = 0

if len(sys.argv) > 1:
	for arg in sys.argv:
		if i > 0:
			comentario = comentario + ' ' + arg
		i = i + 1
else:
    comentario = 'atualizacao'

pr = subprocess.Popen( "/usr/bin/git add *" , cwd = os.path.dirname( os.getcwd() + "/" ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

pr = subprocess.Popen( "/usr/bin/git commit -m '" + comentario + "'", cwd = os.path.dirname( os.getcwd() + "/" ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

pr = subprocess.Popen( "/usr/bin/git push origin master" , cwd = os.path.dirname( os.getcwd() + "/" ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

