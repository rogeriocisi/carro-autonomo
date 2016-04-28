#!/usr/bin/python

# Este programa faz o add, commit e push no Github

import commands
import subprocess
import os
import sys
import sys

comentario = ''

for arg in sys.argv:
    comentario = comentario + ' ' + arg

pr = subprocess.Popen( "/usr/bin/git add *" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

pr = subprocess.Popen( "/usr/bin/git commit -m '" + comentario + "'", cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

pr = subprocess.Popen( "/usr/bin/git push origin master" , cwd = os.path.dirname( '/home/rogerio/carro-autonomo/' ), shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
(out, error) = pr.communicate()

print "saida : " + str(error) + "\n" + str(out)

