from morse.builder import *

# Land robot
atrv = ATRV()

pose = Pose()
pose.translate(y = 4, z = 0.75)
atrv.append(pose)

motion = MotionVW()
atrv.append(motion)

# Add default interface for our robot's components
atrv.add_default_interface('ros')

env = Environment('/home/projeto/carro-autonomo/teste2/fatec')
