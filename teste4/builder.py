from morse.builder import *

# Land robot
atrv = ATRV()
atrv2 = ATRV()
atrv3 = ATRV()
atrv4 = ATRV()
atrv5 = ATRV()

pose = Pose()
pose.translate(y = 4, z = 0.75)
atrv.append(pose)

motion = MotionVW()
atrv.append(motion)

# steerforce = SteerForce()
# atrv.append(steerforce)

keyboard = Keyboard()
atrv.append(keyboard)
keyboard.properties(ControlType = 'Position')

atrv.translate(x = 0, y = 0)
atrv2.translate(x = 0, y = 1.5)
atrv3.translate(x = 1.5, y = 1.5)
atrv4.translate(x = 3, y = 1.5)
atrv5.translate(x = 6, y = 1.5)

# Add default interface for our robot's components
atrv.add_default_interface('ros')

env = Environment('./estacionamento.blend')
