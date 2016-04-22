from morse.builder import *

# Land robot
atrv = ATRV()
atrv2 = ATRV()
atrv3 = ATRV()
atrv4 = ATRV()
atrv5 = ATRV()

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

# add sick sensor
sick = Sick()
sick.translate(0, 0.4, 0.2) 
sick.rotate(1.5, 0, 1.5)
sick.frequency(10.0)
atrv.append(sick)
# sick.add_stream("ros", topic='/sick')

motion = MotionVW()
atrv.append(motion)

keyboard = Keyboard()
keyboard.properties(ControlType = 'Position')
atrv.append(keyboard)

atrv.translate(x = 0, y = 0)
atrv2.translate(x = 0, y = 1.5)
atrv3.translate(x = 1.5, y = 1.5)
atrv4.translate(x = 3.0, y = 1.5)
atrv5.translate(x = 6.2, y = 1.5)

# Add default interface for our robot's components
atrv.add_default_interface('ros')

env = Environment('./estacionamento.blend')



