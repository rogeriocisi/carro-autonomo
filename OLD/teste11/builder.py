from morse.builder import *

# Land robot
atrv = ATRV()
atrv2 = ATRV()
atrv3 = ATRV()
atrv4 = ATRV()
atrv5 = ATRV()

atrv.translate(x = 0, y = 12)
atrv2.translate(x = 0, y = 13.4)
atrv3.translate(x = 1.8, y = 13.4)
atrv4.translate(x = 3.6, y = 13.4)
atrv5.translate(x = 7.5, y = 13.4)

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

#cube = Cube('calcada')
#cube.translate(x = 3.5, y = 2.6, z = -4.7)
#cube.scale = (5, 0.5, .1)

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

# Add default interface for our robot's components
atrv.add_default_interface('ros')

# env = Environment('./estacionamento.blend')
env = Environment('indoors-1/boxes')

env._camera_location = [5, -10, 5]
# env._camera_rotation = [0.7854, 0, 0.7854]



