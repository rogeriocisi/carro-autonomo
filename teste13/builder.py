from morse.builder import *

# Land robot
atrv = ATRV()
atrv2 = ATRV()
atrv3 = ATRV()
atrv4 = ATRV()
atrv5 = ATRV()

atrv.translate(x = 0, y = 12.2)
atrv2.translate(x = 0, y = 13.4)
atrv3.translate(x = 1.8, y = 13.4)
atrv4.translate(x = 3.6, y = 13.4)
atrv5.translate(x = 7.5, y = 13.4)

pose = Pose()
pose.translate(z = 0.75)
pose.frequency(10.0)
atrv.append(pose)

# add sick sensors
sickEsq = Sick()
sickEsq.translate(0, 0.35, 0.2) 
sickEsq.rotate(1.57, 0, 1.57)
sickEsq.frequency(10.0)
atrv.append(sickEsq)

infraredTraz1 = Infrared()
infraredTraz1.translate(-0.6, 0, 0.2) 
infraredTraz1.rotate(0, 0, 3.14)
infraredTraz1.frequency(5.0)
infraredTraz1.min_range = 0.0
infraredTraz1.max_range = 2.0
atrv.append(infraredTraz1)

#sickTraz2 = Sick()
#sickTraz2.translate(-0.6, -0.2, 0.2) 
#sickTraz2.rotate(1.57, 0, 3.14)
#sickTraz2.frequency(10.0)
#atrv.append(sickTraz2)

sickFrente = Sick()
sickFrente.translate(0.6, 0, 0.2) 
sickFrente.rotate(1.57, 0, 0)
sickFrente.frequency(10.0)
atrv.append(sickFrente)

motion = MotionVW()
atrv.append(motion)

keyboard = Keyboard()
keyboard.properties(ControlType = 'Position')
atrv.append(keyboard)

# Add default interface for our robot's components
atrv.add_default_interface('ros')

# env = Environment('./estacionamento.blend')
env = Environment('indoors-1/boxes')

env._camera_location = [14, 10, 9]
# env._camera_rotation = [0.7854, 0, 0.7854]



