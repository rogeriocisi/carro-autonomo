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
pose.frequency(3.0)
atrv.append(pose)

# add infrared sensors
infraredFr1 = Infrared()
infraredFr1.translate(0.6, -0.3, 0.2) 
infraredFr1.rotate(0, 0, 0)
infraredFr1.frequency(5.0)
infraredFr1.min_range = 0.0
infraredFr1.max_range = 2.0
atrv.append(infraredFr1)

infraredFr2 = Infrared()
infraredFr2.translate(0.6, 0, 0.2) 
infraredFr2.rotate(0, 0, 0)
infraredFr2.frequency(5.0)
infraredFr2.min_range = 0.0
infraredFr2.max_range = 2.0
atrv.append(infraredFr2)

infraredFr3 = Infrared()
infraredFr3.translate(0.6, 0.3, 0.2) 
infraredFr3.rotate(0, 0, 0)
infraredFr3.frequency(5.0)
infraredFr3.min_range = 0.0
infraredFr3.max_range = 2.0
atrv.append(infraredFr3)

infraredEs1 = Infrared()
infraredEs1.translate(0, 0.35, 0.2) 
infraredEs1.rotate(1.57, 0, 1.57)
infraredEs1.frequency(10.0)
infraredEs1.min_range = 0.0
infraredEs1.max_range = 2.0
atrv.append(infraredEs1)

infraredEs2 = Infrared()
infraredEs2.translate(0, 0.35, 0.2) 
infraredEs2.rotate(0, 0, 1.57)
infraredEs2.frequency(10.0)
infraredEs2.min_range = 0.0
infraredEs2.max_range = 2.0
atrv.append(infraredEs2)

infraredTr1 = Infrared()
infraredTr1.translate(-0.6, -0.3, 0.2) 
infraredTr1.rotate(0, 0, 3.14)
infraredTr1.frequency(5.0)
infraredTr1.min_range = 0.0
infraredTr1.max_range = 2.0
atrv.append(infraredTr1)

infraredTr2 = Infrared()
infraredTr2.translate(-0.6, 0, 0.2) 
infraredTr2.rotate(0, 0, 3.14)
infraredTr2.frequency(5.0)
infraredTr2.min_range = 0.0
infraredTr2.max_range = 2.0
atrv.append(infraredTr2)

infraredTr3 = Infrared()
infraredTr3.translate(-0.6, 0.3, 0.2) 
infraredTr3.rotate(0, 0, 3.14)
infraredTr3.frequency(5.0)
infraredTr3.min_range = 0.0
infraredTr3.max_range = 2.0
atrv.append(infraredTr3)

# add sick sensors
#sickEsq = Sick()
#sickEsq.translate(0, 0.35, 0.2) 
#sickEsq.rotate(1.57, 0, 1.57)
#sickEsq.frequency(10.0)
#atrv.append(sickEsq)

#sickFrente = Sick()
#sickFrente.translate(0.6, 0, 0.2) 
#sickFrente.rotate(1.57, 0, 0)
#sickFrente.frequency(10.0)
#atrv.append(sickFrente)

#sickTraz2 = Sick()
#sickTraz2.translate(-0.6, -0.2, 0.2) 
#sickTraz2.rotate(1.57, 0, 3.14)
#sickTraz2.frequency(10.0)
#atrv.append(sickTraz2)

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



