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
atrv.append(pose)

# add sick sensor
sick1 = Sick()
sick1.translate(0, 0.4, 0.2) 
sick1.rotate(1.5, 0, 1.5)
sick1.frequency(3.0)

sick2 = Sick()
sick2.translate(-0.6, 0, 0.2) 
sick2.rotate(1.5, 0, 3.0)
sick2.frequency(3.0)

atrv.append(sick1)
atrv.append(sick2)

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



