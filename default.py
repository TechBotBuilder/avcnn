#! /usr/bin/env morseexec

from morse.builder import *
#from simulation.builder.robots import Speeder
from math import pi
SCALINGFACTOR = 14.59813
laserd = SCALINGFACTOR * 4.5

robot = Hummer() #NOT our own, scaled Hummer. Just a hummer, we'll have to scale env to it
robot.translate(136.5, 54.5, 2)
robot.rotate(0,0,3*pi/2)#4.7)#4.712)#3.141)
robot.properties(GroundRobot = True)#no way to control it's z-axis position
#robot.set_mass(1.0)#this might be in kg?
robot.set_mass(10)
robot.add_default_interface('socket')

##teleporter so we can mess with the robot
#teleport = Teleport('teleport')
#teleport.add_stream('socket')
#robot.append(teleport)

motion = SteerForce()
motion.add_service('socket')
robot.append(motion)

clock = Clock()
clock.frequency(10)
clock.add_stream('socket')
robot.append(clock)

#gps = GPS()
#gps.level("raw")
##gps.translate(136.5, 54.5, 2)
##gps.rotate(0,0,4.712)#3.141) #pieces added relative to robot
#gps.frequency(2)#Update GPS twice per second
#gps.add_stream('socket')
#robot.append(gps)

gps = Pose("gps")
gps.frequency(10)
gps.add_stream('socket')
gps.rotate(0,0,-pi)
robot.append(gps)

#position_monitor = Pose("position_monitor")
#position_monitor.frequency(10)
#position_monitor.add_stream('socket')
#position_monitor.rotate(0,0,-pi)
#robot.append(position_monitor)

infraredRight = Infrared()
#infraredRight.translate(137.5, 56, 2.1)#placed forward and to right
#infraredRight.rotate(0,0,3.926)#look forward to right
#^relative to robot.
infraredRight.properties(laser_range = laserd)#4m * 13scaling
infraredRight.translate(-0.6,-3.5,0.6)#forward, right, up
infraredRight.rotate(0,0,-3*pi/4)
infraredRight.frequency(2)
infraredRight.add_stream('socket')
robot.append(infraredRight)

infraredLeft = Infrared()
#infraredLeft.translate(135.5, 55.5, 2.1)#placed forward and to left
#infraredLeft.rotate(0,0,2.356) #look forward to left
infraredLeft.properties(laser_range = laserd)
infraredLeft.translate(0.6,-3.5,0.6)
infraredLeft.rotate(0,0,-pi/4)#pi/4
infraredLeft.frequency(2)
infraredLeft.add_stream('socket')
robot.append(infraredLeft)

infraredCenter = Infrared()
#infraredCenter.translate(136.5, 56, 2.1)
#infraredCenter.rotate(0,0,3.1415)
infraredCenter.properties(laser_range = laserd)
infraredCenter.translate(0,-3.5,0.6)
infraredCenter.rotate(0,0,-pi/2)
infraredCenter.frequency(2)
infraredCenter.add_stream('socket')
robot.append(infraredCenter)

#cam = VideoCamera()
#cam.properties(Vertical_Flip=False)
#cam.translate(-20,-7,12)
#cam.properties(cam_far=1000)
#robot.append(cam)

env = Environment('/home/max/simulation/bigcourse/bigcourse.blend', fastmode = False)
env.add_interface('socket')
env.properties(longitude = 0, latitude = 0, altitude = 135.0)
#env.select_display_camera(cam)
# or for MORSE' CameraFP
#env.set_camera_location([136.5, 54.5, 20])
env.set_camera_location([0,100,820])
env.set_camera_rotation([0,0,3.14])
env.set_camera_clip(clip_end=1200)
env.set_time_scale(accelerate_by=4.8)#3)#6)#5)#10)#3)#can barely run 6 for simplerlrnn)
#env.add_service('socket')
#env.set_camera_location([-20.31814, -385.46207, 115.533])
#env.set_camera_location([-20.31814, -10, 10])
#env.set_camera_rotation([-1.745, -3.142, 3.142])
#env.set_camera_rotation([0, 0, 0])

