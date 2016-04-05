#!/usr/bin/env python3
#This script is originally by Maxwell Budd (TechBotBuilder).
#Please credit me as one or the other if you find this work helpful or derive code from it.

#this script should be setup to run in /etc/rc.local right before exit(0)
#with the line "python3 {path_to_avc_dir}/startup.py &"
#but don't do that unless you have a switch going from https://pinout.xyz/pinout/pin16_gpio23 to gnd
#or else you won't be able to boot up normally.

from controller import main
import RPi.GPIO as GPIO

startswitchpin = 16
GPIO.setup(startswitchpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)#normally connected to +3 v
if GPIO.input(startswitchpin) == 1:
    #if the switch is not closed
    main()#run the program
#otherwise, if someone has closed the switch->0v, just do nothing
