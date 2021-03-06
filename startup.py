#!/usr/bin/env python3
#This script is originally by Maxwell Budd (TechBotBuilder).
#Please credit me as one or the other if you find this work helpful or derive code from it.

#this script should be setup to run in /etc/rc.local right before exit(0)
#with the line "python3 {path_to_avc_dir}/startup.py &"
#but don't do that unless you have a switch going from https://pinout.xyz/pinout/pin16_gpio23 to gnd
#or else you won't be able to boot up normally.

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
startswitchpin = 16
GPIO.setup(startswitchpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)#normally connected to +3 v
if GPIO.input(startswitchpin) == 1:
    #if the switch is not closed do the program launch sequence
    #run the pi-blaster daemon
    from subprocess import call
    call("sudo /home/pi/piblaster/pi-blaster-master/pi-blaster", shell=True)
    from time import sleep
    sleep(10) #give the pi a moment to let other daemons start (like pi-blaster)
    from controller import main
    main()#run the program
#otherwise, if someone has closed the switch->0v, just do nothing
