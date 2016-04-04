from controller import main
import RPi.GPIO as GPIO

startswitchpin = 16
GPIO.setup(startswitchpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)#normally connected to +3 v
if GPIO.input(startswitchpin) == 1:
    #if the switch is not closed
    main()#run the program
#otherwise, if someone has closed the switch->0v, just do nothing