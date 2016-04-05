# avcnn
Use a raspberry pi to run a (possibly recurrent) neural network, which can control an autonomous robot.

This project originally developed for the Autonomous Vehicle Competition at the National Robotics Challenge 2016 (thenrc.org).
Please link to this Github or cite me in your own projects if you find this work helpful or derive code from it.

## Setup
First you should train a neural network using Keras on a simulator or using some other library.

You will need an OS for your Pi; I used Raspbian Jessie. You will also need to disable serial in raspi-config > Advanced. You also might need to overclock your Pi to get everything running in real time; I run mine (a Pi 1B rev2) at Turbo with heatsinks and a fan, but a lower overclock might work.

Detailed instructions for getting all the requirements for avcnn can be found at techbotbuilder.com/avcnn, or you can download and run get_requirements.sh on your pi.

If using get_requirements.sh, these files will be in /home/pi/avc, and you only need to add the model weights from your simulation (file name will need to be weights.h5), then edit lines 16-19 of pickle_compiled_model.py to match the model construction lines you used in your simulation. Finally, run "python3 pickle_compiled_model.py". It will take several minutes. If it gives you a "Exceeded Maximum Recursion Depth" Error, increase the recursion limit at line 6 of pickle_compiled_model.py (at your own risk).

Note: You will need to convert recurrent neural network models to stateful models. This simply means adding a "stateful=True" flag in your RNN layer. You don't need to do this in your simulations, only in pickle_compiled_model.py. This will keep your Pi from redoing all of the previous calculations it has done.

If you want the robot files to run on startup, edit your /etc/rc.local to include "python3 /home/pi/avc/startup.py &" just before "exit(0)". Don't do this unless you have a switch connecting P1 Header pin 16 to ground (see [Pin 16]( https://pinout.xyz/pinout/pin16_gpio23)), or else you won't be able to normally boot up your pi. The switch should be open when the pi boots to run the robot controller script, or closed when it boots to not run the script = boot normally.

## Hardware
View the bottom of techbotbuilder.com/avcnn for full hardware specifications. I will make a fritzing diagram soon showing the electronics as I used them. Until then you should be able to figure it out from the code.

If you want to change the number of Ultrasonic sensors, change the pins of electronics, change pwm values so your servo isn't always a little off center, get rid of the GPS, add a new sensor type, or change anything else, please feel free to do so and also send a pull request.

One more note - the ultrasonic sensors run at 5V (which will hurt your Pi), so you will need to use something to convert their "echo" signals to about 3.3v, which the Raspberry Pi can handle. I used three voltage dividers.
