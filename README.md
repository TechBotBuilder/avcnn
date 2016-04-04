# avcnn
Allows raspberry pi to use a neural network to control an autonomous robot.

This project originally developed for the National Robotics Challenge 2016 (thenrc.org) by Maxwell Budd = TechBotBuilder
Please credit me if you find this work helpful or derive code from it.

# HOWTO
First you should train a neural network using Keras on a simulator or using some other library.
Detailed instructions for getting all the requirements for this on your pi can be found at techbotbuilder.com/avcnn, or you can download and run get_requirements.sh on your pi.
If using get_requirements.sh these files will be in /home/pi/avc, so you only need to add the model weights from your simulation (call that weights.h5)
Edit lines 16-19 of pickle_compiled_model.py to match the model construction lines you used in your simulation.
Run "python3 pickle_compiled_model.py". It will take several minutes. If it gives you a "Exceeded Maximum Recursion Depth" Error, increase the recursion limit at line 6 of pickle_compiled_model.py (at your own risk).

If you want the robot files to run on startup, edit your /etc/rc.local to include "python3 /home/pi/avc/startup.py &" just before "exit(0)".
