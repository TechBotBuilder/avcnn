#!/usr/bin/env python3
#This script is originally by Maxwell Budd (TechBotBuilder).
#Please credit me as one or the other if you find this work helpful or derive code from it.

##this script must be run as sudo for GPIO to work
import RPi.GPIO as GPIO
import time
from threading import Thread
import numpy as np


#GPS Interface inspired by github.com/modmypi/GPS/blob/master/gps.py
import serial
import pynmea2
from subprocess import call
class GPSInterface:
    def __init__(self, positioner):
        #initialize serial connection with GPS
        call("stty -F /dev/ttyAMA0 raw 9600 cs8 clocal -cstopb", shell=True)
        self.connection = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.1)
        #setup some data variables - positions are at the starting pt on the course
        self.latitude = 40.604456
        self.longitude = -83.124911
        self.timestamp = 0.0
        self.positioner = positioner
        self.hasfix = False
    def update(self):
        data = self.connection.readline().decode() #readline() in 3 enforces binary data, .decode() makes it a str.
        if data.find('GGA') > 0:
            self.parseRaw(data)
            self.positioner.update([self.longitude, self.latitude])
    def parseRaw(self, data):
            msg = pynmea2.parse(data)
            if not self.hasfix:
                #if haven't gotten a fix yet, check to see if we do now.
                self.hasfix = int(msg.gps_qual) > 0
            if self.hasfix:
                self.timestamp = msg.timestamp
                self.latitude = msg.lat
                self.longitude = msg.lon
#and GPSUpdater is adapted from http://www.danmandle.com/blog/getting-gpsd-to-work-with-python/
class GPSUpdater(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True
    def setInterface(self, gpsInterface):
        self.gpsInterface = gpsInterface
    def run(self):
        while self.running:
            self.gpsInterface.update()
#map class adapted from the simulation code
#bottom left at 40.604155, -83.125484
#top right at 40.604831, -83.124592
#top to bottom size: 0.000686. left to right size:  0.000892
class GPSMap:
    #just resizes the xy coordinates to be in [0,1]x[0,1]
    def __init__(self, bigness=[0.000892, 0.000675], topleftcoord=[-83.125484, 40.604828], flipxaxis=True):
        #all coordinates need to be in EW, NS format = Longitude, Latitude
        self.tl = topleftcoord
        self.bigness = bigness
        self.internalstate = [0.0,0.0]
        self.flipxaxis = flipxaxis
    def update(self, coord):
        y = self.tl[1] - coord[1]
        x = self.tl[0] - coord[0]
        if(self.flipxaxis):
            x = -x
        y = y/self.bigness[1]
        x = x/self.bigness[0]
        self.internalstate = [x,y]
    def get(self):
        return self.internalstate

class PWMInterface:
    #class to control PWM items = servo and motor.
    def __init__(self):
        #Pi-blaster listens for file writes to control pwm
        #it uses GPIO numbers instead of P1 header pin numbers.
        self.servoPin = 18
        self.motorPin = 17
        self.servoCenterValue = 0.165
        self.motorOffValue = 0.15
        self.setServo(self.servoCenterValue)
        self.setMotor(self.motorOffValue)
    def setPin(self, pin, value):
        with open('/dev/pi-blaster', 'w') as f:
            f.write(str(pin) + "=" + str(float(value)) + "\n")
    def setServo(self, value):
        self.setPin(self.servoPin, value)
    def setMotor(self, value):
        self.setPin(self.motorPin, value)
    def releaseAll(self):
        try:
            with open('/dev/pi-blaster', 'w') as f:
                f.write("release " + str(self.servoPin) + "\n")
                f.write("release " + str(self.motorPin) + "\n")
        except:
            return False
        return True

#got some help from http://www.modmypi.com/blog/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi on this one
class UltrasonicSensor:
    def __init__(self, trigPin, echoPin, expectedTemp = 50):
        self.maxdistance = 4.5 #4.5 meters is max distance used in simulation
        self.trigPin = trigPin
        self.echoPin = echoPin
        #now do math. Temp @ marion supposed to be about 50 F high, -> 8C.
        #v(C) = 331.3*sqrt(1+C/273.15) = ...
        self.speedOfSound = 331.3 * ( (1+((expectedTemp-32)*5/9)/273.15) ** 0.5) #in meters/second
        self.magicConstant = (self.speedOfSound / 2) / self.maxdistance #in per-seconds
        self.timeoutPeriod = 2 * self.maxdistance / self.speedOfSound
        self.distance = 1 #start seeing max distance
        self.setup()
    def setup(self):
        GPIO.setup(self.trigPin, GPIO.OUT)
        GPIO.setup(self.echoPin, GPIO.IN)
        GPIO.output(self.trigPin, GPIO.LOW)
    def update(self):
        self.trigger()
        startedPulse = time.time()
        startedEcho = 0
        endedEcho = 0
        try:
            while self.getEcho() == GPIO.LOW:
                startedEcho = time.time()
                if startedEcho - startedPulse > self.timeoutPeriod:
                    raise RuntimeWarning
            while self.getEcho() == GPIO.HIGH:
                endedEcho = time.time()
                if endedEcho - startedPulse > 2 * self.timeoutPeriod:
                    raise RuntimeWarning
        except RuntimeWarning:
            #we reached a maximum amount of time waiting for a signal
            #so objects are out of range, -> return maximum distance
            self.distance = 1
            return 1
        distance = (endedEcho - startedEcho) * self.magicConstant #return val between 0 and 1
        self.distance = distance
        return distance
    def getEcho(self):
        #read state of echo
        return GPIO.input(self.echoPin)
    def trigger(self):
        #send a short (10us) pulse
        GPIO.output(self.trigPin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigPin, GPIO.LOW)
class UltrasonicUpdater(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True
    def setInterfaces(self, sensorInterfaces):
        #sensorInterfaces should be a list of UltrasonicSensor, in order of updates.
        #maximum of 12 to guarantee all are updated every second with maxdistance=4.5m
        self.sensorInterfaces = sensorInterfaces
    def run(self):
        while self.running:
            firstPingTime = time.time()
            for ultraSensor in self.sensorInterfaces:
                ultraSensor.update()
                #30ms is min recommended time to wait to keep from sensor cross-interference
                time.sleep(0.03)
                #one round of this takes a maximum time of about 0.08217s
                #-> maximum 12 sensors if want to do a full sweep every second
            time.sleep(max(0,(firstPingTime + 1) - time.time())) #wait the rest of a full second


class PhysicalReactor:
    #something to hold all our ways of interacting with the environment
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)#use counting scheme, just from 1 to 26
        #NOT PIblaster counting scheme.
        #so this DOES use counting scheme of 2 at corner, goes up by 1
        #see https://pinout.xyz for pin schemes.
        self.leftUltra = UltrasonicSensor(19, 22)
        self.centerUltra = UltrasonicSensor(21, 24)
        self.rightUltra = UltrasonicSensor(23, 26)
        self.driving = PWMInterface()
        self.positioner = GPSMap()
        self.gps = GPSInterface(self.positioner)
        self.gpsUpdater = GPSUpdater()
        self.gpsUpdater.setInterface(self.gps)
        self.gpsUpdater.start()
        self.ultraUpdater = UltrasonicUpdater()
        self.ultraUpdater.setInterfaces([self.leftUltra, self.centerUltra, self.rightUltra])
        self.ultraUpdater.start()
        leftval = 0.19 #these will vary from servo to servo and motor to motor.
        rightval = 0.14 #they should be somewhere from .1 to .2 -> 10-20ms PWM pulses.
        forwardval = 0.1
        reverseval = 0.2
        centerval = self.driving.servoCenterValue
        noneval = self.driving.motorOffValue
        #8 actions, 0-7.#0 forward, 1 forwardleft, 2 forwardright, 3 backward, 4 backwardleft, 5 backwardright, 6 stop/brake, 7 none
        self.servoActionValues = [centerval, leftval, rightval, centerval, leftval, rightval, centerval, centerval]
        self.motorActionValues = [forwardval, forwardval, forwardval, reverseval, reverseval, reverseval, 0, noneval]#0->braking
        self.paction = 7
    def getDistances(self):
        return [self.leftUltra.distance, self.rightUltra.distance, self.centerUltra.distance]
    def getPosition(self):
        return self.positioner.get()
    def doAction(self, action=7):
        self.driving.setServo(self.servoActionValues[action])
        self.driving.setMotor(self.motorActionValues[action])
    def getState(self):
        return np.array([self.getDistances() + [self.paction == act for act in range(8)] + self.getPosition()])
    def cleanup(self):
        self.driving.releaseAll()
        self.gpsUpdater.running = False #tell the GPS Updating thread to shut down
        self.ultraUpdater.running = False #same for Ultrasonic monitoring thread
        self.gpsUpdater.join() #wait for the GPS Updating thread to finish what it is doing
        self.ultraUpdater.join() #likewise for ultrasonic modules
        GPIO.cleanup()


import theano.tensor as T
from keras.models import Sequential
from keras.layers import Activation, LSTM, TimeDistributedDense
import dill
class Network:
    def __init__(self):
        #commented out section below takes about 7 minutes (stateful RNN) to 12 minutes (stateless)
        #for Theano to compile when Raspberry Pi on Turbo overclock
        """self.model = Sequential()
        self.model.add(LSTM(input_dim=13, output_dim=300, return_sequences=True, batch_input_shape=(1, 1, 13), stateful=True))
        self.model.add(TimeDistributedDense(input_dim=300, output_dim=8))
        self.model.add(Activation("linear"))
        self.model.load_weights('weights.h5')
        self.model.compile(loss=self.mse_rl, optimizer="sgd")
        #self.pstates = np.zeros((0,13))#used for stateless RNN"""
        #Instead, reading model from file takes about 2.5 minutes.
        self.model = dill.load(open("/home/pi/avc/model.dill", "rb"))
    def predict(self, state):
        #self.pstates = np.concatenate((self.pstates, state)) #these two lines also used for stateless RNN
        #return self.model.predict_on_batch(self.pstates.reshape(1, self.pstates.shape[0], self.pstates.shape[1]))[0][0, -1, :] * 300
        return self.model.predict_on_batch(state.reshape(1, state.shape[0], state.shape[1]))[0][0, -1, :] * 300
    @staticmethod
    def mse_rl(y_true, y_pred):
        #We don't really need this here unless you compile your model instead of pickling it.
        #this 
        care = T.nonzero(T.eq(T.isnan(y_true),0))
        return T.mean((y_pred[care] - y_true[care]) ** 2).sum()
    @staticmethod
    def e_greedy(values, epsilon = 0.0):
        #a "policy": tells the robot how to select its next action based on how good it thinks each action is.
        #e_greedy takes the best action (1-epsilon)*100% of the time, and a purely random action epsilon*100% of the time.
        #epsilon=0 means it always takes what it thinks is the best action
        r = np.random.rand()
        if r < epsilon:
            ouraction = np.random.randint(0, values.size)
        else:
            ouraction = np.argmax(values)
        return ouraction, values[ouraction]

def main():
    robot = PhysicalReactor() #initializes sensors and actuators: left, right, center Ultrasonics, gps, servo, and motor
    try:
        brain = Network()
        policy = Network.e_greedy
        #wait for everything to get setup.
        readyLedPin = 15
        GPIO.setup(readyLedPin, GPIO.OUT)
        while not robot.gps.hasfix:
            GPIO.output(readyLedPin, GPIO.LOW)
            time.sleep(0.85)
            GPIO.output(readyLedPin, GPIO.HIGH)
            time.sleep(0.15)
        #turn on led to show we are ready to go.
        GPIO.output(readyLedPin, GPIO.HIGH)
        #wait for button press to start a round
        startswitchpin = 16
        GPIO.setup(startswitchpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)#normally connected to +3 v
        GPIO.wait_for_edge(startswitchpin, GPIO.FALLING)#wait til it's connected to ground (switch closed)
        ledstate = False
        #each run at the NRC has a time limit of 5 minutes. So,
        stoptime = time.time() + 5 * 60 #after 5 minutes, let's just shut down.
        #also shut down if someone flips the switch back to the off position.
        startTimeStep = time.time()
        while startTimeStep < stoptime and 0==GPIO.input(startswitchpin):
            startTimeStep = time.time()
            GPIO.output(readyLedPin, ledstate)
            ledstate = not ledstate
            state = robot.getState()
            values = brain.predict(state)
            action, value = policy(values)
            robot.doAction(action)
            #wait the remainder of 1 second, so each cycle takes 1 second total.
            time.sleep(max(0, startTimeStep + 1 - time.time()))
        robot.cleanup()
        call("sudo shutdown -h now", shell=True)
    except KeyboardInterrupt:
        robot.cleanup()

if __name__ == "__main__":
    main()
