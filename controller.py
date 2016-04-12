import sys
import numpy as np
import dill
from constants import *
from watcher import RewardWatcher

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

#0 forward, 1 forwardleft, 2 forwardright, 3 backward, 4 backwardleft, 5 backwardright, 6 stop/brake, 7 none
#(force, steer, brake)
actions = [(50, 0, 0), (50, -0.349, 0), (50, 0.349, 0),
         (-50, 0, 0), (-50, -0.349, 0), (-50, 0.349, 0),
         (0, 0, 5), (0, 0, 0)]

class RewardAccumulator:
  def __init__(self):
    self.reward = 0
    self.won = False
    self.failed = False
    self.doquit = False
  def getreward(self):
    return self.reward
  def setreward(self, reward):
    self.reward = reward
  def addreward(self, reward):
    self.reward += reward
  def clear(self):
    self.reward = 0

award = RewardAccumulator()

watchdog = RewardWatcher(award)

def parse_infrared(data): #ultrasonic sensors generally return distance to nearest object
  return min(data['range_list']) / (SCALINGFACTOR * 4.5) #rescale w/in 0-1

class Map:
  def __init__(self, bigness, topleftcoord, flipxaxis=False):
    self.tl = topleftcoord
    self.bigness = bigness
    self.internalstate = [0.0,0.0]
    self.flipxaxis = flipxaxis
  def update(self, coord):
    #self.clear()
    y = self.tl[1] - coord[1]
    x = self.tl[0] - coord[0]
    if(self.flipxaxis):
      x = -x
    y = y/self.bigness
    x = x/self.bigness
    self.internalstate = [x,y]
  def clear(self):
    self.internalstate = [0.0, 0.0]
  def get(self):
    return self.internalstate

class State:
  def __init__(self):
    self.init_time = -1
    self.currenttime = 0
    self.irLstate = 0
    self.irRstate = 0
    self.irCstate = 0
    self.direction = 0
    self.helperforgpssodontdo10hzbutrather1hz = 1
    self.action = len(actions)-1
    bigness = 64.2 * SCALINGFACTOR
    self.minimap = Map(bigness, [-466.57178, 574.63947], True)
  def updategps(self, gps):
    watchdog.update(gps)
    if self.helperforgpssodontdo10hzbutrather1hz >= 10:
      x = SCALINGFACTOR * np.random.randn() * 2.5 / 10
      y = SCALINGFACTOR * np.random.randn() * 2.5 / 10
      self.minimap.update((gps['x'] + x, gps['y'] + y))
      self.helperforgpssodontdo10hzbutrather1hz = 1
    else:
      self.helperforgpssodontdo10hzbutrather1hz += 1
  def updateclock(self, clock):
    clockstate = clock
    if self.init_time == -1:
      self.init_time = clockstate['timestamp']
    self.currenttime = clockstate['timestamp'] - self.init_time
  def updateirL(self, irL):
    self.irLstate = parse_infrared(irL)
  def updateirR(self, irR):
    self.irRstate = parse_infrared(irR)
  def updateirC(self, irC):
    self.irCstate = parse_infrared(irC)
  def updateaction(self, action):
    self.action = action
  def reset(self):
    self.init_time = -1
    self.action = len(actions)-1
    self.minimap.clear()
  def state(self):
    return np.array([ [self.irLstate, self.irRstate, self.irCstate,
      self.action==0,self.action==1,self.action==2,self.action==3,self.action==4,self.action==5,self.action==6,self.action==7]
      + self.minimap.get()])

stater = State()

from experienceCollector import ExperienceCollector

experiencer = ExperienceCollector()

def e_greedy(values, epsilon = 0.1):
  r = np.random.rand()
  if r < epsilon:
    ouraction = np.random.randint(0, values.size)
  else:
    ouraction = np.argmax(values)
  return ouraction, float(values[ouraction])

# Adapted from http://stackoverflow.com/a/19760118/1274613
def boltzmann(values):
  weights = np.exp(np.minimum(10,np.maximum(-5, np.squeeze(np.asarray(values - np.mean(values))))))
  cs = np.cumsum(weights)
  index = cs.searchsorted(np.random.random() * cs[-1], 'right')
  return index, values[index]

def goforward(values):
  return 0, values[0]

import _thread as thread
import time
try:
  from msvcrt import getch  # try to import Windows version
except ImportError:
  def getch():   # define non-Windows version
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
def keypress():
  while True:
    command = getch()
    if (command == "q"):
      award.doquit = True
      print("Quitting and saving network...")
    time.sleep(0.001)
thread.start_new_thread(keypress, ())
###stole ^^ from http://rosettacode.org/wiki/Keyboard_input/Keypress_check#Python

import theano.tensor as T
from keras.models import Sequential
from keras.layers import Dense, Activation, LSTM, SimpleRNN, Dropout, TimeDistributedDense
#from keras.optimizers import SGD
def mse_rl(y_true, y_pred):
    care = T.nonzero(T.eq(T.isnan(y_true),0))
    return T.mean((y_pred[care] - y_true[care]) ** 2).sum()

h_size = 300
UPDATE_FREQUENCY = 50
MEMORY_SIZE = 1000
MINIBATCH_SIZE = 100
BACKUPSCHEDULE = UPDATE_FREQUENCY * 10
UPDATE_EPOCHS = 2

model = Sequential()
model.add(LSTM(input_dim=stater.state().size, output_dim=h_size, return_sequences=True))
model.add(TimeDistributedDense(input_dim=h_size, output_dim=len(actions)))
model.add(Activation("linear"))

model.load_weights('v5/4.h5')
#model.load_weights('v4/v40.h5')

#sgd = SGD(lr=0.01, momentum=0.9, decay=0.0, nesterov=True)
model.compile(loss=mse_rl, optimizer="adam")

policy = boltzmann

####BELOW THIS IS WHERE THE ACTIONS START

with Morse() as simu:
  simu.reset()
  motion = simu.robot.motion
  motion.publish({"force": 0, "steer": 0, "brake": 0})
  gps = simu.robot.gps
  clock = simu.robot.clock
  irL = simu.robot.infraredLeft
  irR = simu.robot.infraredRight
  irC = simu.robot.infraredCenter
  
  gps.subscribe(stater.updategps)
  clock.subscribe(stater.updateclock)
  irL.subscribe(stater.updateirL)
  irR.subscribe(stater.updateirR)
  irC.subscribe(stater.updateirC)
  simu.sleep(1.1)
  init_time =  stater.currenttime
  prev_clock_tick = init_time
  currenttime = 0
  state = stater.state()
  prewards = []
  pactions = []
  pstates = np.zeros((0,state.size))
  pstates = np.concatenate((pstates, state))
  guess = model.predict_on_batch(pstates.reshape(1, pstates.shape[0], pstates.shape[1]))
  action, value = policy(guess[0][0, -1, :] * 300)#########
  pactions.append(action)
  
  truedonut = 0
  num_runs = 0
  niet = 0
  
  now = simu.time()
  
  while not award.doquit:
    niet += 1
    
    lel = False
    if currenttime >= 300:
      award.failed = True
      lel = True
    
    stater.updateaction(action)
    f,s,b = actions[action]
    motion.publish({"force": f, "steer": s, "brake": b})
    waits = 0
    while (simu.time() - now) < 0.95:
      simu.sleep(0.05)
      waits+=1
    now = simu.time()
    
    currenttime = stater.currenttime
    award.addreward(-1)
    
    reward = award.getreward()
    prewards.append(reward)
    award.clear()
    if reward > 0:
      print(reward)
    
    next_state = stater.state()
    pstates = np.concatenate((pstates, next_state))
    guess = model.predict_on_batch(pstates.reshape(1, pstates.shape[0], pstates.shape[1]))
    next_action, next_value = policy(guess[0][0, -1, :] * 300)######
    
    pactions.append(next_action)
    truedonut += reward
    
    prev_clock_tick = currenttime
    action = next_action
    state = next_state
    value = next_value
    
    if award.won or award.failed:
      motion.publish({"force": 0, "steer": 0, "brake": 0})
      num_runs += 1
      prewards.append(0)
      targets = []
      prewards.reverse()
      targets = np.flipud(np.cumsum(prewards))
      experiencer.update(pstates, pactions, targets)
      if(num_runs % UPDATE_FREQUENCY == 0):
        print("Reward: {}".format(truedonut))
        xdata = experiencer.xdata
        outputs = experiencer.ydata
        choices = experiencer.actdata
        ydata = np.full((xdata.shape[0], 300, len(actions)), np.nan)
        for episode in range(xdata.shape[0]):
          for step in range(300):
            ydata[episode, step, choices[episode, step]] = outputs[episode, step] / 300
        model.fit(xdata, ydata, nb_epoch=UPDATE_EPOCHS, batch_size=MINIBATCH_SIZE, verbose=0)
        if (experiencer.xdata.shape[0] > MEMORY_SIZE):
          experiencer.reset(UPDATE_FREQUENCY)
      if (num_runs % BACKUPSCHEDULE == 0):
        model.save_weights('v5/backupV5.{}.h5'.format(5050+num_runs))
      award.failed = False
      award.won = False
      simu.reset()
      stater.reset()
      watchdog.reset()
      pactions = []
      simu.sleep(0.5)
      init_time = stater.init_time
      currenttime = stater.currenttime
      prev_clock_tick = stater.currenttime
      state = stater.state()
      prewards = []
      pstates = np.zeros((0,state.size))
      pstates = np.concatenate((pstates, state))
      guess = model.predict_on_batch(pstates.reshape(1, pstates.shape[0], pstates.shape[1]))
      action, value = policy(guess[0][0, -1, :] * 300)#########
      pactions.append(action)
      niet = 0
      now = simu.time()
      truedonut = 0
  print("Number of episodes: " + str(num_runs))
model.save_weights('v5/5.h5')
