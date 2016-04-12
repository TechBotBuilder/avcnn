import numpy as np
import dill

# Adapted from http://stackoverflow.com/a/19760118/1274613
def boltzmann(values, num=1, temperature=1):
  weights = np.exp(np.squeeze(np.asarray(values - np.mean(values))) / temperature)
  cs = np.cumsum(weights)
  index = np.zeros(num)
  for idx in range(num):
    index[idx] = cs.searchsorted(np.random.random() * cs[-1], 'right')
  return index

class ExperienceCollector:
  def __init__(self):
    self.load()
  def save(self, tofile="experience.pkl"):
    with open(tofile, "wb") as f:
      dill.dump([self.xdata, self.ydata, self.actdata], f)
  def load(self, fromfile="experience.pkl"):
    try:
      with open(fromfile, "rb") as f:
        self.xdata, self.ydata, self.actdata = self.data = dill.load(f)
    except IOError:
      self.reset()
  def reset(self, num_to_drop = -1):
    if num_to_drop == -1:
      self.xdata = np.zeros((0, 300, 13))
      self.ydata = np.zeros((0, 300))
      self.actdata = np.zeros((0, 300))
    else:
      #indexes_to_drop = np.random.randint(0, self.xdata.shape[0], size=num_to_drop)
      indexes_to_drop = boltzmann(-self.ydata[:, 0], num_to_drop, 3)
      self.xdata = np.delete(self.xdata, indexes_to_drop, axis=0)
      self.ydata = np.delete(self.ydata, indexes_to_drop, axis=0)
      self.actdata = np.delete(self.actdata, indexes_to_drop, axis=0)
  def update(self, pstates, pactions, targets):
    xnow = pstates.reshape(1, pstates.shape[0], pstates.shape[1])
    padding = np.zeros((1, 300-pstates.shape[0], pstates.shape[1]))
    xnow = np.concatenate((xnow, padding), axis=1)
    self.xdata = np.concatenate((self.xdata, xnow))
    ynow = targets.reshape(1, targets.size)
    padding = np.zeros((1, 300-targets.size))
    ynow = np.concatenate((ynow, padding), axis=1)
    self.ydata = np.concatenate((self.ydata, ynow))
    pactions = np.array(pactions)
    actnow = pactions.reshape(1, pactions.size)
    padding = np.zeros((1, 300-pactions.size))
    actnow = np.concatenate((actnow, padding), axis=1)
    self.actdata = np.concatenate((self.actdata, actnow))

