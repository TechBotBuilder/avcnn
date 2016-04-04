import sys
sys.setrecursionlimit(10000)

import theano.tensor as T
from keras.models import Sequential
from keras.layers import Activation, LSTM, TimeDistributedDense

def mse_rl(y_true, y_pred):
    care = T.nonzero(T.eq(T.isnan(y_true),0))
    return T.mean((y_pred[care] - y_true[care]) ** 2).sum()

model = Sequential()
model.add(LSTM(input_dim=13, output_dim=300, return_sequences=True, batch_input_shape=(1, 1, 13), stateful=True))
model.add(TimeDistributedDense(input_dim=300, output_dim=8))
model.add(Activation("linear"))
model.load_weights('weights.h5')
model.compile(loss=mse_rl, optimizer="sgd")
print("done building model. Packing it...")
import dill
with open("model.dill", "wb") as f:
    dill.dump(model, f)