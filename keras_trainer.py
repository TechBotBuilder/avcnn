from experienceCollector import *
import theano.tensor as T
from keras.models import Sequential
from keras.layers import Dense, Activation, LSTM, SimpleRNN, Dropout, TimeDistributedDense
from keras.optimizers import SGD
from keras.models import model_from_json

episodelen = 300
insize = 13
outsize = 8
h_size = 256

def mse_rl(y_true, y_pred):
    care = T.nonzero(T.eq(T.isnan(y_true),0))
    return T.mean((y_pred[care] - y_true[care]) ** 2).sum()

"""model = Sequential()
#model.add(LSTM(output_dim=h_size, input_dim=insize, input_length=episodelen, return_sequences=True))
model.add(LSTM(output_dim=outsize, input_shape=(
#model.add(SimpleRNN(output_dim=outsize, return_sequences=True))
model.add(Activation("linear"))

#model.load_weights('my_model_weights.h5')

model.compile(loss=mse_rl, optimizer="rmsprop")

#model = model_from_json(open('my_model_architecture.json').read())
"""

model = Sequential()
model.add(LSTM(input_dim=insize, output_dim=300, return_sequences=True))
#model.add(LSTM(input_dim=300, output_dim=500, return_sequences=True))
model.add(Dropout(0.2))
#model.add(LSTM(input_dim=500, output_dim=13, return_sequences=True))
model.add(TimeDistributedDense(input_dim=300, output_dim=outsize))
model.add(Activation("linear"))

model.load_weights('2layer_rescaled_b.h5')

#sgd = SGD(lr=0.01, momentum=0.9, decay=0.0, nesterov=True)
model.compile(loss=mse_rl, optimizer="adam")#sgd)#"rmsprop")

xp = ExperienceCollector()

inputs = xp.xdata
outputs = xp.ydata
actions = xp.actdata

xdata = inputs

ydata = np.full((inputs.shape[0], 300, outsize), np.nan)
for episode in range(inputs.shape[0]):
    for step in range(300):
      ydata[episode, step, actions[episode, step]] = outputs[episode, step]/300# ':' was actions[episode, step]

model.fit(xdata, ydata, nb_epoch=200, batch_size=32, verbose=1)

#json_string = model.to_json()
#open('my_model_architecture.json', 'w').write(json_string)
model.save_weights('2layer_rescaled_c.h5')

print(model.predict(xdata)[0, :, 0])

