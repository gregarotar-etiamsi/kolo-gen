import pandas as pd
from matplotlib import pyplot as plt
import numpy as np


columns = ["gForceX","gForceY","gForceZ","rotX","rotY","rotZ","rawWeight"]
df = pd.read_csv("mqtt.log", usecols=columns)

squared_values = df[['gForceX', 'gForceY', 'gForceZ']].apply(np.square)

sum_of_squares = squared_values.sum(axis=1)

result = np.sqrt(sum_of_squares)

plt.plot(result)
plt.show()



