import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

fname = 'test.csv'

df = pd.read_csv(fname)
data = df.to_numpy()

print(np.shape(data))

fft_result = np.fft.fft(data[:,0])

plt.figure()
plt.plot(np.abs(fft_result))
plt.grid(True)
plt.show()