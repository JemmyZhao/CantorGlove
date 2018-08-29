import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-5, 5, 100)
y = 0.5*x**2 - np.sin(x)

fig = plt.figure()
plt.plot(x, y)
plt.show()