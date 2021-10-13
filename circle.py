import numpy as np
import matplotlib.pyplot as plt
r = 0.1
a, b = (0.3,0)
theta = np.arange(0, 2*np.pi, 0.01)
x = a + r * np.cos(theta)
y = b + r * np.sin(theta)
print(x[0],type(x[0]))

fig = plt.figure() 
axes = fig.add_subplot(111) 
axes.plot(x, y)

axes.axis('equal')
plt.show()