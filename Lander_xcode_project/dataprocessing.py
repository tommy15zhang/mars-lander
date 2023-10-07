import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('/Users/jiahaozhang/Desktop/h_and_ve_data.txt')
plt.figure(1)
plt.clf()
plt.ylabel("velocity (m/s)")
plt.grid()
actual_descent_rate = -results[:, 1]
target_descent_rate = -1*(0.5+results[:, 0]*0.01342)
plt.plot(actual_descent_rate, label='actual descent rate')
plt.plot(target_descent_rate, label='target descent rate')
plt.legend()
plt.show()