from time import sleep
from networktables import NetworkTables
from statistics import median
import numpy as np
import matplotlib.pyplot as plt

NetworkTables.initialize(server="10.11.89.2")
sd = NetworkTables.getTable("SmartDashboard/Test")
sleep(3)

def outliers_and_mean(data, m = 2):
    data = np.asarray(data)
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return np.mean(data[s<m])

data = {}
while not sd.getBoolean("Done", False):
    if sd.getBoolean("Success", False):
        target = sd.getNumber("Target", 0)
        try:
            data[target]
        except KeyError:
            data[target] = []
        data[target].append(sd.getNumber("AngularVelocity", 0))
    sleep(0.05)

print("Processing data...")
total = 0
for value in data.values():
    total += len(value)
print(f"{total} data points gathered.")


x = [key for key in data.keys()]
y = [outliers_and_mean(value) for value in data.values()]
print(x)
print(y)
print("Line of best fit: " + str(np.poly1d(np.polyfit(x, y, 1))))

plt.plot(x, y)

plt.xlabel('Input Degrees')
plt.ylabel('Output Degrees')
plt.title('Angle Characterization')
plt.grid(True)
plt.show()