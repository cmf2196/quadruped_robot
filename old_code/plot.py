
import csv
import matplotlib.pyplot as plt
import numpy as np


def read_csv(file_path):
    with open(file_path, newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
        return data


# all paths from which to collect data
paths = ["data/ea_2_100k_learning.csv", "data/hc_learning_curve_100k.csv", "random_30k_curve.csv"]

# collect the data in arrays
all_data = []
for path in paths:
    data = read_csv(path)
    data_formatted = []
    for entry in data:
        val = float(entry[0])
        data_formatted.append(val)
    all_data.append(data_formatted)

#print(all_data)
ea = np.asarray(all_data[0])
hc = np.asarray(all_data[1])
ra = np.asarray(all_data[2])
x = np.arange(100000)

plt.plot(x, ea, 'r--', x , hc , 'b--', ra, "g--")
plt.xlabel("Evaluations")
plt.ylabel("Fitness")
plt.title("Learning Curves")
plt.legend(["Evolutionary Algorithm", "Hill Climber", "Random"])
plt.show()