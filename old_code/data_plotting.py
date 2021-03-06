'''
Joshua Katz
5/3/2020
Given a set of fitnesses, this script will make fitness curve plots.
'''

import csv
import matplotlib.pyplot as plt
import numpy as np


def read_csv(file_path):
    with open(file_path, newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
        return data


# all paths from which to collect data
paths = ["data\\ea_2_100k_learning.csv", "data\\hc_learning_curve_100k.csv", "random_30k_curve.csv"]

# collect the data in arrays
all_data = []
for path in paths:
    data = read_csv(path)
    data_formatted = []
    for entry in data:
        data_formatted.append(entry[0])
    all_data.append(data_formatted)

# make plots
for data in all_data:
    print(data)
    print(len(data))
    plt.plot(data)
    plt.yticks(np.arange(-1, 10, 0.5))
    plt.show()
