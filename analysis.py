import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

husky_ur5 = "csv/husky_ur5_push.csv"
ur5 = "csv/ur5_push.csv"
husky_ur5_3dof = "csv/husky_ur5_push_3dof.csv"

def get_value(file_name):

    value = []

    with open(file_name, newline='') as csvfile:

        rows = csv.DictReader(csvfile)

        for row in rows:
            value.append(float(row["Value"]))

    return np.array(value)

husky_ur5_value = get_value(husky_ur5)
ur5_value = get_value(ur5)
husky_ur5_3dof_value = get_value(husky_ur5_3dof)

x1 = len(husky_ur5_value)
x2 = len(ur5_value)
x3 = len(husky_ur5_3dof_value)

x1_axis = np.array([x for x in range(x1)])
x2_axis = np.array([x for x in range(x2)])
x3_axis = np.array([x for x in range(x3)])
plt.grid(True)

plt.plot(x2_axis, ur5_value, "b", label="DoorGym")
plt.plot(x1_axis, husky_ur5_value, "r", label="6 joints")
plt.plot(x3_axis, husky_ur5_3dof_value, "g", label="3 DOF")
plt.legend(loc="lower right")

plt.title("Pull")
plt.ylabel("Door open success rate")
plt.xlabel("Steps")

plt.show()
