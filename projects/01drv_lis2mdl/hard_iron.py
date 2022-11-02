#!/usr/local/bin/python3

import csv
import sys

reader = csv.reader(iter(sys.stdin.readline, ""), delimiter=",")
data = list(reader)

x = [float(row[0]) for row in data]
y = [float(row[1]) for row in data]
z = [float(row[2]) for row in data]

offset_x = (max(x) + min(x)) / 2
offset_y = (max(y) + min(y)) / 2
offset_z = (max(z) + min(z)) / 2

for row in data:
    corrected_x = float(row[0]) - offset_x
    corrected_y = float(row[1]) - offset_y
    corrected_z = float(row[2]) - offset_z

    print(",".join(format(value, ".15f") for value in [corrected_x, corrected_y, corrected_z]))
