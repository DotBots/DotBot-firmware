import pandas
from matplotlib import pyplot
df = pandas.read_csv("magnetometer.csv")
pyplot.plot(df.X, df.Y, 'bo')
pyplot.plot(df.Y, df.Z, 'ro')
pyplot.plot(df.X, df.Z, 'go')
pyplot.show()
