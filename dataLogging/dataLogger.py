import plotly.express as px
import pandas as pd
import serial
import json
import time
from datetime import datetime
import math
import numpy as np
import plotly.graph_objects as go
import pandas

sampleDataTimerLimitInSeconds = float(input(
    "How long do you want to collect data for (seconds)?: "))

# Reads config
with open('/Users/s121003/Documents/Arduino/TitanMVP/.vscode/arduino.json') as f:
    arduinoData = json.load(f)

# connect to serial port
ser = serial.Serial(arduinoData['port'], arduinoData["baudRate"])

# creates data log file based on current date time
fileName = "./logs/"+datetime.now().strftime("%d-%m-%Y-%H:%M:%S") + ".csv"
file = open(fileName, "a")
print("Created file")

# read data from serial line
getData = str(ser.readline().decode())
data = getData[0:][:-2]

# adds headers externally from sample data
file = open(fileName, "w")
file.write("TIME,ACCEL" + "\n")
file.close()

# Starts a timer and collects data until timer reaches value of sampleDataTimerLimitInSeconds
startTime = time.time()
collectData = True

while (collectData):
    currentTime = round(time.time() - startTime, 2)
    if(currentTime >= sampleDataTimerLimitInSeconds):
        collectData = False

    if(str(ser.readline().decode())):
        getData = str(ser.readline().decode())
        data = getData[0:][:-2]
        if(int(data) >= 0):
            file = open(fileName, "a")
            file.write(str(currentTime) + "," + data + "\n")

print("\nData collection complete!")
file.close()


plotData = pd.read_csv(fileName)
fig = px.line(plotData, x='TIME', y='ACCEL',
              title='Titan linear acceleration over time')
fig.show()
