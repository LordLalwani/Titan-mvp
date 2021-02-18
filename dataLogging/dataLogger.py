import serial
import json
import time
from datetime import datetime

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
file.write(data + "\n")
file.close()

# Starts a timer and collects data until timer reaches value of sampleDataTimerLimitInSeconds
startTime = time.time()
collectData = True

while (collectData):
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    file = open(fileName, "a")
    file.write(data + "\n")

    print(time.time() - startTime)
    if(time.time() - startTime >= sampleDataTimerLimitInSeconds):
        collectData = False


print("\nData collection complete!")
file.close()
