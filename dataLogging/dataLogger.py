import serial

arduino_port = "/dev/cu.usbmodem141201"  # serial port of Arduino
baud = 115200  # arduino uno runs at 9600 baud
fileName = "./logs/datalog.csv"  # name of the CSV file generated

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port:" + arduino_port)
file = open(fileName, "a")
print("Created file")

# display the data to the terminal
getData = str(ser.readline().decode())
data = getData[0:][:-2]

# add the data to the file
file = open(fileName, "w")  # append the data to the file
file.write(data + "\\n")  # write data with a newline

# close out the file
file.close()

samples = 20  # how many samples to collect
print_labels = False
line = 0  # start at 0 because our header is 0 (not real data)
while line <= samples:
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    print(data)

    file = open(fileName, "a")
    file.write(data + "\n")  # write data with a newline
    line = line+1

print("Data collection complete!")
file.close()
