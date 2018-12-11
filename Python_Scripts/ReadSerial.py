import serial
import sys
import csv
from pathlib import Path

''' get the arguments. at the first place should be the serial port, at the second should be a relative/full path
for the csv file'''
serialPort = sys.argv[1]
csv_file_path = sys.argv[2]
s = serial.Serial(serialPort, 9600, timeout=1)

# Open csv file
file = Path(csv_file_path)
if not file.exists():
    print('There is not such file please check second argument')
    sys.exit(1)

csv_file = open(file, 'w', newline='')
csv_writer = csv.writer(csv_file)

''' loop all the time and collect the data. Insert the data to csv file.'''
while True:
    line = s.readline()
    line = line.decode('utf-8')
    csv_writer.writerow(line)
