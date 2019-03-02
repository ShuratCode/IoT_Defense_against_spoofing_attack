import serial
import sys
import csv
from pathlib import Path
import time

''' get the arguments. at the first place should be the serial port, at the second should be a relative/full path
for the csv file'''
if len(sys.argv) == 3:
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
    # title = "Max Mean Min StandardDev AvgDev"
    # title = "MSE"
    title = "Not_attack Under_attack"
    lineList = [title.split()]
    timeout = time.time() + 100
    attacks = 0
    non_attacks = 0
    ''' loop all the time and collect the data. Insert the data to csv file.'''
    while True:
        if time.time() > timeout:
            break
        line = s.readline()
        line = line.decode('utf-8')
        if line != '':
            if line == 'No attack\n':
                non_attacks += 1
            elif line == 'Under attack\n':
                attacks += 1
    summary = str(non_attacks) + ' ' + str(attacks)
    lineList.append(summary.split())
    csv_writer.writerows(lineList)    
else:
    print('The number of arguments needed to be 3, the script, the serial to listen to and the path for the csv file')