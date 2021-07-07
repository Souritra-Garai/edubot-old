import serial
import csv

# Open the file in the write mode
fileObj = open('scripts/PWM_Pulse_Signal.csv', 'w')

# Create the csv writer
writer = csv.writer(fileObj)

# Header line
header = ['Time', 'PWM', 'Angular Velocity']

# Write a row (header line) to the csv file
writer.writerow(header)

serialPort = serial.Serial('/dev/ttyACM0', 115200)

# List of all data
listData = []

# Variable for while and if loop
check = True

while(check):

    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):

        if (check):

            serialString = serialPort.readline().decode("ascii")
            listData.append(serialString[0:-2]) # Ommit new line character from end

            # End while loop when max time is reached
            if (serialString[0:4]=='4980'):
                check = False


# Loop for writing into csv file
for i in range(len(listData)):

    # Split the string at each tab - '\t'
    temp = listData[i].split('\t')
    writer.writerow(temp)

# Close the file object
fileObj.close()