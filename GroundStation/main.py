# Ground Station for UHA CRW USLI 2021
import serial.tools.list_ports
import sys
import time
portlist = [comport.name + ' - ' + comport.description for comport in serial.tools.list_ports.comports()]

print('Currently Open Ports:')
for port in portlist: print(port)

##port = 'COM3' # Hardcoded, will implement way to chose later
##baud = 115200
##xbee = serial.Serial(port,baud)
##
##
##xbee.write(b"Hello?\n")
buff = []
sys.stdout.write("Welcome to the Payload Drop Test UI\n")
sys.stderr.write("In order to release, type Release\n")
print('In order to lock, type "Lock"')

sys.stdout.flush()
sys.stderr.flush()
while True:
    time.sleep(0.5)

##    while xbee.in_waiting > 0: # While their are bytes waiting to be read
##        char = xbee.read()
##        print(char) # DEBUG
##        print(buff) # DEBUG
##        buff.append(char) # Appends the read character to the message buffer
##
##        # if the message is finishe (marked by return character) then decode and display message
##        if char in [b'\r', b'\n']:
##            # Flag Structure for task management?
##
##            mess = ''
##            for char in buff:
##                mess += char.decode('utf-8')
##            print(mess)
##            buff = []
