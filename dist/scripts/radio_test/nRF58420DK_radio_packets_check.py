#test_radio.py

import serial
import time
import serial.tools.list_ports
import numpy as np

serialport_rx = 'COM9' 
baud = 1000000
packets = []
RSSI = []
miss=0;
x=['0','1','0','0']
cnt=0
crc=0
error=0


myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
nRF_rx = serial.Serial(serialport_rx, baud, timeout=1)
serialport_rx  = [port for port in myports if serialport_rx  in port ][0]



while nRF_rx.isOpen() == False : #wait until connection
    nRF_rx.Open()
 
while serialport_rx in myports : #create packets list
    x[3]=x[2]
    x[2]=x[1]
    x[1]=x[0]
    x[0]=nRF_rx.read()
    print(x[0]) #debug
    
    if x[0]==b'' :
        miss+=1
        
    if x[0]==b'\x00' :  # end of packet
        if (x[1]==b'\x00') : #the packet number is x00 and we wait to have the RSSI 
            continue
            
        if (x[2]==b'\x00' and x[3]==b'\x00' ) : #the packet number is x00 and we validate the packet
            print(ord(x[2]))
            RSSI.append(ord(x[1]))
            packets.append(ord(x[2]))
        elif not ( x[1]==b'C' and x[2]==b'R' and x[3]==b'C' ) :# the packet has not an CRC error
            print(ord(x[2]))
            RSSI.append(ord(x[1]))
            packets.append(ord(x[2]))
    
    else :
        if x[0]==b'C' : # the packet had and CRC error
            if  x[1]==b'R' and x[2]==b'C' :
                print("CRC")
                packets.append("CRC")
        
    time.sleep(0.1)
    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]


#check list
for i in range(len(packets)-1) :
    
    if not isinstance(packets[i], int) : # if not int means string = CRCFAIl
        crc+= 1  
        

print("the first and last packets missing will not be know") 
result = 'Total Packets Receive :' + str(len(packets))
print(result) 
result = 'Missing Packets :' + str(miss)
print(result) 
result = 'CRC Packets :' + str(crc)
print(result) 
result = 'RSSI dBm :' + str( np.int8((sum(RSSI))/len(RSSI)))
print(result) 