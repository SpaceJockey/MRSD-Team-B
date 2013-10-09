from serial import *
import time
import struct

port = "COM14"
baudRate= 9600
ser = Serial(port, baudRate, timeout=0, writeTimeout=0)
#timeout=0,
print("Opening Serial Port...");
time.sleep(3)

packet = b''

thetaL = [0,]*13
thetaL[2]=4
thetaL[1]=0

thetaL[0]=0xDEAD
thetaL[12] = 0xBEEF

# Fix checksum later
print(thetaL);
txBuf = ctypes.create_string_buffer(24)
struct.pack_into("=HBBHHHHHHHHHH", txBuf, 0, *tuple(thetaL))
ser.write(txBuf)
time.sleep(1);
thetaL[1]=8
print(thetaL);
struct.pack_into("=HBBHHHHHHHHHH", txBuf, 0, *tuple(thetaL))
ser.write(txBuf)
time.sleep(1);
thetaL[1]=7
thetaL[2]=1
print(thetaL);
struct.pack_into("=HBBHHHHHHHHHH", txBuf, 0, *tuple(thetaL))
ser.write(txBuf)

#servo to computer controlled value
time.sleep(1);
thetaL[1]=0
thetaL[2]=0
thetaL[3]=0xffff
print(thetaL);
struct.pack_into("=HBBHHHHHHHHHH", txBuf, 0, *tuple(thetaL))
ser.write(txBuf)

	
# getPacket: Reads in serial inputs and attempts to parse packets
def getPacket():
    root.after(100,getPacket)
    global packet
    packet += ser.read(100)
    ser.flushInput()
    head = packet.find(b'\xAD\xDE')
    if head == -1:
        return False

    if len(packet)<24+head:
        return False
    
    try:
        thetas = struct.unpack_from("=HBBHHHHHHHHHH",packet,head)
        packet = b''
        if(isValid(thetas)):
            processPacket(thetas)
            return True
        else:
            return False
    except struct.error:
        return False