from Tkinter import *
from serial import *
import time
import struct
import random
from copy import copy

port = "COM5"
baudRate= 9600
ser = Serial(port, baudRate, writeTimeout=0)
#timeout=0,
print "Opening Serial Port..."
time.sleep(1)

thermalVal, forceVal, lightVal, rangeVal,potVal = (1.0,)*5
data = [0.0,]*100
packet = b''

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
            print thetas
            processPacket(thetas)
            return True
        else:
            return False
    except struct.error:
        return False

def sendPacket(thetas):
    thetas[0]=0xDEAD
    thetas[12] = 0xBEEF
    print thetas
    # Fix checksum later
    txBuf = ctypes.create_string_buffer(24)
    struct.pack_into("=HBBHHHHHHHHHH", txBuf, 0, *tuple(thetas))
    ser.write(txBuf)
    
# isValid: Checks the validity of a given packet     
def isValid(thetas):      
    if thetas[0]!=0xDEAD:
        return False
    else: 
        return True
                
def processPacket(thetas):         
    global thermalVal, forceVal, lightVal, rangeVal, potVal, data, plotWin, wChan,old_plot
    thermalVal = thetas[10]
    potVal = thetas[3]
    
def updateSensors():
    global thermalVal, forceVal, lightVal, rangeVal, potVal, data, plotWin
    global wChan, old_plot, servo_chan_val, old_servo, dc_chan_val, old_dc, stepper_chan_val, old_stepper

    # UPDATE SENSOR READOUTS
    thermalVar.set("%4.3f" % thermalVal)
    forceVar.set("%4.3f" % forceVal)
    lightVar.set("%4.3f" % lightVal)
    rangeVar.set("%4.3f" % rangeVal)
    potVar.set("%4.3f" % potVal)

    
    if old_plot != wChan.get():
        old_plot = wChan.get()
        if old_plot=="Local":
            data = [0.0,]*100
        elif old_plot=="Thermal":
            data = [thermalVal,]*100
        elif old_plot=="Force":
            data = [forceVal,]*100
        elif old_plot=="Light":
            data = [lightVal,]*100
        elif old_plot=="Range":
            data = [rangeVal,]*100
        elif old_plot=="Potentiometer":
            data = [potVal,]*100
        
    data.pop(0)
    if old_plot=="Local":
        data.append(0.0)
    elif old_plot=="Thermal":
        data.append(thermalVal)
    elif old_plot=="Force":
        data.append(forceVal)
    elif old_plot=="Light":
        data.append(lightVal)
    elif old_plot=="Range":
        data.append(rangeVal)
    elif old_plot=="Potentiometer":
        data.append(potVal)
        
    updatePlot(data,plotWin)
    
    # UPDATE COMMAND VARIABLES
    thetaL = [0,]*13
    # Servo Motor
    if old_servo != servo_chan_val.get():
        old_servo = servo_chan_val.get()
        thetaL[2]=4
        if old_servo=="Test":
            thetaL[1]=8
        
    
    
        # DC    Motor
        if old_dc != dc_chan_val.get():
            old_dc = dc_chan_val.get()
    
        sendPacket(thetaL)
    
    root.after(100,updateSensors)
    
def updatePlot(data,plotWin):
    d = [105-100*n/(max(data)+0.0000001) for n in data]
    plotWin.delete("all")
    for i in range(1,100):
        plotWin.create_line((i-1)*2,d[i-1],i*2,d[i], fill="red")
    
# 
def servoSet():
    print servo_val.get()
    
# stepperMove: Fires whenever the "Move" button  is pressed in the stepper frame
def stepperMove():
    print stepper_val.get()

# dcUpdate: Fires whenever the dc velocity slider is moved
def dcUpdate(val):
    print val
    
# dcMove: Fires whenever the "Move" button is pressed in the dc frame
def dcMove():
    print ""

 
root = Tk()   
root.wm_title("MRSD Sensors and Motors Lab")
root.config(background="#DDDDDD")

# Left frame - Sensor Outputs

leftFrame = Frame(root, width=300,height=400)
leftFrame.grid(row=0, column=0, padx=10, pady=2)

# Sensor Outputs
Label(leftFrame, text="Sensor Outputs").grid(row=0, column=0, columnspan=2,padx=2, pady=2)
Label(leftFrame, text="Thermal:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Force:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Light:").grid(row=3,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Range:").grid(row=4,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Potentiometer:").grid(row=5,column=0,padx=2,pady=1, sticky=E)

thermalVar = StringVar()
forceVar = StringVar()
lightVar = StringVar()
rangeVar = StringVar()
potVar = StringVar()

Label(leftFrame, textvariable=thermalVar).grid(row=1,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=forceVar).grid(row=2,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=lightVar).grid(row=3,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=rangeVar).grid(row=4,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=potVar).grid(row=5,column=1,pady=1,stick=W)

sensor_options = ('Local','Thermal','Force','Light','Range','Potentiometer','Test')

# Plot Window
plotWin = Canvas(leftFrame, width=200, height=110,bg="#FFFFFF")
plotWin.grid(row=6,column=0,columnspan=2,padx=5,pady=5)
wChan = StringVar()
old_plot = ''
wChan.set(sensor_options[0])
Label(leftFrame,text="Plot Channel:").grid(row=7,column=0,padx=5,pady=5,stick=E)
w_chan = OptionMenu(leftFrame,wChan,*sensor_options)
w_chan.grid(row=7,column=1,padx=5)

# Right Frame - Motor Controls	
rightFrame = Frame(root, width=800, height = 500)
rightFrame.grid(row=0, column=1, padx=10, pady=2)
Label(rightFrame, text="Motor Controls").grid(row=0, column=0, columnspan=3, padx=10, pady=2)
Label(rightFrame, text="Channel Assignment").grid(row=0, column=4, columnspan=3, padx=10, pady=2)




# Servo Motor Controls
Label(rightFrame, text="Servo:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
servoFrame = Frame(rightFrame)
servoFrame.grid(row=1,column=1,columnspan=2,pady=10,sticky=W)
Label(servoFrame,text="Move to position:").grid(row=0,column=0, pady=5, sticky=W)
servo_val = StringVar()
servo_entry = Entry(servoFrame,textvariable=servo_val,width=6).grid(row=0,column=1)
servo_set = Button(servoFrame,text="Move",command=servoSet).grid(row=0,column=2,padx=5)
servo_chan_val = StringVar()
old_servo = ''
servo_chan_val.set(sensor_options[0])
servo_chan = OptionMenu(rightFrame,servo_chan_val,*sensor_options)
servo_chan.grid(row=1,column=4,padx=5)

# DC Motor Controls
Label(rightFrame, text="DC:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
dcFrame = Frame(rightFrame)
dcFrame.grid(row=2,column=1,columnspan=2, pady=10)
Label(dcFrame,text="Velocity Control").grid(row=0,column=0,columnspan=3)
dc_vel_val = DoubleVar()
dc_scale = Scale(dcFrame, command=dcUpdate, variable=dc_vel_val,orient=HORIZONTAL, from_=-100.0, to=100.0, tickinterval=50, length=200,width=10)
dc_scale.grid(row=1, column = 0, columnspan=3, pady=0,padx=5)
Label(dcFrame,text="Move by # degrees:").grid(row=2,column=0, pady=5)
dc_move_val = DoubleVar()
dc_move_entry = Entry(dcFrame,textvariable=dc_move_val,width=6)
dc_move_entry.grid(row=2,column=1)
dc_move_set = Button(dcFrame,text="Move",command=dcMove)
dc_move_set.grid(row=2,column=2,padx=5)
dc_chan_val = StringVar()
old_dc = ''
dc_chan_val.set(sensor_options[0])
dc_chan = OptionMenu(rightFrame,dc_chan_val,*sensor_options)
dc_chan.grid(row=2,column=4,padx=5)


# Stepper Motor Controls
Label(rightFrame, text="Stepper:",).grid(row=3,column=0,padx=2,pady=1, sticky=E)
stepperFrame = Frame(rightFrame)
stepperFrame.grid(row=3,column=1,columnspan=2,pady=10,sticky=W)
Label(stepperFrame,text="Move by # degrees:").grid(row=0,column=0, pady=5, sticky=W)
stepper_val = StringVar()
stepper_entry = Entry(stepperFrame,textvariable=servo_val,width=6).grid(row=0,column=1,sticky=W)
stepper_set = Button(stepperFrame,text="Move",command=stepperMove).grid(row=0,column=2,padx=5,sticky=W)
stepper_chan_val = StringVar()
old_stepper = ''
stepper_chan_val.set(sensor_options[0])
stepper_chan = OptionMenu(rightFrame,stepper_chan_val,*sensor_options)
stepper_chan.grid(row=3,column=4,padx=5)

updateSensors()
getPacket()

root.mainloop()