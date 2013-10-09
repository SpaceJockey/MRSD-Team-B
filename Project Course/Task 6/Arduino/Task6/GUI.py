from tkinter import *
from serial import *
import time
import struct
import random
from copy import copy

# Open Serial Port
port = "COM5"
baudRate= 9600
ser = Serial(port, baudRate, timeout=0,writeTimeout=0)
time.sleep(1)

# Setup GUI Window
root = Tk()   
root.wm_title("MRSD Sensors and Motors Lab")
root.config(background="#DDDDDD")

forceVal, rangeVal,potVal,triVal = (1.0,)*4
sensor_options = ('Local','On','Off','Force','Range','Potentiometer','Triangle')
data = [0.0,]*100

forceVar = StringVar()
rangeVar = StringVar()
potVar = StringVar()

packet = b''


#########################################
############ HELPER FUNCTIONS ###########
######################################### 

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

def sendPacket(thetas):
    thetas[0]=0xDEAD
    thetas[12] = 0xBEEF
    print(thetas);
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
    global forceVal, rangeVal, potVal, triVal
    sensors = thetas[2:]
    forceVal = sensors[getChan('Force')]
    rangeVal = sensors[getChan('Range')]
    potVal = sensors[getChan('Potentiometer')]
    triVal = sensors[getChan('Triangle')]
    
def getVal(name):
    global forceVal, rangeVal, potVal,triVal
    if name=="Local":
        return 0.0
    elif name=="On":
        return 100.0
    elif name=="Off":
        return 0.0
    elif name=="Force":
        return forceVal
    elif name=="Range":
        return rangeVal
    elif name=="Potentiometer":
        return potVal   
    elif name=="Triangle":
        return triVal
    
def getChan(name):
    if name=="Local":
        return 0
    elif name=="On":
        return 7
    elif name=="Off":
        return 0
    elif name=="Force":
        return 2
    elif name=="Range":
        return 3
    elif name=="Potentiometer":
        return 1  
    elif name=="Triangle":
        return 8
        
def getChanOut(name):
    if name=="Servo":
        return 4
    elif name=="DCPos":
        return 2
    elif name=="DCVel":
        return 3
    elif name=="Stepper":
        return 5     
    
def updateSensors():
    global wChan, old_plot, servo_chan_val, old_servo, dc_chan_val, old_dc, stepper_chan_val, old_stepper, control_type_val, old_type, data, plotWin
    
    # UPDATE SENSOR READOUTS
    forceVar.set("%4.3f" % getVal('Force'))
    rangeVar.set("%4.3f" % getVal('Range'))
    potVar.set("%4.3f" % getVal('Potentiometer'))

    # UPDATE DATA BUFFER AND PLOT
    if old_plot != wChan.get():
        old_plot = wChan.get()
        data = [getVal(old_plot),]*100
    data.pop(0)
    data.append(getVal(old_plot))        
    updatePlot(data,plotWin)
    
    
    # UPDATE COMMAND VARIABLES
    thetaL = [0,]*13
    
    # SERVO MOTOR
    if old_servo != servo_chan_val.get():
        old_servo = servo_chan_val.get()
        thetaL[2]=getChanOut('Servo')
        thetaL[1]=getChan(old_servo)
        sendPacket(thetaL)        
    
    # DC MOTOR
    if old_dc != dc_chan_val.get() or old_type != control_type_val.get():
        old_dc = dc_chan_val.get()
        old_type = control_type_val.get()
        
        if old_type == 'Position Control':
            thetaL[2]=getChanOut('DCPos')
        elif old_type == 'Velocity Control':
            thetaL[2]=getChanOut('DCVel')
        thetaL[1]=getChan(old_dc)
        sendPacket(thetaL)
    
    # STEPPER MOTOR
    if old_stepper != stepper_chan_val.get():
        old_stepper = stepper_chan_val.get()
        thetaL[2]=getChanOut('Stepper')
        thetaL[1]=getChan(old_stepper) 
        sendPacket(thetaL)        

    root.after(100,updateSensors)
    
def updatePlot(data,plotWin):
    d = [105-100*n/(max(data)+0.0000001) for n in data]
    plotWin.delete("all")
    for i in range(1,100):
        plotWin.create_line((i-1)*2,d[i-1],i*2,d[i], fill="red")
    

def servoSet():
    global old_servo
    if old_servo == "Local":
        theta = [0,]*13
        theta[2]=getChanOut('Servo')
        theta[1]=9
        theta[3]=int(int("0xFFFF",0)*servo_val.get()/90)
        sendPacket(theta)
    

# dcUpdate: Fires whenever the DC velocity slider is moved
def dcUpdate(val):
    global old_dc,old_type
    if old_dc == "Local":
        theta = [0,]*13
        theta[1]=10 
        if old_type == 'Position Control':
            theta[2]=getChanOut('DCPos')
            theta[4] = int(int("0xFFFF",0)*(int(val)+100)/200);
        elif old_type == 'Velocity Control':
            theta[2]=getChanOut('DCVel')
            theta[4] = int(int("0xFFFF",0)*(int(val)+100)/200);
           
        
        sendPacket(theta)
        
        
# stepperMove: Fires whenever the "Move" button  is pressed in the stepper frame
def stepperMove():
    global old_stepper
    if old_stepper == "Local":
        theta = [0,]*13
        theta[2]=getChanOut('Stepper')
        theta[1] = 11
        theta[5] = int(int("0xFFFF",0)*stepper_val.get()/90)
        sendPacket(theta)
 
#########################################
########## SET UP GUI ELEMENTS ##########
######################################### 

###################################
### LEFT FRAME - SENSOR OUTPUTS ###
###################################
leftFrame = Frame(root, width=300,height=400)
leftFrame.grid(row=0, column=0, padx=10, pady=2)

# Sensor Readouts
Label(leftFrame, text="Sensor Outputs").grid(row=0, column=0, columnspan=2,padx=2, pady=2)
Label(leftFrame, text="Force:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, textvariable=forceVar).grid(row=1,column=1,pady=1,stick=W)
Label(leftFrame, text="Range:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, textvariable=rangeVar).grid(row=2,column=1,pady=1,stick=W)
Label(leftFrame, text="Potentiometer:").grid(row=3,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, textvariable=potVar).grid(row=3,column=1,pady=1,stick=W)

# Plot Window
plotWin = Canvas(leftFrame, width=200, height=110,bg="#FFFFFF")
plotWin.grid(row=6,column=0,columnspan=2,padx=5,pady=5)
wChan = StringVar()
old_plot = ''
wChan.set(sensor_options[0])
Label(leftFrame,text="Plot Channel:").grid(row=7,column=0,padx=5,pady=5,stick=E)
w_chan = OptionMenu(leftFrame,wChan,*sensor_options)
w_chan.grid(row=7,column=1,padx=5)

####################################
### RIGHT FRAME - MOTOR CONTROLS ###
####################################
rightFrame = Frame(root, width=800, height = 500)
rightFrame.grid(row=0, column=1, padx=10, pady=2)
Label(rightFrame, text="Motor Controls").grid(row=0, column=0, columnspan=3, padx=10, pady=2)
Label(rightFrame, text="Channel Assignment").grid(row=0, column=4, columnspan=3, padx=10, pady=2)

# Servo Motor Controls
Label(rightFrame, text="Servo:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
servoFrame = Frame(rightFrame,bg="#DEDEDE")
servoFrame.grid(row=1,column=1,columnspan=2,pady=10,sticky=W)
Label(servoFrame,text="Move to position:").grid(row=0,column=0, pady=5, sticky=W,padx=2)
servo_val = IntVar()
servo_entry = Entry(servoFrame,textvariable=servo_val,width=6).grid(row=0,column=1)
servo_set = Button(servoFrame,text="Move",command=servoSet).grid(row=0,column=2,padx=5)
servo_chan_val = StringVar()
old_servo = ''
servo_chan_val.set(sensor_options[0])
servo_chan = OptionMenu(rightFrame,servo_chan_val,*sensor_options)
servo_chan.grid(row=1,column=4,padx=5)

# DC Motor Controls
Label(rightFrame, text="DC:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
dcFrame = Frame(rightFrame,bg="#DEDEDE")
dcFrame.grid(row=2,column=1,columnspan=2, pady=10)
control_type_val = StringVar()
old_type = ''
control_options = ('Position Control','Velocity Control')
control_type = OptionMenu(dcFrame,control_type_val,*control_options).grid(row=0,column=0,columnspan=3,pady=2)
control_type_val.set(control_options[0])
dc_vel_val = IntVar()
dc_vel_val.set(-100)
dc_scale = Scale(dcFrame, command=dcUpdate, variable=dc_vel_val,orient=HORIZONTAL, from_=-100.0, to=100.0, tickinterval=50, length=200,width=10)
dc_scale.grid(row=1, column = 0, columnspan=3, padx=5,pady=2)
dc_chan_val = StringVar()
old_dc = ''
dc_chan_val.set(sensor_options[0])
dc_chan = OptionMenu(rightFrame,dc_chan_val,*sensor_options)
dc_chan.grid(row=2,column=4,padx=5)

# Stepper Motor Controls
Label(rightFrame, text="Stepper:",).grid(row=3,column=0,padx=2,pady=1, sticky=E)
stepperFrame = Frame(rightFrame,bg="#DEDEDE")
stepperFrame.grid(row=3,column=1,columnspan=2,pady=10,sticky=W)
Label(stepperFrame,text="Move by # degrees:").grid(row=0,column=0, pady=5, sticky=W,padx=2)
stepper_val = IntVar()
stepper_entry = Entry(stepperFrame,textvariable=stepper_val,width=6).grid(row=0,column=1,sticky=W)
stepper_set = Button(stepperFrame,text="Move",command=stepperMove).grid(row=0,column=2,padx=5,sticky=W)
stepper_chan_val = StringVar()
old_stepper = ''
stepper_chan_val.set(sensor_options[0])
stepper_chan = OptionMenu(rightFrame,stepper_chan_val,*sensor_options)
stepper_chan.grid(row=3,column=4,padx=5)


updateSensors()
getPacket()

root.mainloop()