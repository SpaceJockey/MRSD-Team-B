from Tkinter import *
from serial import *
import time

port = "COM3"
baudRate= 9600
#ser = Serial(port, baudRate, timeout=0,writeTimeout=0)
#print "Opening Serial Port..."
#time.sleep(1)

thermalVal, forceVal, lightVal, rangeVal = (1.0,)*4

root = Tk()

def ping():
    print "Ping!"
    ser.write("ping\n")

def setServo():
    print servo_val.get()
	
def updateSensors():
    global thermalVal, forceVal, lightVal, rangeVal
    print "Updating..."
    
    thermalVal = thermalVal + 1 * 0.9
    forceVal = forceVal *1.01
    lightVal = lightVal *1.1
    rangeVal = rangeVal + 0.1
    
    thermalVar.set("%4.3f" % thermalVal)
    forceVar.set("%4.3f" % forceVal)
    lightVar.set("%4.3f" % lightVal)
    rangeVar.set("%4.3f" % rangeVal)

    root.after(1000,updateSensors)
	
def moveStepper():
    print stepper_val.get()

def dcUpdate(val):
    print val
    
def dcMove():
    print ""
	
root.wm_title("MRSD Sensors and Motors Lab")
root.config(background="#DDDDDD")

# Left frame - Sensor Outputs

leftFrame = Frame(root, width=300,height=400)
leftFrame.grid(row=0, column=0, padx=10, pady=2)

Label(leftFrame, text="Sensor Outputs").grid(row=0, column=0, columnspan=2,padx=2, pady=2)
Label(leftFrame, text="Thermal:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Force:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Light:").grid(row=3,column=0,padx=2,pady=1, sticky=E)
Label(leftFrame, text="Range:").grid(row=4,column=0,padx=2,pady=1, sticky=E)

thermalVar = StringVar()
forceVar = StringVar()
lightVar = StringVar()
rangeVar = StringVar()
updateSensors()

Label(leftFrame, textvariable=thermalVar).grid(row=1,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=forceVar).grid(row=2,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=lightVar).grid(row=3,column=1,pady=1,stick=W)
Label(leftFrame, textvariable=rangeVar).grid(row=4,column=1,pady=1,stick=W)


# Right Frame - Motor Controls	
rightFrame = Frame(root, width=800, height = 500)
rightFrame.grid(row=0, column=1, padx=10, pady=2)
Label(rightFrame, text="Motor Controls").grid(row=0, column=0, columnspan=3, padx=10, pady=2)


# Servo Motor Controls
Label(rightFrame, text="Servo:").grid(row=1,column=0,padx=2,pady=1, sticky=E)
servoFrame = Frame(rightFrame)
servoFrame.grid(row=1,column=1,columnspan=2,pady=10)
Label(servoFrame,text="Move to position:").grid(row=0,column=0, pady=5, sticky=W)
servo_val = StringVar()
servo_entry = Entry(servoFrame,textvariable=servo_val,width=6).grid(row=0,column=1)
servo_set = Button(servoFrame,text="Move",command=setServo).grid(row=0,column=2,padx=5)


# DC Motor Controls
Label(rightFrame, text="DC:").grid(row=2,column=0,padx=2,pady=1, sticky=E)
dcFrame = Frame(rightFrame, width=200, height=200,borderwidth=1)
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



# Stepper Motor Controls
Label(rightFrame, text="Stepper:",).grid(row=3,column=0,padx=2,pady=1, sticky=E)
stepperFrame = Frame(rightFrame)
stepperFrame.grid(row=3,column=1,columnspan=2,pady=10)
Label(stepperFrame,text="Move by # degrees:").grid(row=0,column=0, pady=5, sticky=W)
stepper_val = StringVar()
stepper_entry = Entry(stepperFrame,textvariable=servo_val,width=6).grid(row=0,column=1,sticky=W)
stepper_set = Button(stepperFrame,text="Move",command=moveStepper).grid(row=0,column=2,padx=5,sticky=W)


root.mainloop()