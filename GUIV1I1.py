from tkinter import *
import serial
from serial import Serial


#Declaration for root
root = Tk()


#Relevant drop down menus

#for mode
variable1 = StringVar(root)
variable1.set("Mode")
mode = OptionMenu(root, variable1, "DC Voltage", "AC Voltage", "Resistance", "Continuity")
mode.grid(row=0, column=1)
modelabel = Label(root, text="Current mode")
modelabel.grid(row=0,column=0)

#for hold
variable2 = StringVar(root)
variable2.set("status")
holdlabel = Label(root, text= "Hold")
hold = OptionMenu(root, variable2, "on","off" )
hold.grid(row=1, column=1)
holdlabel.grid(row=1, column=0)

#backlight level
variable3 = StringVar(root)
variable3.set("Level")
mode = OptionMenu(root, variable3, "1", "2", "3", "4", "5")
mode.grid(row=2, column=1)
lightlabel = Label(root, text="Backlight Level")
lightlabel.grid(row=2, column=0)

#readings, going to add relevant comands after pyserial implementation
readinglabel = Label(root, text="Current reading")
readinglabel.grid(row=3,column=0)
readingentry = Entry(root, width = 10)
readingentry.grid(row=3, column=1)

Minlabel = Label(root,text="Current Minimum")
Minlabel.grid(row=5,column=0)
minentry = Entry(root, width=10)
minentry.grid(row=5,column=1)


Maxlabel = Label(root, text="Current Maximum")
Maxlabel.grid(row=6,column=0)
maxentry = Entry(root,width=10)
maxentry.grid(row=6,column=1)

#reset
resetButton = Button(root, text="RESET")
resetButton.grid(row=7,column=0)

#current message
messagelabel = Label(root, text="Current Message")
messagelabel.grid(row=8,column=0)
messageentry = Entry(root, width=10) 
messageentry.grid(row=8,column=1)
#run all commands
mainloop()