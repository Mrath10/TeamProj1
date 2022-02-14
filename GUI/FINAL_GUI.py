from tkinter import *
import threading
import serial.tools.list_ports
import serial
import tkinter as tk

root = tk.Tk()

hold = 0
currentmode = "None Chosen"
minimum = 0.000  # minimum reading
maximum = 0.000  # maximum reading
current = 0  # current reading
offset = 1  # dc offset data
continuity = 2.0  # continuity value
bright = 0  # brightness level
port = serial.Serial()
baudrate = 9600
foundport = False
ports = [comport.device for comport in serial.tools.list_ports.comports()]

# Readline function to handle readings from UART
def read_line():
    while 1:
        global current
        global maximum
        global minimum
        global bright
        global hold
        # standard iterations to obtain values
        # Get the current reading
        current = port.readline().decode('utf-8').split(':')

        # checl of the device is in hold
        if current[0] == 'H':
            hold = 1
            holdvar.set("HOLD ON")
            # print that it is in hold

        # if hold != 1:
        # Check what the current reading is, change the mode and set the labels
        elif 'A' in current[0]:
            hold = 0
            holdvar.set("HOLD OFF")
            ac_read()
            currentmeas.set(current[1] + "VRMS")
            currentmin.set(current[2] + "VRMS")
            currentmax.set(current[3] + "VRMS")
            bright = current[4]
            brightcheck()
            # set the brightness variable in here and call dthe brightness function every time
        elif "D" in current[0]:
            hold = 0
            holdvar.set("HOLD OFF")
            dc_read()
            currentmeas.set(current[1] + "V")
            currentmin.set(current[2] + "V")
            currentmax.set(current[3] + "V")
            bright = current[4]
            brightcheck()

        elif 'R' in current[0]:
            hold = 0
            holdvar.set("HOLD OFF")
            resistance_read()
            currentmeas.set(current[1] + "Ω")
            bright = int(current[2])
            brightcheck()
        elif 'C' in current[0]:
            hold = 0
            holdvar.set("HOLD OFF")
            cont_read()
            currentmeas.set(current[1] + "Ω")
            currentmin.set("---")
            currentmax.set("---")
            bright = int(current[2])
            brightcheck()


readings = threading.Thread(target=read_line)


# handler for if DC is read from UART
def dc_read():
    global currentmode
    currentmode = "DC"
    modelabel.config(text="Current Mode: DC")


# handler for if AC is read from UART
def ac_read():
    global currentmode
    currentmode = "AC"
    modelabel.config(text="Current Mode: AC")
    data.config(text="AC Voltage (Vrms):")


# handler for if RESISTANCE is read from UART
def resistance_read():
    global currentmode
    currentmode = "Resistance"
    modelabel.config(text="Current Mode: Resistance")
    data.config(text="Resistance:")
    currentmin.set("---")
    currentmax.set("---")


# handler for if CONTITNUITY is read from UART
def cont_read():
    global currentmode
    currentmode = "Contintuity"
    modelabel.config(text="Current Mode: Continuity")
    data.config(text="Current reading: ")


# handler for brightness level coming from GUI
def brightcheck():
    brightcheckvar = "Brightness level :" + str(bright)
    lightvar.set(brightcheckvar)


# handler for if DC button is pressed
def dc_voltage():
    global currentmode
    currentmode = "DC"
    modelabel.config(text="Current Mode: DC")
    port.write(b'D')


# handler for if AC button is pressed
def ac():
    global currentmode
    currentmode = "AC"
    modelabel.config(text="Current Mode: AC")
    data.config(text="AC Voltage (Vrms):")
    port.write(b'A')


# handler for if RESISTANCE button is pressed
def resistance():
    global currentmode
    currentmode = "Resistance"
    modelabel.config(text="Current Mode: Resistance")
    data.config(text="Resistance: ")
    currentmin.set("---")
    currentmax.set("---")
    port.write(b'R')


# handler for if CONTINUITY button is pressed
def cont():
    global currentmode
    currentmode = "Contintuity"
    modelabel.config(text="Current Mode: Continuity")
    data.config(text="Current reading: ")
    port.write(b'C')


# handler for if HOLD button is pressed
def togglehold():
    global hold
    hold ^= 1
    if hold == 0:
        holdvar.set("HOLD OFF")
        port.write(b'N')
    else:
        holdvar.set("HOLD ON")
        port.write(b'H')


# handler for if RESET button is pressed
def reset():
    port.write(b'x')


# handler for sending the custom message
def sendmes():
    to_send = "m" + mesVar.get() + "\n"

    if len(to_send) < 16:

        port.write(to_send.encode('utf-8'))
    else:
        print("String exceeds 16 characters")


# handler for the CONNECT button
def connectport():
    # initialise port params
    global port
    global baudrate

    portname = var_port.get()

    try:
        port = serial.Serial(portname, baudrate)
        conport.set("Connected to: " + str(portname))
        readings.start()
    except serial.serialutil.SerialException:
        port = None
        conport.set("Error Connecting to Port, try reconnecting")
        print("Error Connecting to Port, try reconnecting")


# handler if brightness level 1 button is pressed
def l1():
    lightvar.set("Brightness level: 1")
    port.write(b'1')


# handler if brightness level 2 button is pressed
def l2():
    lightvar.set("Brightness level: 2")
    port.write(b'2')


# handler if brightness level 3 button is pressed
def l3():
    lightvar.set("Brightness level: 3")
    port.write(b'3')


# handler if brightness level 4 button is pressed
def l4():
    lightvar.set("Brightness level: 4")
    port.write(b'4')


# handler if brightness level 5 button is pressed
def l5():
    lightvar.set("Brightness level: 5")
    port.write(b'5')


# handler if continuity increase button is pressed
def upcont():
    print("+")
    port.write(b'+')


# handler if brightness continuity decrease button is pressed
def downcont():
    print("-")
    port.write(b'-')


# ALL REVELVANT BUTTONS
DCButton = Button(root, text="DC", command=lambda: dc_voltage())
ACButton = Button(root, text="AC", command=lambda: ac())
ResButton = Button(root, text="Resistance", command=lambda: resistance())
ContButton = Button(root, text="Continuity", command=lambda: cont())
HoldButton = Button(root, text="Hold", command=lambda: togglehold())
ResetButton = Button(root, text="Reset", command=lambda: reset())
PortButton = tk.Button(root, text="Connect", command=lambda: connectport())
B1 = tk.Button(root, text="1", command=lambda: l1())
B2 = tk.Button(root, text="2", command=lambda: l2())
B3 = tk.Button(root, text="3", command=lambda: l3())
B4 = tk.Button(root, text="4", command=lambda: l4())
B5 = tk.Button(root, text="5", command=lambda: l5())
contup = tk.Button(root, text="+", command=lambda: upcont())
contdown = tk.Button(root, text="-", command=lambda: downcont())
sendButton = tk.Button(root, text="SEND", command=lambda: sendmes())

# ENTRIES

# port entry
var_port = tk.StringVar()
portentry = tk.Entry(root, textvariable=var_port)

# custom message entry
mesVar = tk.StringVar()
mesVar.set('')
customMes = tk.Entry(root, textvariable=mesVar)

# ALL LABELS

# Mode
modelabel = Label(root, text="Current Mode:")

# Ports
conport = tk.StringVar()
portconlabel = tk.Label(root, textvariable=conport)
portnamelabel = tk.Label(root, text="Select Port: ")

# Brightness
lightvar = tk.StringVar()
lightvar.set("Brightness level:")
lightlabel = Label(root, textvariable=lightvar)

# Minimum label
minlabel = Label(root, text="Minimum:")
currentmin = tk.StringVar()
currentmin.set('0')
minreading = tk.Label(root, textvariable=currentmin)

# Maximum label
maxlabel = Label(root, text="Maximum:")
currentmax = tk.StringVar()
currentmax.set('0')
maxreading = tk.Label(root, textvariable=currentmax)

# Current reading label
data = Label(root, text="Current Reading: ")
currentmeas = tk.StringVar()
currentmeas.set('0')
reading = tk.Label(root, textvariable=currentmeas)

# Continuity labels
contthresh = tk.StringVar()
contthresh.set("Contintuity Threshold")
contthreshlabel = tk.Label(root, textvariable=contthresh)

# Hold labels
holdvar = tk.StringVar()
holdvar.set(' ')
holdlabel = tk.Label(root, textvariable=holdvar)

# BUILDING THE GUI
root.title("ENGG2800 GROUP 11 DIGITAL MULTIMETER")

# Grid all the elements
portnamelabel.grid(row=0, column=0)
portentry.grid(row=0, column=1)
PortButton.grid(row=0, column=2)
portconlabel.grid(row=0, column=3)
DCButton.grid(row=1, column=1)
ACButton.grid(row=1, column=2)
ResButton.grid(row=1, column=3)
ContButton.grid(row=1, column=4)
modelabel.grid(row=1, column=0)
data.grid(row=3, column=0)
reading.grid(row=3, column=1)
minlabel.grid(row=4, column=0)
minreading.grid(row=4, column=1)
maxlabel.grid(row=5, column=0)
maxreading.grid(row=5, column=1)
contthreshlabel.grid(row=6, column=0)
contdown.grid(row=6, column=1)
contup.grid(row=6, column=2)
lightlabel.grid(row=7, column=0)
B1.grid(row=7, column=1)
B2.grid(row=7, column=2)
B3.grid(row=7, column=3)
B4.grid(row=7, column=4)
B5.grid(row=7, column=5)
ResetButton.grid(row=8, column=0)
HoldButton.grid(row=8, column=1)
holdlabel.grid(row=8, column=2)
customMes.grid(row=9, column=2)
sendButton.grid(row=9, column=3)

if __name__ == "__main__":
    root.update()
    root.mainloop()
