try:
    import Tkinter as tk # this is for python2
except:
    import tkinter as tk # this is for python3
import time
import colorsys
import serial
from DataPoint import NV11DataAccessories, NV11DataSpeedo, NV11DataBMS, NV11DataCommands
import RPi.GPIO as GPIO
import os
import traceback
import logging 




RESET_PIN = 16

ELEMENT_SIZE = 125
DEFAULT_LIGHT_COLOR = "#000000"
FONT_SIZE = 80

FONT_COLOR = "#ffffff"

SPEEDO_FONT_SIZE = 150

GREEN = "#42f47a"
RED = "#FF0000"

ERROR_FONT_SIZE = 15




## serial imports
import sys
import glob


#threading imports
from threading import Thread

#for logging
##import logging
##logging.basicConfig(format='%(asctime)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')

NO_OF_TELEMETRYCAGES = 10 

num_row = 8
num_col = 4
screen_width = 1024
screen_height = 800

ELEMENT_SIZE = screen_height/num_col
ELEMENT_WIDTH = screen_width/num_row

#[x,y ,row span (height), column span (width),description, height, width]
pouch_pannel_configuration = [0 for i in range(NO_OF_TELEMETRYCAGES)]
pouch_pannel_configuration[0] = [0,0,3,1,"left", ELEMENT_SIZE*3, ELEMENT_WIDTH] #left signal light
pouch_pannel_configuration[1] = [1,0,3,3,"speed", ELEMENT_SIZE*3, ELEMENT_WIDTH *3] #speed 
pouch_pannel_configuration[2] = [4,0,3,1,"right", ELEMENT_SIZE*3, ELEMENT_WIDTH] #right signal light
pouch_pannel_configuration[3] = [2,3,1,1,"brake", ELEMENT_SIZE, ELEMENT_WIDTH] #brake
pouch_pannel_configuration[4] = [3,3,1,1,"headlights", ELEMENT_SIZE, ELEMENT_WIDTH] #headlights 
pouch_pannel_configuration[5] = [5,0,1,3, "current",ELEMENT_SIZE, ELEMENT_WIDTH * 3] #current
pouch_pannel_configuration[6] = [5,1,1,3, "voltage",ELEMENT_SIZE, ELEMENT_WIDTH * 3] #voltage
pouch_pannel_configuration[7] = [5,2,1,3, "cycletime",ELEMENT_SIZE, ELEMENT_WIDTH * 3] #cycle time
pouch_pannel_configuration[8] = [5,3,1,3, "totaltime",ELEMENT_SIZE, ELEMENT_WIDTH * 3] #total time
pouch_pannel_configuration[9] = [0,3,1,2, "error",ELEMENT_SIZE, ELEMENT_WIDTH*2] #total time


def return_map(myArray):
    myDict = {}
    for i in range(len(myArray)):
        myDict[myArray[i][4]] = i
    return myDict
    
dict_map = return_map(pouch_pannel_configuration)





class DriveUserInterface(tk.Frame):
    def __init__(self, master = None):
        self.master = master
        pad = 3
        self._geom = '200x200+0+0'
        master.geometry("{0}x{1}+0+0".format(master.winfo_screenwidth()-pad, master.winfo_screenheight()-pad))
        master.bind('<Escape>', self.toggle_geom)
        
        tk.Frame.__init__(self, master, bg =DEFAULT_LIGHT_COLOR)
        screen_width = master.winfo_screenwidth()-pad
        screen_height = master.winfo_screenheight()-pad
        self.scrollbar = None
        self.grid()
        self.telemetrycages = []
        self.createCells(NO_OF_TELEMETRYCAGES)
        
    
    def toggle_geom(self, event):
        geom= self.master.winfo_geometry()
        print(geom, self._geom)
        self.master.geometry(self._geom)
        self._geom = geom
        
    def createCells(self, no_of_pannels):
        for i in range(NO_OF_TELEMETRYCAGES):
            configuration = pouch_pannel_configuration[i]
            x = configuration[0]
            y = configuration[1]
            span_x = configuration[2]
            span_y = configuration[3]
            description = configuration[4]
            height = configuration[5]
            width = configuration[6]
            new_pannel = UI_switcher(self, height, width, description) #switcher here 
##            new_pannel = TelemetryCage(self, height, width, description)
            self.telemetrycages.append(new_pannel)
            new_pannel.grid(column = x, row = y, columnspan = span_y, rowspan = span_x)
            print("Initialising: ", description)
        self.initText()


    def setText(self,x ,y):     
##         new_text = str(self.scrollbar.get())
         self.text_to_change_var.set(str(x +"\n " + y))
         self.scrollbar.set(y, 0)
            
##        listbox = tk.Listbox(self, yscrollcommand=scrollbar.set)
##        for i in range(1000):
##            listbox.insert(tk.END, str(i))
##        listbox.pack(side=tk.LEFT, fill=tk.BOTH)
##
##        scrollbar.config(command=listbox.yview)
        
    def initText(self):
        for element in self.telemetrycages:
            element.initText()

    def showError(self, error_message):
        self.UISelector("error").setText(error_message)


    def updateDriveUI(self, dataAccessory, dataSpeed, dataBMS, dataCommands):
        ##pass
        self.UISelector("speed").setText(abs(round(dataSpeed.speedKmh)))
        self.UISelector("left").setText(dataAccessory.lsig)
        self.UISelector("right").setText(dataAccessory.rsig)
        self.UISelector("brake").setText(dataAccessory.brake)
        self.UISelector("left").toggle_hazard(dataAccessory.hazard) #pass in True to toggle on or off
        self.UISelector("right").toggle_hazard(dataAccessory.hazard) #pass in True to toggle on or off
        self.UISelector("left").sync(self.UISelector("right")) #sync indicator light. Will only sync if hazard toggle is on
        #TODO: Integrate hazard
        #TODO: Integrate BMS
        self.UISelector("voltage").setText(round(dataBMS.volt,1))
        self.UISelector("current").setText(round(dataBMS.amp,1))
        #cycle times
        self.UISelector("cycletime").displayTime()
        if dataCommands.lapCounter:
            dataCommands.lapCounter = 0
            self.UISelector("cycletime").toggleTime()
        self.UISelector("totaltime").displayTime()
        
        #self.UISelector("temperature").setText(round(dataBMS.temperature,2))


                
        
    def UISelector(self, description):
        return self.telemetrycages[dict_map[description]]





class TelemetryElement(tk.Canvas):
    def __init__(self, parent, passHeight, passWidth, passDescription):
        #default color set as white
        self.color = "#000" #rgb format, hexidecimal
        self.width = passWidth
        self.height = passHeight
        tk.Canvas.__init__(self, parent, background = DEFAULT_LIGHT_COLOR, height = passHeight, width = passWidth)
        self.description = passDescription
        self.text = ""

    #to be overwritten if necessary
    def initText(self):
        self.text_id = self.create_text((self.width/2,self.height/2), text=self.description, font=("Courier", FONT_SIZE), fill = FONT_COLOR, anchor=tk.CENTER)
        xOffset = self.findXCenter(self, self.text_id)
##        self.move(self.text_id, xOffset, 0)


    def findXCenter(self, canvas, item):
      coords = canvas.bbox(item)
      print("Coords", coords)
      xOffset = (self.width / 2) - ((coords[2] - coords[0]) / 2)
      print("window width", self.width)
      print("xOffset", xOffset)
      return -1  * xOffset

        
    def setColor(self, new_color):
        self.configure(background = new_color)

    def setText(self, text_to_change ):
        text_to_change = str(text_to_change)
        self.itemconfigure(self.text_id, text=text_to_change)
        self.text = text_to_change
        self.indicate()

    def indicate(self):
        if self.text == "1":
            self.setColor(RED)
        else:
            self.setColor(DEFAULT_LIGHT_COLOR)
        


def UI_switcher(parent, height, width, description):
    if description == "speed":
        return Speedometer(parent, height, width, description)
    elif description == "left" or description == "right":
        return Indicator(parent, height, width, description)
    elif description == "error":
        return ErrorBox(parent, height, width, description)
    elif description == "cycletime" or description == "totaltime":
        return TimeBoxes(parent, height, width, description)
    else:
        return TelemetryElement(parent, height, width, description)
    


class Speedometer(TelemetryElement):
    def __init__(self, parent, passWidth, passHeight, description):
        TelemetryElement.__init__(self,parent, passWidth, passHeight, description)

    def initText(self): #override
        self.text_id = self.create_text((self.width/2,self.height/2), text=self.description, font=("Courier", SPEEDO_FONT_SIZE), anchor=tk.CENTER)
        xOffset = self.findXCenter(self, self.text_id)
##        self.move(self.text_id, xOffset, 0)
    def indicate(self):
        rgb = hsv_hex_conversion(float(self.text),1,1)
        self.setColor(rgb)


class TimeBoxes(TelemetryElement):
    def initText(self): #overide to start the time 
        super().initText()
        self.start_time = time.time() #in seconds

    def toggleTime(self):
        #resets start time, new laptime or 
        self.start_time = time.time()


    def displayTime(self):
        current_time = time.time()
        elapse_time = int(current_time - self.start_time)
        seconds = elapse_time % 60
        minutes = elapse_time // 60
        self.setText("{0}:{1}".format(minutes, seconds))

        
        
    
        
        
        


class Indicator(TelemetryElement):
    def initText(self): #override
        self.text_id = self.create_text((self.width/2,self.height), text=self.description, font=("Courier", FONT_SIZE), anchor=tk.CENTER)
##        self.move(self.text_id, 0, -10)
        xOffset = self.findXCenter(self, self.text_id)
        self.hazard_toggle = False
        self.prev_time = time.time()
        self.blinkspeed = 0.5 #in seconds

    def setText(self, text_to_change ): #override
        if self.hazard_toggle: #if hazard is on
            #ignore the setText
            self.blink() #call blinking function
        else:
            text_to_change = str(text_to_change)
            self.itemconfigure(self.text_id, text=text_to_change)
            self.text = text_to_change
            self.indicate()


    def toggle_hazard(self, toToggle):
        if toToggle:
            if self.hazard_toggle == True:
                self.hazard_toogle = False
            else:
                self.hazard_toogle = True
    
    def blink(self):
        curr_time = time.time()
        time_elapse = curr_time - self.prev_time
        if time_elapse > self.blinkspeed:
            #time to toggle the light
            self.prev_time = curr_time #reset timer.
            if self.text == "1":
                self.text = "0"
                self.indicate()
            else:
                self.text = "1"
                self.indicate()

    def sync(self, otherIndicator):
        if self.hazard_toggle: #only in hazard mode then sync
            self.text = otherIndicator.text        
                    

    def indicate(self):
        self.delete(tk.ALL)
        padding = 10
        center_y = self.height / 2
        center_x = self.width / 2
        if self.description == "left":
            triangle_top = [self.width - padding, 0 + padding]
            triangle_mid = [0 + padding, center_y]
            triangle_btm = [self.width - padding, self.height - padding]
        else:
            triangle_top = [0 + padding, 0 + padding]
            triangle_mid = [self.width - padding, center_y]
            triangle_btm = [0 + padding, self.height - padding]
        points = triangle_top + triangle_mid + triangle_btm
        if self.text == "1":
            self.create_polygon(points, outline='#f11', 
            fill='#1f1', width=2)
        else:
            self.create_polygon(points, outline='#f11', 
            fill=DEFAULT_LIGHT_COLOR, width=2)
            

class ErrorBox(TelemetryElement):
    def initText(self):
        self.text = "No error"
        self.text_id = self.create_text((self.width/2,self.height/2), text=self.text, font=("Courier", ERROR_FONT_SIZE ), fill = FONT_COLOR, anchor=tk.CENTER)
        xOffset = self.findXCenter(self, self.text_id)
        



def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port, timeout=2)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException): 
            pass
    return result


def funct1():
    ser = serial.Serial(serial_ports()[0], timeout=2)
    print(temperature_array)
    while True:
        data = ser.read_until()
        data = data.decode('UTF-8') #conversion to string
        module, temperature, index = parse_temp(data)
##        print("index", index)
##        print("module", module)
        temperature_array[module][index] = temperature
        
## To convert HSV into simulation friendly hex codes
# h values(min:0 max:100), s values(min:0 max:1), v values(min:0 max:1)

def my_map(s, min_s = 0, max_s = 100, min_h = 0, max_h =45):
    h = max_h - s * (max_h - min_h) / (max_s - min_s)
    return h

def hsv_hex_conversion(h, s, v):
    h = my_map(h)
    rgb = colorsys.hsv_to_rgb(h/100, s, v)
    r = hex(int(rgb[0]* 255))[2:]
    g = hex(int(rgb[1]* 255))[2:]
    b = hex(int(rgb[2]* 255))[2:]
    if len(r) == 1:
        r = r + r
    if len(g) == 1:
        g = g + g
    if len(b) == 1:
        b = b + b    
    return ('#' + r + g + b)            



mode = "run"

def reset_arduino(s=''):
    #pulse the reset pin
    app.showError("Reseting \n arduino")
    app.update()
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(3)
    app.showError("No error")
    app.update()
    print(s,"resetting Arduino")


def log_error(exctype, value, tb):
    with open("error_log.txt", "a+") as f:
        f.write(datetime.datetime.now() + "\n")
        f.write("Type:" + str(exctype) + + "\n")
        f.write("Value:" +  str(value) +  "\n")
        f.write("Traceback:" +  str(tb) +  "\n"), 


def shut_down():
    os.system("sudo shutdown -h now")
    


def try_connect():
    while True:
        try:
    ##            s = serial.Serial("/dev/serial0", baudrate=9600, timeout=2)
            if mode == "dev":
                serial_port_str = serial_ports()
                if len(serial_port_str) != 0:
                    s = serial.Serial(serial_port_str[0], baudrate=9600, timeout=2)
                else:
                    raise serial.serialutil.SerialException
            else:
                print("connecting to serial0")
                s = serial.Serial("/dev/serial0", baudrate=9600, timeout=1) 
                print("after connecting serial0") 
            dataSpeed = NV11DataSpeedo(0x0A)
            dataAcc = NV11DataAccessories(0x10)
            dataBMS = NV11DataBMS(0x11)
            dataCommands = NV11DataCommands(0x12)
            app.showError("No error")
            return s, dataSpeed, dataAcc, dataBMS, dataCommands
        except serial.serialutil.SerialException:
            reset_arduino("serial port not found")
            app.showError("Error opening \n serial port")
            time.sleep(1)
            app.update()
            print("Trying to restart serial")
    




print("Display")
GPIO.setmode(GPIO.BOARD)
GPIO.setup(RESET_PIN, GPIO.OUT)
while True:
    try:
        root = tk.Tk()
        #for root
##            if mode != "dev":
        root.wm_attributes('-type', 'splash')
        app = DriveUserInterface(root)
        app.background = DEFAULT_LIGHT_COLOR
        ##app.master.title('DRIVE GUI')
        app.update()
        break
    except tk.TclError as e:
        print("error starting window")
        time.sleep(1)
        traceback.print_exc()
        

prev_time = time.time()
logging.basicConfig(filename='log.txt', filemode='w', format='%(levelname)s-%(asctime)s-%(message)s', level = logging.DEBUG)
logging.debug("Starting logs")
reset_arduino()
app.showError("Reboot Arduino")
app.showError("Booting serial port...")
app.update()
s, dataSpeed, dataAcc, dataBMS, dataCommands = try_connect()
canTimeout = False
canTimeoutCount = 0
while True:
    #curr_time = time.time()
    #print("Cycle time: " + str(curr_time - prev_time))
    #prev_time = curr_time
        
    try:
        
        line = s.readline().decode()
        s.flushInput()
        
        if line != '':                    
            if dataSpeed.checkMatchString(line):
                dataSpeed.unpackString(line)                
            elif dataAcc.checkMatchString(line):
                dataAcc.unpackString(line)
            elif dataBMS.checkMatchString(line):
                dataBMS.unpackString(line)
            elif dataCommands.checkMatchString(line):
                dataCommands.unpackString(line)
            
            if dataCommands.shutdownPi:
                dataCommands.shutdownPi = 0
                shut_down()

            app.updateDriveUI(dataAcc, dataSpeed, dataBMS, dataCommands)
            canTimeoutCount = 0
            if canTimeout:
                canTimeout= False
                app.showError("No error")
                
        else:
            canTimeoutCount += 1
            if canTimeoutCount > 5:
                print("Can error")
                app.showError("Can't read \n CAN bus.")
                canTimeout = True
                reset_arduino("no CAN msg,")
                canTimeoutCount = 0 
        app.update()
    except serial.serialutil.SerialException as e:
        logging.error("Serial exception occured", exc_info = True)
        print(e)
        print("Serial Error")
        app.showError("Can't read \n serial port")
        app.update()
        s, dataSpeed, dataAcc, dataCommands = try_connect()
    except UnicodeDecodeError as e:
        logging.error("Decode exception occured", exc_info = True)
        print("Decode error")
        app.showError("Decode Error")
        app.update()
    except:
        logging.error("Unexpected exception occured", exc_info = True)
        traceback.print_exc()
        print("unexpected error occured")
        app.showError("Unexpected error")
        app.update()
        

##if __name__ == '__main__':
####    Thread(target = funct1).start()
##    Thread(target = funct2).start()




