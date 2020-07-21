import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
#import RPi.GPIO as GPIO

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn import model_selection
from sklearn import neighbors
import smbus            #import SMBus module of I2C
from time import sleep          #import
import os,time

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
d = 0


# Load default font.
font = ImageFont.load_default()
'''font2 = ImageFont.truetype('digital-7.mono.ttf', 24)'''

# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
# font = ImageFont.truetype('Minecraftia.ttf', 8)

dateString = '%A %d %B %Y'
timeString = '%H:%M:%S'



#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19

CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
BCCEL_XOUT_H = 0x3B
BCCEL_YOUT_H = 0x3D
BCCEL_ZOUT_H = 0x3F
DCCEL_XOUT_H = 0x3B
DCCEL_YOUT_H = 0x3D
DCCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

import serial
import RPi.GPIO as GPIO      
import os, time

GPIO.setmode(GPIO.BCM)
GPIO.setup(14,GPIO.IN)
GPIO.setup(15,GPIO.OUT)
    
def send_message(message):
    # Enable Serial Communication
    port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)
    # Transmitting AT Commands to the Modem
    # '\r\n' indicates the Enter key
    temp = 'AT+CMGF=1'+'\r\n' 
    port.write(bytes(temp,'utf-8'))  # Select Message format as Text mode 
    rcv = port.read(10)
    #print(rcv)
    time.sleep(1)
    temp = 'AT+CNMI=2,1,0,0,0'+'\r\n' 
    port.write(bytes(temp,'utf-8'))   # New SMS Message Indications
    rcv = port.read(10)
    #print(rcv)
    time.sleep(1)
        # Sending a message to a particular Number
    temp = 'AT+CMGS="8247302604"'+'\r\n' 
    port.write(bytes(temp,'utf-8'))
    rcv = port.read(10)
    print(rcv)
    time.sleep(1)
    temp = message+'\r\n' 
    port.write(bytes(temp,'utf-8'))  # Message
    rcv = port.read(10)
    print(rcv)
    temp = "\x1A" 
    port.write(bytes(temp,'utf-8')) # Enable to send SMS


    
import speech_recognition as sr
import pyaudio
   
def get_speech_command():
    r = sr.Recognizer()
    with sr.Microphone(device_index=3) as source:
        try:
            print('say anything:')
            r.adjust_for_ambient_noise(source,duration=0.0001)
            audio = r.listen(source,2)
            print("System Predicts:"+r.recognize_sphinx(audio))
            text = r.recognize_sphinx(audio)
            print('you said : {}',format(text))
            return text
        except Exception:
            print("No input given")
            return "No input given"
    
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
   
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
   
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
   
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
   
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    #concatenate higher and lower value
    value = ((high << 8) | low)
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

def oled_display(message1,message2):
    if "none" not in message2:
        disp.begin()
                # Clear display.
        disp.clear()
        disp.display()
        width = disp.width
        height = disp.height
        image = Image.new('1', (width, height))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        padding = -2
        top = padding
        bottom = height-padding
        d = 0
        font = ImageFont.load_default()

        print(message2)
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((d+35,top),message1, font=font,fill=255)
        draw.text((d+35, top+14), message2,  font=font, fill=500)
        disp.image(image)
        disp.display()
        sleep(4)
    
#try:
while True:
        strDate = datetime.datetime.now().strftime(dateString)
        result  = datetime.datetime.now().strftime(timeString)
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((d,top),strDate, font=font,fill=255)
        draw.text((d+35, top+14), result,  font=font, fill=500)
        draw.line((0, top+14, 127, top+14), fill=100)
        disp.image(image)
        disp.display()
        #time.sleep(.1)
        tf=pd.read_csv('prava_dataset.csv')
        x=tf.iloc[:, :-1].values
        y=tf.iloc[:, -1].values
        X_train,X_test,y_train,y_test=model_selection.train_test_split(x,y,test_size=0.5)
        knn=neighbors.KNeighborsClassifier(n_neighbors=1)
        knn.fit(X_train,y_train)
        tf1=pd.read_csv('falldetect.csv')
        a=tf1.iloc[:, :-1].values
        b=tf1.iloc[:, -1].values
        A_train,A_test,b_train,b_test=model_selection.train_test_split(a,b,test_size=0.5)
        knn1=neighbors.KNeighborsClassifier(n_neighbors=1)
        knn1.fit(A_train,b_train)
        i=0
        list1 =[]
        #sleep(0.5)
        print('start')
        command=get_speech_command()
        if 'No input given' in command:
            pass
        elif 'iris' in command or 'Iris' in command or 'ok' in command:
            message=get_speech_command()
            if 'No input given' not in message:
                send_message(message)
                oled_display(message,"")
        while i<=9:
            bus = smbus.SMBus(4)     # or bus = smbus.SMBus(0) for older version boards
            Device_Address = 0x68   # MPU6050 device address
            MPU_Init()
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = int(acc_x/10)
            Ay = int(acc_y/10)
            Az = int(acc_z/10)
            list1.extend([Ax,Ay,Az])
            i=i+1
            sleep(0.3)
        print(list1)
        l=np.array(list1).reshape(1,30)
        k=knn.predict(l)
        k1=knn1.predict(l)
        m=knn.predict_proba(l)
        print(m)
        print(k)
        print(k[0])
        if 'none' not in k[0]:
            send_message("Swetha\n"+k[0])
            oled_display("I need",k[0])
        print(k1)
        if k1=='failed':
            send_message("The person has fell down, need immediate assistance")
            oled_display("FELL DOWN","SEND SOS")
