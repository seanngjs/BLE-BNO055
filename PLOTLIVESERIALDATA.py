# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 09:57:38 2019

@author: Sean
"""

import collections
import serial
import threading
import time    
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation    
import numpy as np
import scipy.integrate as itg


serial_port = 'COM13'; #Serial port that receives data from BLE Receiver Arduino
baud_rate = 115200; #In arduino, Serial.begin(baud_rate)
write_to_file_path = 'usbplottest_' + time.strftime("%Y%m%d-%H%M%S")+'.txt' #FOR USB
#write_to_file_path = 'BTplottest_' + time.strftime("%Y%m%d-%H%M%S")+'.txt' # FOR BLUETOOTH
output_file = open(write_to_file_path, "w+");
t = np.linspace(0,10,1000)
Ox = collections.deque([0]*1000, maxlen=1000) # Create an initial array with 1000 zero values for plotting
Oy = collections.deque([0]*1000, maxlen=1000)
Oz = collections.deque([0]*1000, maxlen=1000)
Ax = collections.deque([0]*1000, maxlen=1000)  
Ay = collections.deque([0]*1000, maxlen=1000)
Az = collections.deque([0]*1000, maxlen=1000)   
Vx = collections.deque([0]*1000, maxlen=1000) 
Vy = collections.deque([0]*1000, maxlen=1000)
Vz = collections.deque([0]*1000, maxlen=1000)
Sx = collections.deque([0]*1000, maxlen=1000) 
Sy = collections.deque([0]*1000, maxlen=1000)
Sz = collections.deque([0]*1000, maxlen=1000)
delta_t = collections.deque([0]*1000, maxlen=1000)  

def in_background(): #Reads incoming data and adds them to array and text file.
    arduinoData = serial.Serial(serial_port, baud_rate) 
    dataArrayNow = 0.0
    while True:  
        arduinoString = arduinoData.readline() #read the line of text from the serial port
        arduinoString = arduinoString.decode("utf-8")
#        arduinoString = arduinoString.encode("hex")
        arduinoString = arduinoString.strip()
#        print(arduinoString)
#        output_file.write(arduinoString +"\n")
        dataArray = arduinoString.split(',')   #Split it into an array called dataArray
        print(dataArray) #calibration format: Sys-Gyro-Acc-Mag (3=calibrated)
        for i in dataArray: 
            if i != "" : 
                if dataArrayNow != float(dataArray[8]):
#                    print('OK')
    #                t.append(float(dataArray[0]))
                    Ox.append(float(dataArray[1]))
                    Oy.append(float(dataArray[2]))
                    Oz.append(float(dataArray[3]))
                    Ax.append(float(dataArray[4]))
                    Ay.append(float(dataArray[5]))
                    Az.append(float(dataArray[6])) # 0.5 is the error correction factor
#                    delta_t.append(float(dataArray[7]))
                    dataArrayNow = float(dataArray[8])
                    Vx_v = np.trapz(Ax,None,0.01)
                    Vy_v = np.trapz(Ay,None,0.01)
                    Vz_v = np.trapz(Az,None,0.01)
                    Sx_v = np.trapz(Vx,None,0.01)
                    Sy_v = np.trapz(Vy,None,0.01)
                    Sz_v = np.trapz(Vz,None,0.01)
                    Vx.append(Vx_v)
                    Vy.append(Vy_v)
                    Vz.append(Vz_v)
                    Sx.append(Sx_v)
                    Sy.append(Sy_v)
                    Sz.append(Sz_v)
                    output_file.write(arduinoString +","
                                      + str(Vx_v) + ","
                                      + str(Vy_v) + ","
                                      + str(Vz_v) + ","
                                      + str(Sx_v) + ","
                                      + str(Sy_v) + ","
                                      + str(Sz_v) +"\n")


#Loops the in_background function
try:
    thread = threading.Thread(target = in_background)
    thread.start() 

except KeyboardInterrupt:
    sys.exit()

#Plots all 4 figures
fig = plt.figure()

ax1 = fig.add_subplot(4,1,1)
ax2 = fig.add_subplot(4,1,1)
ax3 = fig.add_subplot(4,1,1) 
line, = ax1.plot(t,Ox,label='Ox')
line2, = ax2.plot(t,Oy,label='Oy')
line3, = ax3.plot(t,Oz,label='Oz')
ax1.set_ylim(-360,360)
ax1.set_title('Orientation (deg)')
ax1.legend()

bx1 = fig.add_subplot(4,1,2)
bx2 = fig.add_subplot(4,1,2)
bx3 = fig.add_subplot(4,1,2)     
line4, = bx1.plot(t,Ax,label='Ax')
line5, = bx2.plot(t,Ay,label='Ay')  
line6, = bx3.plot(t,Az,label='Az')
bx1.set_ylim(-2,2)
bx1.set_title('Acceleration m/s^2')
bx1.legend()

cx1 = fig.add_subplot(4,1,3)
cx2 = fig.add_subplot(4,1,3)
cx3 = fig.add_subplot(4,1,3)   
line7, = cx1.plot(t,Vx,label='Vx')
line8, = cx2.plot(t,Vy,label='Vy')
line9, = cx3.plot(t,Vz,label='Vz')
cx1.set_ylim(-10,10)
cx1.set_title('Velocity m/s')   
cx1.legend()

dx1 = fig.add_subplot(4,1,4)
dx2 = fig.add_subplot(4,1,4)
dx3 = fig.add_subplot(4,1,4)    
line10, = dx1.plot(t,Sx,label='Sx')
line11, = dx2.plot(t,Sy,label='Sy')
line12, = dx3.plot(t,Sz,label='Sz')
dx1.set_ylim(-10,10)
dx1.set_title('Position m')
dx1.legend()

fig.subplots_adjust(hspace=1,wspace=1)
    
def animate(i): # Used to refresh the real-time plots at 200fps
    line.set_ydata(Ox)
    line2.set_ydata(Oy)
    line3.set_ydata(Oz)
    
    line4.set_ydata(Ax)
    line5.set_ydata(Ay)
    line6.set_ydata(Az)
    
    line7.set_ydata(Vx)
    line8.set_ydata(Vy)
    line9.set_ydata(Vz)

    line10.set_ydata(Sx)
    line11.set_ydata(Sy)
    line12.set_ydata(Sz)

ani = animation.FuncAnimation(fig, animate, interval=200)

plt.show()