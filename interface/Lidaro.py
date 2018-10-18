import socket
from threading import Thread
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import numpy as np
import math as m
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,\
    NavigationToolbar2TkAgg
import tkinter as Tk
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D 
from tkinter import messagebox

pltInterval = 10

d = 300

xmin = -d
xmax = d
ymin = -d
ymax = d
zmin = -d
zmax = d

def sphe2cart (r, theta, phi):
    print(str(r) + " " + str(theta))
    theta = np.deg2rad(theta)+m.atan2(14.5,r)
    phi = np.deg2rad(phi)
    x = np.cos(theta) * np.sin(phi) * (r + abs(25*np.cos(theta)))
    y = np.sin(phi) * np.sin(theta) * (r + abs(25*np.sin(theta)))
    z = np.cos(phi) * r
    #print("cart " , str(x), str(y), str(z))
    return x, y, z

def rot_3D(xcal, ycal, zcal, point):
    a = np.deg2rad(xcal)
    b = np.deg2rad(ycal)
    c = np.deg2rad(zcal)
    rot_matrix = [[np.cos(b)*np.cos(c), np.cos(a)*np.sin(c)+np.sin(a)*np.sin(b)*np.cos(c), np.sin(a)*np.sin(c)-np.cos(a)*np.sin(b)*np.cos(c)],
                  [-np.cos(b)*np.sin(c), np.cos(a)*np.cos(c)-np.sin(a)*np.sin(b)*np.sin(c), np.sin(a)*np.cos(c)+np.cos(a)*np.sin(b)*np.sin(c)],
                  [np.sin(b)          , -np.sin(a)*np.cos(b)                             , np.cos(a)*np.cos(b)]]
    return np.matmul(rot_matrix,point)

class wifiPlot:
    def __init__(self, Port_r = 8090, Port_s = 8091, ip = "192.168.0.197"):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rawData = 0
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.ip = ip
        self.mode = 1
        self.port_s = Port_s
        self.x = []
        self.y = []
        self.z = []
        self.receiver.bind(('0.0.0.0', Port_r))
        self.sender.bind(('0.0.0.0', Port_s))
        self.receiver.settimeout(1)
        self.receiver.listen(0)
        self.xcal = 0
        self.ycal = 0
        self.zcal = 0
        self.lastTime = 0
        self.now = 0
    def readWifiStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread, daemon=True)
            self.thread.start()
            
    def decode(self, msg):
        msg = msg.decode().strip().split(" ")
        return msg
    
    def backgroundThread(self):   
        while self.isRun:
            try:
                client, addr = self.receiver.accept()
                self.isReceiving = True
                while self.isRun:
                    content = client.recv(32)
                    if len(content) == 0:
                        break
                    else:
                        self.rawData = self.decode(content)
                        #print(str(self.rawData[0]) + " " +str(self.rawData[1]) + " " +str(self.rawData[2]))
                client.close()
            except :
                pass
            
    def getWifiData(self,num):
        while self.isReceiving != True:
                time.sleep(0.1)
                return graph,
        
        self.now = int(round(time.time() * 1000))   
        if(self.now-self.lastTime > 700):
            self.lastTime =self.now
            lbl_online.configure(text="offline", bg="red")    
        
        if(len(self.rawData) > 0):
            self.lastTime = self.now
            lbl_online.configure(text="online", bg="green")
            if (self.rawData[0] == "ping"):
                pass
            elif(self.rawData[0] == 'c0'):
                print(self.rawData)
                self.zcal = float(self.rawData[1])
                self.ycal = -float(self.rawData[3])
                self.xcal = float(self.rawData[2])
            
            elif (self.rawData[2].isdigit() and float(self.rawData[2]) > 0):
                if(self.mode == '0'):
                    xx, yy, zz = rot_3D(self.xcal, self.ycal, self.zcal,sphe2cart(float(self.rawData[2]), float(self.rawData[1]),90))
                else:
                    xx, yy, zz = rot_3D(self.xcal, self.ycal, self.zcal, sphe2cart(float(self.rawData[2]), float(self.rawData[1]),float(self.rawData[0])))
                self.x.append(xx)
                self.y.append(yy)
                self.z.append(zz)

                graph._offsets3d = (self.x,self.y,self.z)
                
            self.rawData = []
            return graph,
        
    
    def send_value(self, value):
        self.sender.sendto(value.encode(),(self.ip, self.port_s))
        
    def close(self):
        self.isRun = False
        self.thread.join()
        
    def reset(self):
        self.x = []
        self.y = []
        self.z = []

    def make_data_set(self):
        raw_data = {'X': self.x,
                    'Y': self.y,
                    'Z': self.z}
        df = pd.DataFrame(raw_data, columns = ['X', 'Y', 'Z'])
        df.to_csv('example.csv')
    
def clicked_onoff():
    if btn_onoff['text'] == "Start":
        btn_onoff.configure(text="Stop")
        lbl_onoff.configure(text="  ON  ", bg="green")
        c.send_value("o1")
        if(c.mode == '1'):
            c.send_value("e"+str(sld_vel_spin.get()))
        c.send_value("a"+str(sld_vel_osc.get()))
    else:
        btn_onoff.configure(text="Start")
        lbl_onoff.configure(text="  OFF ", bg="red")
        c.send_value("o0")
        c.send_value("a0")
        c.send_value("e0")

        
def getVelOsc(event):
    c.send_value("a"+str(sld_vel_osc.get()))

def getVelSpin(event):
    c.send_value("e"+str(sld_vel_spin.get()))

def mode_chg():
    c.mode = v.get()
    print(c.mode)
    if (c.mode == '0'):
        sld_vel_spin.config(state='disabled')
        c.send_value("e0")
        print("disable")
    else:
        sld_vel_spin.config(state='normal')
        print("enable")

def on_closing():
    c.send_value("o0")
    c.send_value("a0")
    c.send_value("e0")
    c.close()
    root.destroy()
    root.quit()
    print("Bye! :)")


c = wifiPlot()
c.readWifiStart()

root = Tk.Tk()
root.wm_title("Lidar interface")

fig = plt.figure(figsize=(5,5), dpi=100)
canvas = FigureCanvasTkAgg(fig, master = root)
canvas.get_tk_widget().grid(column=0,row=5)
canvas._tkcanvas.grid(column=0,row=5)

ax = Axes3D(fig)

graph = ax.scatter([0],[0],[0], color = "red", s =1)
ax.set_xlim3d([xmin, xmax])
ax.set_ylim3d([ymin, ymax])
ax.set_zlim3d([zmin, zmax])             
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_ylabel("Z")
anim = animation.FuncAnimation(fig, c.getWifiData,frames = 100, interval=pltInterval)

button_frame = Tk.Frame(root)
button_frame.grid(column=1,row=5, padx= 10, pady=10)

btn_onoff = Tk.Button(button_frame, text="Start", command=clicked_onoff)
btn_onoff.grid(column=1, row=0, padx= 10, pady=10)
lbl_onoff = Tk.Label(button_frame, text="  OFF ", bg="red")
lbl_onoff.grid(column=2, row=0, padx= 10, pady=10)

sld_vel_osc = Tk.Scale(button_frame, from_=0, to=5, orient="horizontal", command=getVelOsc)
sld_vel_osc.grid(column=2, row=1, padx= 10, pady=10)
lbl_vel_osc = Tk.Label(button_frame, text="Velocidade Az")
lbl_vel_osc.grid(column=1, row=1, padx= 10, pady=10)

sld_vel_spin = Tk.Scale(button_frame, from_=0, to=5, orient="horizontal", command=getVelSpin)
sld_vel_spin.grid(column=2, row=2, padx= 10, pady=10)
lbl_vel_spin = Tk.Label(button_frame, text="Velocidade Alt")
lbl_vel_spin.grid(column=1, row=2, padx= 10, pady=10)

v = Tk.StringVar()
v.set(c.mode)
for text, mode in [("2D", 0),("3D", 1)]:
        btn_mode = Tk.Radiobutton(button_frame,variable=v, text=text, value=mode, indicatoron = 0, command=mode_chg)
        btn_mode.grid(row=3, column=mode+1)
btn_mode = Tk.Label(button_frame, text="Modo")
btn_mode.grid(column=0, row=3, padx= 10, pady=10)

btn_reset = Tk.Button(button_frame, text="Reset", command=c.reset)
btn_reset.grid(column=3, row=4)

btn_reset = Tk.Button(button_frame, text="To CSV", command=c.make_data_set)
btn_reset.grid(column=3, row=5)

lbl_online = Tk.Label(button_frame, text="offline", bg="red")
lbl_online.grid(column=3, row=0, padx= 10, pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
Tk.mainloop()

