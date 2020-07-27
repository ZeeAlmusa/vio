# -*- coding: utf-8 -*-
"""
Created on Sat Jul 25 21:06:23 2020

@author: Zee Almusa
"""


import serial
import numpy as np
import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, os
from matplotlib.animation import PillowWriter

def warm_up(counter):
    
    if counter >= 200:
        return counter, True
    
    counter = counter + 1
    return counter, False
    


def get_sensors():
     
    serial_input = arduino.readline()
    print(serial_input)
    sensors = None
    if serial_input.isascii() and warmed_up:
        #[x_odometry, y_odometry, theta_odometry(yaw), yaw_imu, roll_imu, pitch_imu]
        try:
            sensors = np.array(serial_input.decode('ascii').rstrip('\r\n').split('/'), dtype=np.float) #[x,y,yaw_odometry, yaw_IMU]  
        except:
            return None
    
  
    return sensors
    

def get_odometer_state(odo_state, current_state):
    
    odo_state[0] += current_state[0] * np.cos(current_state[1])
    odo_state[1] += current_state[0] * np.sin(current_state[1])
    odo_state[2] = current_state[1]
    
    return odo_state

def get_IMU_state(IMU_state, current_state):
    
    
    IMU_state[0] += current_state[0] * np.cos(current_state[2])
    IMU_state[1] += current_state[0] * np.sin(current_state[2])
    IMU_state[2] = current_state[2]
    
    return IMU_state

def send_to_processing(odo_state, IMU_state, processing_socket):
   
    # message = (str(odo_state[0]) + "/" + str(odo_state[1]) \
    #            + "/" + str(odo_state[2]) + "/" + str(IMU_state[0]) + "/" \
    #            + str(IMU_state[1]) + "/" + str(IMU_state[2]) + "\n")
      
    message = ("{:.2f}".format(odo_state[0]) + "/" +"{:.2f}".format(odo_state[1]) \
                + "/" + "{:.2f}".format(odo_state[2]) + "/" + "{:.2f}".format(IMU_state[0])  \
                + "/" + "{:.2f}".format(IMU_state[1])+ "/" + "{:.2f}".format(IMU_state[2]) + "\n")
    
    # message = "{:.2f}".format(odo_state[0])
               
    print(message)
    
    processing_socket.send(str.encode(message))

def animation_frame(i, odo_path, IMU_path):
    
    odo_mask = np.where(odo_path.any(axis=1))[0]
    IMU_mask = np.where(IMU_path.any(axis=1))[0]
    
    odo_path = odo_path[odo_mask]
    IMU_path = IMU_path[IMU_mask]
    print(i)
    print("just the animation " ,odo_path.shape)
    ax.clear()
    
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    plt.title('Trajectory')
    plt.xlabel("x [cm]")
    plt.ylabel("y [cm]")
    ax.grid()
    
    markers_on = [len(odo_path[0:i])-1]
   
    ax.plot(odo_path[0:i,0], odo_path[0:i,1], marker=(3, 0, -90 + np.degrees(odo_path[i,2])), color='blue', linewidth=0.5, label = "odometry", markersize = 5, markevery=markers_on)
    ax.plot(IMU_path[0:i,0], IMU_path[0:i,1],marker=(3, 0, -90 + np.degrees(IMU_path[i,2])),  color='deeppink', linewidth=0.5, label = "IMU", markersize = 5, markevery=markers_on)


def plot_trajectory(odo_path, IMU_path):
    
    
    odo_mask = np.where(odo_path.any(axis=1))[0]
    IMU_mask = np.where(IMU_path.any(axis=1))[0]
    
    odo_path = odo_path[odo_mask]
    IMU_path = IMU_path[IMU_mask]
    
    print("just the plot " ,odo_path.shape)
    fig, ax = plt.subplots()
    ax.plot(odo_path[:,0], odo_path[:,1], color='blue', linewidth=0.5, label = "odometry")
    ax.plot(IMU_path[:,0], IMU_path[:,1], color='deeppink', linewidth=0.5, label = "IMU")
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    plt.title('Trajectory')
    plt.xlabel("x [cm]")
    plt.ylabel("y [cm]")
    ax.grid()
    plt.savefig("trajectory.png", dpi=300)
    plt.show()
    
    
########################################
#######         MAIN LOOP        ######
#######################################
    

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind(("127.0.0.1", 5205))
# s.listen(5)


arduino = serial.Serial('com6', 115200)

current_state = np.zeros(6)
old_state = np.zeros(6)
warmed_up = False
counter = 0 

odo_state = np.zeros(3)

IMU_state = np.zeros(3)

odo_path = np.zeros((10000,3))
IMU_path = np.zeros((10000,3))

i = 0

try:
    while True:
        
        # clientsocket, address = s.accept()
        # print(f"Connection from {address} has been established!")
        
       
        while True:
            
            old_state = current_state
            counter, warmed_up = warm_up(counter)
            
            if warmed_up:
                current_state = get_sensors()
                if current_state is None:
                    current_state = old_state
           
            
            odo_state = get_odometer_state(odo_state, current_state)
            IMU_state = get_IMU_state(IMU_state, current_state)
                    
            
            if(warmed_up):
                try:
                    # odo_state = current_state[0:3]
                    # IMU_state = current_state[3:6]
                   
                    odo_path[i,:] = odo_state[0:3]
                    IMU_path[i,:] = IMU_state[0:3]
                    print(odo_state)
                    i = i + 1
                except:
                    continue
                    
                
            
            if i >= 10000:
                break
                
            # print(current_state)
            # print(odo_state)
            # print(IMU_state)
            # print("================")
            #send_to_processing(odo_state, IMU_state, clientsocket)
    
except:
    
   
    arduino.close()
    plot_trajectory(odo_path, IMU_path)
    
    
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)  
    
writer = PillowWriter(fps=20)
  
fig, ax = plt.subplots()


ani = animation.FuncAnimation(fig, func = animation_frame, frames = np.arange(0,odo_path.shape[0],25), interval = 1, fargs=(odo_path, IMU_path))
plt.show()

ani.save('animation.gif', writer=writer)

 
arduino.close() 
# plot_trajectory(odo_path, IMU_path)
    