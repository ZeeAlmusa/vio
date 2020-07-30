# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 23:33:27 2020

@author: Zee Almusa
"""
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, os
from matplotlib.animation import PillowWriter
import copy

class Robot(object):
    
    #Default constructor for using a live sequence (camera)
    def __init__(self, 
                 serial_port = serial.Serial('com6', 115200),
                 current_state = np.zeros(3),
                 old_state = np.zeros(3),
                 odo_state = np.zeros(3),
                 old_odo_state = np.zeros(3),
                 IMU_state = np.zeros(3),
                 counter = 0,
                 warmed_up = False):
        
        self.serial_port = serial_port
        self.current_state = current_state
        self.old_state = old_state
        self.warmed_up = warmed_up
        self.counter = counter
        self.odo_state = odo_state
        self.IMU_state = IMU_state
                                    # Start the execution
    
    def warm_up(self):
        
        if self.counter <= 200:
            self.serial_port.readline()
            self.counter += 1
           
        else:
            self.warmed_up = True
    
    def get_sensors(self):
        
        self.old_state = self.current_state
        try:
            serial_input = self.serial_port.readline()
            
        
        except:
            print("No robot data available.")
            serial_input = b'0/0/0\r\n'
        #print(serial_input)
        
        sensors = None
        if serial_input.isascii():
            #[d_center, theta, yaw]
            try:
                sensor = np.array(serial_input.decode('ascii').rstrip('\r\n').split('/'), dtype=np.float) #[x,y,yaw_odometry, yaw_IMU]  
                if len(sensor) == 3:
                    self.current_state = sensor
                    
            except Exception as e:
                print("Sensor data is not the correct format.")
                pass
        self.serial_port.flush()
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        
        
        
    def get_odometer_state(self):
        
        self.old_odo = copy.deepcopy(self.odo_state)
        
        self.odo_state[0] += self.current_state[0] * np.cos(self.current_state[1])
        self.odo_state[1] += self.current_state[0] * np.sin(self.current_state[1])
        self.odo_state[2] = self.current_state[1]
    
       


    def get_IMU_state(self):
    
        
        self.IMU_state[0] += self.current_state[0] * np.cos(self.current_state[2])
        self.IMU_state[1] += self.current_state[0] * np.sin(self.current_state[2])
        self.IMU_state[2] = self.current_state[2]
        
    
    def update_state(self):
        
        self.get_sensors()
        self.get_odometer_state()
        self.get_IMU_state()
        
        
    
       