# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 22:19:27 2020

@author: Potato
"""

import visual_odometer as viso
import numpy as np
import cv2
import matplotlib.pyplot as plt
import robot


camera = cv2.VideoCapture(0)
rover = robot.Robot()
vo = viso.VisualOdometer(rover.current_state,rover, camera)
vo.ground_truth = rover.odo_state
vo.prev_ground_truth = rover.odo_state

camera_path = np.zeros((10000,2))
odo_path = np.zeros((10000,2))
IMU_path = np.zeros((10000,2))


i = 0
current_point = np.zeros(3)


 
# try:
while(vo.hasNextFrame()):
   
    if not rover.warmed_up:
        
        rover.warm_up()
    
    else:
        
        frame = vo.current_frame
        
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        rover.update_state()
        #vo.prev_ground_truth = rover.old_odo
        vo.ground_truth = rover.odo_state
        if i % 5 == 0:
            vo.process_frame(rover)
        
     
        camera_path[i,0] = vo.get_mono_coordinates()[2]
        camera_path[i,1] = vo.get_mono_coordinates()[0]
        
        odo_path[i,0] = rover.odo_state[0]
        odo_path[i,1] = rover.odo_state[1]
        
        IMU_path[i,0] = rover.IMU_state[0]
        IMU_path[i,1] = rover.IMU_state[1]
        
        #print out path just to check if everything is working fine
        print(rover.odo_state)
        print("================")
            
        i = i + 1
        
        
plt.scatter(camera_path[:,0], camera_path[:,1], color = 'green' )        
plt.scatter(odo_path[:,0], odo_path[:,1], color = 'red')  
plt.scatter(IMU_path[:,0], IMU_path[:,1], color = 'blue')  


# except Exception as e:
#     rover.serial_port.close()
#     vo.camera.release()
#     cv2.destroyAllWindows()
    
#     print(e)
    
rover.serial_port.close()
vo.camera.release()
cv2.destroyAllWindows()
   