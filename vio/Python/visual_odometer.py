# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 14:19:23 2020

@author: Zee Almusa
modified from https://github.com/alishobeiri/Monocular-Video-Odometery.git

"""
import numpy as np
import cv2
import os
import copy


class VisualOdometer(object):
    
    #Default constructor for using a live sequence (camera)
    def __init__(self, 
                ground_truth,
                rover,
                camera,
                camera_intrinsics = np.array([[474.00206465 ,  0.  ,       324.64215794],
                                              [  0.    ,    477.42451785, 261.96670994],
                                              [  0.    ,       0.     ,      1.        ]]),
                lk_params=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)), 
                detector=cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)):
        
        self.ground_truth = ground_truth
        self.prev_ground_truth = np.zeros(2)
        self.camera = camera
        self.detector = detector
        self.lk_params = lk_params
        self.camera_instrinsics = camera_intrinsics
        self.R = np.eye(3)
        self.t = np.zeros(shape=(3, 3))
        self.id = 0
        self.n_features = 0
        self.pose = np.zeros(3)
        self.current_frame = None
        self.previous_frame = None
        self.live = True
        self.thing = np.ones(3)
        self.rover = rover
        #Note: I use this setting for my 16-bit grayscale camera.
        #You might not need this if you're camera is outputs a different format
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        camera.set(cv2.CAP_PROP_CONVERT_RGB, False)
        camera.set(cv2.CAP_PROP_BRIGHTNESS, 10)
        
        self.process_frame(rover)
    
    #Construct with this if you want to load a saved image sequence that you have
    def image_sequence(self, 
            img_file_path,
            pose_file_path,
            focal_length = 718.8560,
            pp = (607.1928, 185.2157), 
            lk_params=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)), 
            detector=cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)):
        
        self.file_path = img_file_path
        self.pose_file_path = pose_file_path
        self.detector = detector
        self.lk_params = lk_params
        self.focal = focal_length
        self.pp = pp
        self.R = np.eye(3)
        self.t = np.zeros(shape=(3, 3))
        self.id = 0
        self.n_features = 0
        self.current_frame = None
        self.previous_frame = None
        self.live = False
        
        try:
            if not all([".png" in x for x in os.listdir(img_file_path)]):
                raise ValueError("img_file_path is not correct and does not exclusively png files")
        except Exception as e:
            print(e)
            raise ValueError("The designated img_file_path does not exist, please check the path and try again")

        try:
            with open(pose_file_path) as f:
                self.pose = f.readlines()     
        except Exception as e:
            print(e)
            raise ValueError("The pose_file_path is not valid or did not lead to a txt file")
    
    
    
    def hasNextFrame(self):
        '''Used to determine whether there are remaining frames
           in the folder to process
        
        Returns:
            bool -- Boolean value denoting whether there are still 
            frames in the folder to process
        '''
        if self.live:
            return True
      
        else:
            return self.id < len(os.listdir(self.file_path)) 
    
    
    def detect(self, img):
        '''Used to detect features and parse into useable format
        
        Arguments:
            img {np.ndarray} -- Image for which to detect keypoints on
        
        Returns:
            np.array -- A sequence of points in (x, y) coordinate format
            denoting location of detected keypoint
        '''

        p0 = self.detector.detect(img)
        
        return np.array([x.pt for x in p0], dtype=np.float32).reshape(-1, 1, 2)
    
    
    def visual_odometery(self, rover):
        
        if self.n_features < 2000:
            self.p0 = self.detect(self.previous_frame)
        
        self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.previous_frame, self.current_frame, self.p0, None, **self.lk_params)
            
        self.good_old = self.p0[st == 1]
        self.good_new = self.p1[st == 1]
        
        if self.id < 2:
            E, _ = cv2.findEssentialMat(self.good_new, self.good_old, self.camera_instrinsics, cv2.RANSAC, 0.999, 1.0, None)
            _, self.R, self.t, _ = cv2.recoverPose(E, self.good_old, self.good_new, self.camera_instrinsics)
        else:
            E, _ = cv2.findEssentialMat(self.good_new, self.good_old, self.camera_instrinsics, cv2.RANSAC, 0.999, 1.0, None)
            _, R, t, _ = cv2.recoverPose(E, self.good_old, self.good_new, self.camera_instrinsics)
            
            #insert methods to get ground_truth and update previous ground_truth
           
            absolute_scale = self.get_absolute_scale(rover.odo_state, rover.old_odo)
            print(self.prev_ground_truth)
            print(self.ground_truth)
            print("{:.6f}".format(absolute_scale))
            if (absolute_scale > 0.01 and abs(t[2][0]) > abs(t[0][0]) and abs(t[2][0]) > abs(t[1][0])):
                
                absolute_scale = self.get_absolute_scale(self.ground_truth, self.prev_ground_truth)
                print("other scale = ", absolute_scale)
                self.t = self.t + absolute_scale*self.R.dot(t)
                self.R = R.dot(self.R)
                self.prev_ground_truth = rover.old_odo
                
        self.n_features = self.good_new.shape[0]
    
    def get_mono_coordinates(self):
        # We multiply by the diagonal matrix to fix our vector
        # onto same coordinate axis as true values
        diag = np.array([[-1, 0, 0],
                        [0, -1, 0],
                        [0, 0, -1]])
        adj_coord = np.matmul(diag, self.t)

        return adj_coord.flatten()
    
    
    
    def get_absolute_scale(self, new_distance, old_distance):
        '''Used to provide scale estimation for mutliplying
           translation vectors
        
        Returns:
            float -- Scalar value allowing for scale estimation
        '''
        if self.live:
            true_vect = new_distance
            prev_vect = old_distance
            
        else:
            pose = self.pose[self.id - 1].strip().split()
            x_prev = float(pose[3])
            y_prev = float(pose[7])
            z_prev = float(pose[11])
            pose = self.pose[self.id].strip().split()
            x = float(pose[3])
            y = float(pose[7])
            z = float(pose[11])
    
            true_vect = np.array([[x], [y], [z]])
            self.true_coord = true_vect
            prev_vect = np.array([[x_prev], [y_prev], [z_prev]])
        
        return np.linalg.norm(true_vect - prev_vect)
    
    
    def process_frame(self, rover=None):
      
        
        if self.id < 2:
            if self.live:
                ret1, self.previous_frame = self.camera.read()
                ret2, self.current_frame = self.camera.read()
                
                while not ret1 and not ret2:
                    
                    ret1, self.previous_frame = self.camera.read()
                    ret2, self.current_frame = self.camera.read()
                    
                self.previous_frame = self.previous_frame*15
                self.current_frame = self.current_frame*15
                
                self.previous_frame = (self.previous_frame >> 8).astype('uint8')
                self.current_frame = (self.current_frame >> 8).astype('uint8')
                
                
                self.visual_odometery(rover)
                self.id = 2
            else:
                self.previous_frame = cv2.imread(self.file_path +str().zfill(6)+'.png', 0)
                self.current_frame = cv2.imread(self.file_path + str(1).zfill(6)+'.png', 0)
                self.visual_odometery()
                self.id = 2
            
        else:
            if self.live:
                self.previous_frame = self.current_frame
                ret2, self.current_frame = self.camera.read()
                
                
                self.current_frame = self.current_frame*15
                
                self.current_frame = (self.current_frame >> 8).astype('uint8')
                self.visual_odometery(rover)
                self.id += 1
            else:
                self.previous_frame = self.current_frame
                self.current_frame = cv2.imread(self.file_path + str(self.id).zfill(6)+'.png', 0)
                self.visual_odometery()
                self.id += 1
     