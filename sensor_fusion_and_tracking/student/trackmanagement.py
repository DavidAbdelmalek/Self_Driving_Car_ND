# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############
        
        # Initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinate
        z = np.vstack((meas.z,np.ones(1)))
        z[:3,:]= meas.sensor.sens_to_veh[:3,:3]* z[:3,:]
        
        M = np.zeros((4,4))
        M[:3,:3] = M_rot
        M[:,3] = np.vstack((np.ones((3,1)) * params.dt,np.ones(1))).ravel()
        
        state = M * z
        self.x = np.vstack((meas.z,np.zeros((int(params.dim_state/2),1))))
        
        # set up velocity estimation error covariance
        sigma_p44 = params.sigma_p44
        sigma_p55 = params.sigma_p55
        sigma_p66 = params.sigma_p66
        P_vel = np.matrix([[sigma_p44**2, 0, 0],
                           [0, sigma_p55**2, 0],
                           [0, 0, sigma_p66**2]])
        
        # set up postion estimation error covariance
        P_pos = M_rot * meas.R * M_rot.T
        
        # Combine P_pos and P_vel into P (covariance matrix).
        self.P = np.zeros((6,6))
        self.P[:3,:3] = P_pos
        self.P[3:,3:] = P_vel
        
        # While the rest is P_velocity which is zero for Lidar sensor.
        self.state = 'initialized'
        self.score = 1./params.window
    
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        ############
        # Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big.
        ############
        
        # decrease score for unassigned tracks
        for i in unassigned_tracks:
            track = self.track_list[i]
            score = track.score
            # check visibility    
            if meas_list: # if not empty
                if meas_list[0].sensor.in_fov(track.x):
                    # decrease score by some constant
                    score -= 1./params.window
                    # update track score
                    track.score = score
        
        # Update old tracks
        for track in self.track_list : 
            # delete old tracks   
            P = track.P
            if track.state == 'confirmed':
            # delete confirmed tracks if the score is below threhold or P is too big
                if track.score < params.delete_threshold or track.P[0,0] > params.max_P or track.P[1,1] > params.max_P:
                    self.delete_track(track)
                     
            # delete tracks if the score is too low or P is too big
            elif track.score < 0.1 or track.P[0,0] > params.max_P or track.P[1,1] > params.max_P:
                self.delete_track(track)
    
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self, track):      
        ############
        # Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############
        score = track.score
        
        # Increase track score
        score += 1./params.window
        
        # Update track state
        if score > params.confirmed_threshold:
            track.state = 'confirmed'
        else:
            track.state = 'tentative'
            
        # update score
        track.score = score