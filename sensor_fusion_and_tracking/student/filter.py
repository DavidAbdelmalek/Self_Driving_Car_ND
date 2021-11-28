# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dt = params.dt
        self.q  = params.q
        
    def F(self):
        ############
        # Implement and return system matrix F
        ############
        
        dt = self.dt
        return np.matrix([[1, 0, 0, dt, 0, 0],
                          [0, 1, 0, 0, dt, 0],
                          [0, 0, 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]])
      
    def Q(self):
        ############
        # Implement and return process noise covariance Q
        ############
        
        dt = self.dt
        q = self.q
        q1 = ((dt**3)/3) * q
        q2 = ((dt**2)/2) * q
        q3 = dt * q

        return np.matrix([[q1, 0, 0, q2, 0, 0],
                          [0, q1, 0, 0, q2, 0],
                          [0, 0, q1, 0, 0, q2],
                          [q2, 0, 0, q3, 0, 0],
                          [0, q2, 0, 0, q3, 0],
                          [0, 0, q2, 0, 0, q3]])

                         
    def predict(self, track):
        
        ############
        # Predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        
        F = self.F()
        Q = self.Q()
        x = track.x
        P = track.P
        x = F*x # state prediction
        P = F*P*F.T + self.Q() # covariance 
        
        # update x and P 
        track.set_x(x)
        track.set_P(P)
         
    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        P = track.P
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        gamma = self.gamma(track,meas)
        
        # Calculate kalman gain
        K =  P * H.T * np.linalg.inv(S)
        x = track.x + K*gamma # state update
        I = np.identity(params.dim_state)
        P = (I - K*H) * P # covariance update
        
        # update x and P 
        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # Calculate and return residual gamma
        ############
        # Get the measurement function evaluated at the current state, h(x).
        hx = meas.sensor.get_hx(track.x)
        gamma = meas.z - hx
        return gamma

    def S(self, track, meas, H):
        ############
        # Calculate and return covariance of residual S
        ############
        S = H * track.P * H.T + meas.R
        return S

