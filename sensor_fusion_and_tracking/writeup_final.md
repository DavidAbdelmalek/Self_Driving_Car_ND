# 3D Object Tracking

## Project Overview
After fusing camera and lidar detections and detecting 3D objects, In this project we focus on multi-object tracking in real-world scenarios using Extended Kalman Filter. Along with creating tracking managemenet, updating and deleting tracks, assigning measurements to tracks with data association techniques, and managing several tracks simultaneously. This is an extension from [3D object detection project](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/writeup_midterm.md).

## Steps taken to implement Extended Kalman Filter (KMF): 

### 1. Filter ([filter.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/filter.py))

The Kalman Filter class for predicting/updating a vehicle's state  `x`  and estimation covariance error  `P`  is implemented. When measurement data from lidar or color camera is received,  `x`,  `P`  at timestamp  `k`  is updated, and then predicted at timestamp  `k+1`. The single target tracking results and its corresponding RMSE plot (the residual between estimated state and ground truth state) are shown below.

![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/filter.png](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/filter.png)
![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/filter_RMSE.png](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/filter_RMSE.png)
----
### 2. Track Management ([trackmanagement.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/trackmanagement.py))

In order to handle multiple tracks and measurements, we need multi-tracking system to fulfill such tasks. The track management is one of the main blocks of multi-tracking systems which has specific tasks:
    -   **Initialize new tracks**: A new track is initialized if any measurement is left in the unassigned measurement list. The `x` and `P` are initialized based on the unassigned measurement transformed from Sensor coordinate to Vehicle coordinate. 
    - **Set track score & track state**:  The track score for a new track is initialized with `1./params.window` and the track state is set to `initialized`. A track's score will be accumulated if it is associated with measurements in next several frames, while a track's score will be decreased if it is unassigned and out of the sensor's field of view. A track's state is set to `tentative` if its track score is between 0.2 and 0.8. If the score is over 0.8, the state is set to `confirmed`. 
    -   **Delete old tracks**: For `confirmed` track, it will be deleted if it drops below 0.6 or its estimation error covariance `P` is too large. While for `tentative` tracks, If a track's score is too low or `P` is too large.
The RMSE plot for one single track without track losses in between is shown below. The mean RMSE is 0.78.

![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/track_RMSE.png](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/track_RMSE.png)
----
### 3. Association ([association.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/association.py))

The  data associationdata  association  assigns measurements to tracks and decides which track to update with which measurement.  Single Nearest Neighbor (SNN) algorithm is used to  associate measurements to tracks based on  `Mahalanobis Distance` which  is applied to compute the distance between a track and a measurement. In order to reduce association complexity, `Gating` is used to reduce the association complexity by removing unlikely association pairs with chi-square-distribution. The Gating threshold is set to 0.995 in order to not remove too many correct measurements.

Steps to update associate matrix at each timestamp:
-  Initialise `association_matrix`  with the actual association matrix based on Mahalanobis distances for all tracks in the input  `track_list`  and all measurements in the input  `meas_list`. 
- Compute `MHD()` to track the distance between track and measurement.
- Use `gating()`  function to check if a measurement lies inside a track's gate.
- Update the list of unassigned measurements  `unassigned_meas`  and unassigned tracks  `unassigned_tracks`  to include the indices of all measurements and tracks that did not get associated. The following visualizations showed a multiple object tracking scenario with approximately 200 frames.

![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/association_RMSE.png](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/association_RMSE.png)
----
### 4. Camera-lidar fusion ([measurements.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/measurements.py))

The fusion of Lidar and Color camera sensors for predicting object motions is implemented. The measurement values for Lidar sensor is 3D positions `x, y, z`. The measurement noise covariance matrix `R` is 3x3. On the other hand, for the color camera sensor, only 2D positions `u, v` in image space can be measured. Hence, its covariance matrix `R` is 2x2. Unlike the Lidar sensor, the color camera sensor relies on a nonlinear measurement model h(x) to convert vehicle's state `x` to 2D measurement space. To get the residual covariance `S` and kalman gain `K`, the nonlinear function h(x) is linearized as Jacobian H. The tracking updates all tracks when Lidar or Color camera measurements arrives to the system. The visualization of multiple object tracking results along with the RMSE can be seen below:

![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_object_tracking.gif](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_object_tracking.gif)

![https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/tracking_RMSE.png](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/tracking_RMSE.png)
---
### Project challenges

Implementing . However, track measurement and association was not trivial to complete. Besides, constructing non-linear `hx` and Jacobian `H`  was very challenging as It needs strong mathematics background and understanding of Gaussian distributions.

----
### Cons in using camera-lidar fusion compared to lidar-only tracking

By adding the second sensor color camera, the object detection can be strengthen thanks to the RGB information, which compensates the shortage part from lidar sensor and enrich the system with image with high resolution that at the end can help improve trackings.

--- 
### Sensor Fusion Challenges in real-life scenarios

Since each sensor type may have a different field of view, one concern is that how to deal with invisible/occlusion areas that some of sensors cannot capture while maintaining high accuracy for object detection. While fully covering the area of an ego car's surroundings may improve the detection, the calibration of heterogeneous sensors is still challenging.

---
### Possible improvements
Add more sensors to cover a wider area, for instance, radars to detect the object's position/velocity, stereo cameras for 2D/3D information.