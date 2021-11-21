## Project Overview

LiDAR is one of the prominent sensors to provide the 3D information of the object in terms of the point cloud to localize the objects and characterize the shapes.  For this project, a deep-learning approach is used to detect vehicles in LiDAR data ([Waymo Open dataset](https://waymo.com/open/)) based on a birds-eye view perspective of the 3D point-cloud. Detection performances such as Precision and Recall by comparing the ground truth labels with detection results are also evaluated. The steps of accomplishing 3D object detection are described below.

### Compute Lidar Point-Cloud from Range Image
#### Visualize range image channels ([`show_range_image`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L74) function)
Range Image format holds 3d points as a 360 degree "photo" of the scanning environment with the row dimension denoting the elevation angle of the laser beam and the column dimension denoting the azimuth angle. These are the steps taken to extract range image:
-   Convert range image `range` channel to 8bit.
-   Convert range image `intensity` channel to 8bit and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers.
-   Carry out negative values for visulization. 
-   Crop range image to +/- 90 deg. left and right of the forward-facing x-axis to focus on the scene at the front-view.

----
#### Visualize point-cloud
The goal of this task is to convert range image into lidar point-cloud using spherical coordinates then use the Open3D library to display it in a 3d viewer in order to develop a feel for the nature of lidar point-clouds. Below, there are 10 random images with varying degrees of visibility:

-   Find and display 6 examples of vehicles with varying degrees of visibility in the point-cloud
-   Identify vehicle features that appear as a stable feature on most vehicles (e.g. rear-bumper, tail-lights) and describe them briefly. Also, use the range image viewer from the last example to underpin your findings using the lidar intensity channel.

|   |
:-------------------------:|:-------------------------:
![Frame 1](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_1.png)  |  ![Frame 2](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_2.png)
![Frame 3](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_3.png)  |  ![Frame 4](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_4.png)
![Frame 5](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_5.png)  |  ![Frame 6](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_6.png)
![Frame 7](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_7.png)  |  ![Frame 8](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_8.png)
![Frame 9](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_9.png)  |  ![Frame 10](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_10.png)

----

### Create Birds-Eye View from Lidar PCL
### Model-based Object Detection in BEV Image
### Performance Evaluation for Object Detection