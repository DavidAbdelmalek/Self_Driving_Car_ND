
## Project Overview

LiDAR is one of the prominent sensors to provide the 3D information of the object in terms of the point cloud to localize the objects and characterize the shapes.  For this project, a deep-learning approach is used to detect vehicles in LiDAR data ([Waymo Open dataset](https://waymo.com/open/)) based on a birds-eye view perspective of the 3D point-cloud. Detection performances such as Precision and Recall by comparing the ground truth labels with detection results are also evaluated. The steps of accomplishing 3D object detection are described below.

### Compute Lidar Point-Cloud from Range Image
1. #### Visualize range image channels ([`show_range_image`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L74) function)
Range Image format holds 3d points as a 360 degree "photo" of the scanning environment with the row dimension denoting the elevation angle of the laser beam and the column dimension denoting the azimuth angle. These are the steps taken to extract range image:
-   Convert range image `range` channel to 8bit.
-   Convert range image `intensity` channel to 8bit and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers.
-   Carry out negative values for visulization. 
-   Crop range image to +/- 90 deg. left and right of the forward-facing x-axis to focus on the scene at the front-view.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/range_img.png)
----
2. #### Visualize point-cloud  ([`show_pcl`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L43) function)
The goal of this task is to convert range image into lidar point-cloud using spherical coordinates then use the Open3D library to display it in a 3d viewer in order to develop a feel for the nature of lidar point-clouds. Below, there are 10 random images with varying degrees of visibility:

 ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_2.png)             |  ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_1.png) |
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_3.png)  |  ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_4.png)
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_5.png)  |  ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_7.png)
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_8.png)  |  ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_6.png)
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_9.png)  |  ![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/lidar_10.png)

---- 
### Create Birds-Eye View from Lidar PCL ([`bev_from_pcl`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L115)  function)

A detailed description of all required steps can be found in the code.
1. #### Convert sensor coordinates to bev-map coordinates
we will create the BEV map by first discretizing the cells and then converting the point coordinates from vehicle space to BEV space with width 608 and height 608 to be able to switch from `meter` to `pixels`. The visualization of 3D point cloud in the bev map coordinate is shown as follow:
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/bev_map.png)

----
2. #### Compute intensity layer of bev-map
After we have created BEV coordinate transformation, we re-arrange elements in BEV by `x`, then `y`, then by decreasing `height` using `numpy.lexsort` in order to assign the intensity value of the top-most lidar point to the respective BEV pixel. Then, extract all points with identical `x` and `y` such that only the top-most z-coordinate is kept (use `numpy.unique`). Finally, normalized corresponding intensity value and mapped to 8-bit scale are taken into account.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/bev_intensity.png)


----
3. #### Compute height layer of bev-map
Same as we did for intensity map computation, the top-most z value in each cell is picked up and normalized with the difference between the maximum and the minimum height. The visualization of height map is shown below. 
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/71c679a2317558b6859b4cd3175f7c29c9cc44e8/sensor_fusion_and_tracking/img/bev_height.png)

----
### Model-based Object Detection in BEV Image
### Performance Evaluation for Object Detection