
# 3D object detection


| Lidar 3D points                 | 3D object detection in BEV | Real-World Frame |
|:-------------------------------------:|:-------------------------------------:|:---|
| <img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_lidarPoints.png" width="600" height="350"> | <img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/bev_lidar.png" width="600" height="350"> | <img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/real_img.png" width="600" height="350">



## Project overview
LiDAR is one of the prominent sensors to provide the 3D information of the object in terms of the point cloud to localize the objects and characterize the shapes.  For this project, a deep-learning approach is used to detect vehicles in LiDAR data ([Waymo Open dataset](https://waymo.com/open/)) based on a birds-eye view perspective of the 3D point-cloud. Detection performances such as Precision and Recall by comparing the ground truth labels with detection results are also evaluated. The steps of accomplishing 3D object detection are described below.

### Project data structure
LiDAR scans can be represented in range images. This data structure holds 3d points as a 360 degree of scanned environment. With each incremental rotation around the z-axis, the lidar sensor returns a number of range and intensity measurements, which are then stored in the corresponding cells of the range image.

<img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/lidar_data_structure.jpg" alt="bev intensity channel" width="600"/>

## Project steps: 
### Compute Lidar Point-Cloud from Range Image
1. #### Visualize range image channels ([`show_range_image`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L74) function)
These are the steps taken to extract range image:
-   Convert range image `range` channel to 8bit.
-   Convert range image `intensity` channel to 8bit and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers.
-   Carry out negative values for visulization since they are used as flag for invalid points
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
## Object Detection
There is a pipeline for object detection which consists of three major parts, which are (1) data representation, (2) feature extraction and (3) model-based detection. The following figure shows the data flow through the pipeline with raw point cloud on one end and the classified objects on the other end:

<img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/object_detection_pipeline.jpg" alt="bev intensity channel" width="600"/>


### Create Birds-Eye View from Lidar PCL ([`bev_from_pcl`](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_pcl.py#L115)  function)

A detailed description of all required steps can be found in the code.
1. #### Convert sensor coordinates to bev-map coordinates
we will create the BEV map by first discretizing the cells and then converting the point coordinates from vehicle space to BEV space with width 608 and height 608 to be able to switch from `meter` to `pixels`. The visualization of 3D point cloud in the bev map coordinate is shown as follow:

<img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/bevmap.png" alt="bev map" width="600"/>

----
2. #### Compute intensity layer of bev-map
After we have created BEV coordinate transformation, we re-arrange elements in BEV by `x`, then `y`, then by decreasing `height` using `numpy.lexsort` in order to assign the intensity value of the top-most lidar point to the respective BEV pixel. Then, extract all points with identical `x` and `y` such that only the top-most z-coordinate is kept (use `numpy.unique`). Finally, normalized corresponding intensity value and mapped to 8-bit scale are taken into account.

<img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/bev_intensity.png" alt="bev intensity channel" width="600"/>

----
3. #### Compute height layer of bev-map
Same as we did for intensity map computation, the top-most z value in each cell is picked up and normalized with the difference between the maximum and the minimum height. The visualization of height map is shown below. 

<img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/bev_height.png" alt="bev height channel" width="600"/>

----
### Model-based Object Detection in BEV Image

The implementation can be found in  `load_configs_model`,  `create_model`  and  `detect_objects`  func. in  [objdet_detect.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_detect.py).

#### 1. Add a second model 

The model-based detection of objects in lidar point-clouds using deep-learning is a heavily researched area. The model selected is  [Super Fast and Accurate 3D Object Detection based on 3D LiDAR Point Clouds](https://github.com/maudzung/SFA3D).

The necessary configuration arguments for `fpn_resnet` are picked up and integrated into our codes. The input BEV map computed in the previous section is fed into the model for object inference and outputs further are decoded and post-processed. Finally, the bounding box information of detect objects in the BEV map can be acquired. For  `fpn_resnet`, the detection parameters of each bounding box in BEV map are structured as  `[score, x, y, z, h, w, l, yaw]`, where  `x, y`  (pixel coordinate),  `w, l`  (width and height),  `yaw`  (orientation angle) can be used to draw the predicted 2D bounding box. Below, you can find an example of detection output from `fpn_resnet`
[
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/example-detections-data.png) 
](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/example-detections-data.png)

----------

#### 2. Extract 3D bounding boxes from model response

After we obtain the detected object's information associated with the BEV map, we convert them into metric coordinates in vehicle space. The converted result is denoted as  `[1, x, y, z, h, w, l, yaw]`, where where `1` denotes the class id for the object type `vehicle`,  `x, y, z`  represents the object position,  `h, w, l`  represents the bounding box size, and  `yaw`  represents the bounding box's yaw angle. The ground-truth labels in 2D color image versus the detected bounding boxes in BEV map can be visualized:

[![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_object_detection.gif)](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/img/3d_object_detection.gif)


----
### Performance Evaluation for Object Detection

#### 1. Compute intersection-over-union between labels and detections (`measure_detection_performance`  func. in  [objdet_pcl.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_eval.py))

To evaluate the performance of Object Detection, we compute the geometrical bounding box overlap between ground-truth labels and detected objects. The min. IOU is set to 0.5. The object corners can be obtained based on  `x, y, w, l, yaw`. Then the intersection over union (IOU) between label and detected bounding box can be computed. In case of multiple matches, only the object/label pair with max. IOU is kept, and the true positives count is accumulated if the IOU value of a object/label is greater than min. IOU threshold. 

----------

#### 2. Compute false-negatives and false-positives (`measure_detection_performance`  func. in  [objdet_pcl.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_eval.py))

- **True Positives (TP)**: It shows the number of correctly classified objects in defined area. 
- **False Negatives (FN)**: It is the number of undetected objects
- **False Positives (FP)**: It represents the number of incorrect object predictions.

#### 3. Compute precision and recall (`compute_performance_stats`  func. in  [objdet_pcl.py](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/student/objdet_eval.py))

We compute  `precision`  and  `recall`  by processing around 100 frames in a image sequence. The  `precision`  is 1.0, and the  `recall`  is 0.67. The performance measures are plotted below:

Measurements              |  Count
:-------------------------:|:-------------------------:
TP  |  205
FN  |  101
FP  |  0


[![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/preformance_evaluation.png)](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/preformance_evaluation.png)
