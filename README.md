

# Udacity Self-Driving Car Engineer Nanodegree

  

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/cover.png)


- Image: [Udacity Self-Driving Car Nanodegree](https://github.com/udacity/self-driving-car)

  
*Udacity Self-Driving Car Engineer Nano Degree projects.*

  
In this program, I learnt the techniques that power self-driving cars across the full stack of a vehicle’s autonomous capabilities. Using Deep Learning with radar and lidar sensor fusion, I also trained the vehicle to detect and identify its surroundings to inform navigation. This program is co-authored with Mercedes Benz and Waymo


## Projects:

  

- Finding Lane Lines

- Traffic Sign Classifier

- Object detection in urban enviroment

- 3D Object detection

- Sensor Fusion

- Scan Matching Localization

- Motion Planning

- Control and Trajectory tracking for autonomous vehicles

---

## Projects

### [Finding Lane Lines](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/tree/main/lane_line_detection)

The goal of the project is to **detect road lines** in an image taken from a roof-mounted camera. Then, **create detection pipeline** that can be applied to video stream from same camera. This project applies only **computer vision** techniques without any machine learning involed.

**Keywords**: Python, Computer Vision, Hough Transforms, Canny edge detection

#### Demo


![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/gif.gif)


---


### [Traffic Sign Classifier](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/tree/main/german_traffic_sign_classifier)

Using deep neural networks and CNN to classific german traffic signs using TensorFlow. Perform image pre-processing, data augmentation, and validation to guard against overfitting. After several experiments using different architectures, our select model is inspired by LeNet [1] architecture but fine-tuned to match our goal.
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/lowest_15_class_distribution.png)

-   **Keywords:**  Python, Deep Learning, CNN, TensorFlow, Data Augmentation, Hyperparameters.

#### Demo
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_1_Correct.png)

---
 
### [Object detection in urban enviroment](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/object_detection_urban_environment/project_writeup.md#object-detection-in-an-urban-environment)

Accurately detecting the surrounding objects can help autonomous vehicles react and prevent potential collisions. In this project, we use convolutional neural network approaches to detect and classify objects using data from  [Waymo Open dataset](https://waymo.com/open/), which provides driving scenes, will be used to train our neural network models. The dataset contains images of urban environments containing annotated cyclists, pedestrians and vehicles. I trained SSD_ResNet_50 using default hyperparamters and apply data augmentation to avoid overfitting.

-   **Keywords:**  Python, CNN, Object detection, ResNet, Pytorch, Data Augmentation, Bounding boxes, TensorFlow Object Detection API.

#### Demo
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/object_detection_urban_environment/images/animation.gif)

---
### [Sensor Fusion: 3D Object detection](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/writeup_midterm.md#3d-object-detection)

In this project, we first load and process 3D lidar point clouds and then use a deep-learning approach to detect vehicles in LiDAR data ([Waymo Open dataset](https://waymo.com/open/)) based on a birds-eye view perspective of the 3D point-cloud.  We then used precision and recall metrics to evaluate our model.

-   **Keywords:**  Python, CNN, Object detection, LiDar, Sensor Fusion, Bird eye, Bounding boxes, Object Detection, Point cloud.

#### Demo
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_object_detection.gif)

--- 

### [Sensor Fusion: 3D Object Tracking](https://github.com/DavidAbdelmalek/Self_Driving_Car_ND/blob/main/sensor_fusion_and_tracking/writeup_final.md#3d-object-tracking)

After fusing camera and lidar detections and detecting 3D objects, In this project we focus on multi-object tracking in real-world scenarios using Extended Kalman Filter. Along with creating tracking managemenet, updating and deleting tracks, assigning measurements to tracks with data association techniques, and managing several tracks simultaneously. 

-   **Keywords:**  Python, multi-object tracking, Object detection, Kalman Filter, Sensor Fusion, Point cloud, RSME.

#### Demo

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/sensor_fusion_and_tracking/img/3d_object_tracking.gif)


--- 


## Certificate 

<p align="center">
  <img src="https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/udacity_certificate_graduation.png" width="500" height="400">
</p>

[1]  [Traffic Sign Recognition with Multi-Scale Convolutional Networks](http://yann.lecun.com/exdb/publis/pdf/sermanet-ijcnn-11.pdf)
