
# **Finding Lane Lines on the Road**

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

  

## Overview

When we drive, we use our eyes to decide where to go. The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle. Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm.

  

In this project you will detect lane lines in images using Python and OpenCV. OpenCV means "Open-Source Computer Vision", which is a package that has many useful tools for analyzing images.

  

To complete the project, two files will be submitted: a file containing project code and a file containing a brief write up explaining your solution. We have included template files to be used both for the [code](https://github.com/udacity/CarND-LaneLines-P1/blob/master/P1.ipynb) and the [writeup](https://github.com/udacity/CarND-LaneLines-P1/blob/master/writeup_template.md).The code file is called P1.ipynb and the writeup template is writeup_template.md

  

To meet specifications in the project, take a look at the requirements in the [project rubric](https://review.udacity.com/#!/rubrics/322/view)

---

## Goal of this project

The goal of the project is to   **detect road lines**  in an image taken from a roof-mounted camera. Then, **create detection pipeline** that can be applied to video stream from same camera. This project applies only  **computer vision**  techniques without any machine learning involed.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/gif.gif)

### Pipeline steps:
#### 1. Extract frames from video streaming and process them one by one.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/test_imgs.png)
#### 2. Convert img to grayscale with one channel.
- Applies the Grayscale transform. This will return an image with only one color channel.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/gray.png) 
#### 3. Apply gaussian distribtuion / blurring.
- To suppress noise and spurios gradients


![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/blur.png)
#### 4.  Apply canny transform detection
- Canny algorithm is used to detect the locations with a gradient in color change.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/canny.png)
#### 6. Extract region (ROI):
- Applies an image mask. Only keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black. `vertices` should be a numpy array of integer points.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/roi.png)
#### 7. Apply hough transformation
- To find lines out of the dots of all the edges processed with Canny function, we should use a model of a line (y = mx + b). Then we can fit that model to the assortment of dots in the edge detection image. For that purpose we use the Hough Transform, which represents a line in the image space as a dot after transformation and a point in image space as a line. So we are looking for intersecting lines in Hough space to identify lines in image space.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/hough_space.jpg)
Image: [Udacity Self-Driving Car Nanodegree](https://github.com/udacity/self-driving-car)

**Here is the result:**
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/hough.png)

### Final result
![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/lane_line_detection/readme_imgs/final_result.png)


### Possible improvement

1.  Another color space besides "RGB" can be useful to solve strong sun light problem. For example, "HSL" whose "S" channel is more robust even facing sun light problem, can be a good solution.
    
2.  Using plain simple straight lines to detect and fit road lanes may not be a very good solution due to the complexity of roads, vehicles, enviroment, etc. A better algorithm to detect lines and a different equation, polynomial equation for instance, to fit the lanes are needed.
