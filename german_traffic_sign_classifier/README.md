
  
  

# **German Traffic Sign Recognition**

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

  

Overview

---

In this project, I will use what I've learned about deep neural networks and convolutional neural networks to classify traffic signs. I will train and validate a model so it can classify traffic sign images using the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). After the model is trained, I will then try out that model on images of German traffic signs.

### Data Set Summary

  

I used the pandas library to calculate summary statistics of the traffic

signs data set:

  

* The size of training set is 34799

* The size of the validation set is 4410

* The size of test set is 12630

* The shape of a traffic sign image is (32, 32, 3)

* The number of unique classes/labels in the data set is 43

  

### Data Exploration

  

We begin with a simple exploratory analysis of the training data set. There are a little less than 35k images for training dataset.

  

Our traffic signs belong to 43 different classes and each images below shows the distribution of classes over training, validation, and testing dataset, accordingly :

  

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/class_distribution_accross_training.png)

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/class_distribution_accross_validation.png)

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/class_distribution_accross_testing.png)

  

The next step was to plot random image for the classes that were least represented, this would give us an idea of what kind of problems to expect down the road.

  

The 15 classes that have the least amount of training images are as follows:

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/lowest_15_class_distribution.png)

  

### Preprocessing

  
  

As we can see that the quality of the images are very different one from another. In addition, most of classes have a huge varaint in their frequency. This discrepancy will certainly impact our model if we are not careful with the pre-processing of these images.

  

On [1] the winners of the GTSRB challenge addressed this problem by normalizing the contrast during preprocessing.

On [2] the authors introduced more samples to the training dataset by applying translations, scaling and rotations during preprocessing and proposed data augmentation to generate enough new instances to ensure each category has at least 200 instances.

  

We will follow their award winning approach to preprocessing before moving on to our model architecture.

So, our preprocessing approach consisted of:

1. Converting to grayscale - This worked well for [2] as described in their traffic sign classification article. It also helps to reduce training time, which was nice when a GPU wasn't available.

  

2. Normalizing the data to the range (-1,1) - This was done using the line of code X_train_normalized = (X_train - 128)/128. The resulting dataset mean wasn't exactly zero.

  

You can see the results of preprocessing on the set of images below:

  

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/pre_processed.png)

  

### Data Augmentation

  

For this part I created a function that will randomly rotate, scale and translate our images with the same parameters as implemented in [2]

  

[2] also suggested that their model could perform better if they added some Real-world deformations, such as motion blur and shape deformations. So I decided to add them to our pipeline as well.

  

- Rotation between -15 and 15 degrees

- Scaling between 0.9 and 1.1

- Translation between -2 and 2 pixels

  

I have also added a shear transformation and motion blur as suggested by the authors at the summary of their work. The parameters for this new transformations are:

  

- Motion blur kernel size of 3 pixels

  

The images below show a comparison between the original image, a rotated image, translated, blurred, and all transformation together.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/all_transformation.png)

  

The result of data augmentation is reflected directly on the distribiution of classes in training dataset. Thus, each class has occurred at least 800 times in training dataset and that will result in a more robust and general model.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/training_class_distribution_after_augmentation.png)

  

### Model Architecture


My final model consisted of the following layers:

| Layer         		|     Description	        					| outputs
|:---------------------:|:---------------------------------------------:| -:| 
| Input         		| 32x32x3 RGB image   							|    
| Convolution 5x5     	| 1x1 stride, same padding,, depth of 32 	    | 28x28x32
| RELU					|	Non-linear activation function 								 |
| Max pooling	      	| 2x2 kernel, stride of 2, valid padding 		 | 14x14x32
| Convolution 5x5     	| 1x1 stride, same padding,, depth of 64 	     | 10x10x64
| RELU					|	Non-linear activation function 					 |
| Max pooling	      	| 2x2 kernel, stride of 2, valid padding 		 | 5x5x64
| Flatten	            | converting the data into a 1-dimensional array |    
| Dropout	            | randomly deactivate set of neurons to 0 |     									
| Fully connected		| 1600 hidden neurons        					 |1600
| Dropout	            | randomly deactivate set of neurons to 0 |   
| Fully connected		| 120 hidden neurons        					 |120 
| Dropout	            | randomly deactivate set of neurons to 0 |   
| Fully connected		| 84 hidden neurons        					 |84
| Dropout	            |randomly deactivate set of neurons to 0  |    
|Softmax				| output layer calculating final logits corresponding to target classes 		     | 43

**Hyperparameters** :

-  **Dropout**: 0.5

-  **Learning rate**: 1e-3

-  **Epochs**: 40

-  **Batch size**: 128

-  **Optimizer**: Adam Optimizer

  

I believe this is a good selection of most of the important techniques that have been adopted by the industry.

  

I have also decided to keep the structure of the NN on a conventional way, I believe this will have an impact on our prediction capabilities as the best results I've seen out there are following the same approach as in [2], but by keeping our model with a similar form as the one from LeNet we can more accurately compare the performance between these two models.

  

I intentionally left out Inception and Residual Neural Net modules as the intent of this study is to compare "similarly" constructed networks, although they would certainly perform better than our proposed model.

  

In the references section of the notebook you will find links to all of the sources that inspired this study.

  

#### Training & Validation Analysis

The refereed model achieved a **97.02%**  *training accuracy* while **98.25%**  *validation accuracy*. Below, the image shows how the accuracy has been developed and increased through 40 epochs

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/training_vs_validation_loss.png)

  

#### Testing Analysis

Finally we get to see how our model is doing in the test set, but surprisingly the accuracy dropped, quite a lot, to 97%.

  

It's usually the case that your model will perform better on the validation set because your model was optimized to give you the best result possible with that set of data in mind.

  

Getting your model to generalize well is one of the most difficult parts of tuning it, but further exploring what went wrong will give us important information as to how to improve our prediction power.

  

The model misclassified 419 images. Here is the label distribution of those misclassified images

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/incorrect_classification.png)

It seems that most of our errors are concentrated in less than 10 classes.

  
  

### Test a Model on New Images

  

#### Choose 10 German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

  

The image below shows the 10 new images along with their preprocessed versions.![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/test_preprocess.png)

  

As expected we didn't do too well on this limited sample, only 80% accuracy, compared with a 97% on the test set. But that's fine, they were chosen to be difficult for the model to classify. The idea of this sample was to give us more information about where are are getting things wrong.

  

Looking at the top 5 probabilities will show us how certain, or uncertain, the model was on these two misclassification cases.

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_19_Wrong.png)

  

Int the first case, we can observe that the model had no clue what that sign was and was in doubt amongst several speed limit signs, that have the same shape.

  

![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_33_Wrong.png)

 
However, we can see that in the second case our model was actually in doubt between the correct class and one alternative


Below, you can find examples of how our model correctly predicts testing images:

[![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_11_Correct.png)](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_11_Correct.png)
  
[![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_18_Correct.png)](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_18_Correct.png)[![](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_1_Correct.png)](https://raw.githubusercontent.com/DavidAbdelmalek/Self_Driving_Car_ND/main/german_traffic_sign_classifier/images/display/predict_img_1_Correct.png)

Finally, The model was able to correctly guess 8 of the 10 traffic signs and here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Right-of-way at the next intersection      		| Right-of-way at the next intersection   									| 
| Dangerous curve to the left     			| Bumpy road 										|
| End of all speed					| End of all speed											|
| General caution	      		| General caution					 				|
| 30km/h		|30km/h      							|
| 60km/h		|60km/h      							|
| Traffic signals		|Traffic signals      							|
| Road work		|Road work      							|
| Turn right ahead		|Ahead only      							|



## References

[1] [Multi-Column Deep Neural Network for Traffic Sign](http://people.idsia.ch/~juergen/nn2012traffic.pdf)

  

[2]  [Traffic Sign Recognition with Multi-Scale Convolutional Networks](http://yann.lecun.com/exdb/publis/pdf/sermanet-ijcnn-11.pdf)
