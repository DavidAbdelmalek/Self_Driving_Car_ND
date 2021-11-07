import os
import glob
from utils import get_dataset
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

if __name__ == "__main__":
    # Dictionary of number of classes in each image.
    directories = ['val/']
    records = []
    base_dir = 'data/'
    
    # Average brightness per image
    brightness = []
    
    # Label mapping
    label_map = {1:'vehicle',
                 2:'pedestrian',
                 4:'cyclist'}
    
    dic_classes = {'vehicle':0,'pedestrian':0,'cyclist':0}
    num_obj_img = []
    
    for target in directories:
        records = get_dataset(base_dir + f'{target}*')
        classes_images = []
        i = 0 
        for batch in records:
            img = batch['image'].numpy()
            brightness.append(np.mean(img))

            classes_images.append(len(batch['groundtruth_classes'].numpy()))
            ground_truth_classes = batch['groundtruth_classes'].numpy()
            for class_ in ground_truth_classes:
                class_ = label_map[class_]
                dic_classes[class_]+=1
            
            num_obj_img.append(len(ground_truth_classes))
          
        
    # Subplots
    f, ax = plt.subplots(1,3, figsize=(12, 4))
    ax[0].hist(brightness, density=True)
    ax[0].set_title("Brightness per image")
    
    
    keys = dic_classes.keys()
    values = dic_classes.values()
    obj_class_dist = np.array(list(Counter(values)))
    obj_class_dist = obj_class_dist/float(obj_class_dist.sum())
    ax[1].bar(keys, obj_class_dist)
    ax[1].set_title("Object class distribution")
    
    ax[2].hist(num_obj_img, density=True)
    ax[2].set_title("Number of objects per image")
    
    plt.savefig("images/EDA.png")
    