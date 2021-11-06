import os
import glob
from utils import get_dataset
import numpy as np

if __name__ == "__main__":
    # Dictionary of number of classes in each image.
    directories = ['val/']
    records = []
    base_dir = 'data/'
    
    brightness = []
    dic_classes = {}
    
    for target in directories:
        records = get_dataset(base_dir + f'{target}*')
        classes_images = []
        for batch in records:
            img = batch['image'].numpy()
            brightness.append(np.mean(img))

            classes_images.append(len(batch['groundtruth_classes'].numpy()))
            for class_ in batch['groundtruth_classes'].numpy():
                if not class_ in dic_classes:
                    dic_classes[class_]=1
                else:
                    dic_classes[class_]+=1
               
            
    print(dic_classes)
    print(classes_image)
    #recordss)
    #classes_images = []
    #i = 0 

    #    print(batch)
    #plt.hist(classes_images, bins=30)  # density=False would make counts
    #plt.ylabel('Probability')
    #plt.xlabel('Data');