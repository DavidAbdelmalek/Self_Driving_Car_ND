from utils import get_dataset
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter
import seaborn as sns

def plot_target(dist, plot_type, title):
    """
        Plot target EDA & save it.
        Arguments:
		    - dist [dict]: Counter dictionary for each analysis
            - plot_type [str]: Plot type. It is either 'bar' or 'hist'
            - title [str]: Plot title and name to be saved with.
     """
    
    f, ax = plt.subplots(1, figsize=(12, 4))
    if plot_type == 'bar':
        sns.barplot(list(dist.keys()), list(dist.values()), alpha=0.8)
    else:
        sns.histplot(data=dist, bins=50,color="red")
    
    ax.set_title(f"Total {title}")
    plt.savefig(f"images/{title}.png")
    
if __name__ == "__main__":
    
    # Average brightness per image
    brightness = []
    
    # Label mapping
    label_map = {1:'vehicle',
                 2:'pedestrian',
                 4:'cyclist'}
    
    dic_classes = {'vehicle':0,'pedestrian':0,'cyclist':0}
    scene_type = {'day':0,'night':0}
    
    num_obj_img = []
    
    dataset = get_dataset('/home/workspace/data/train/*.tfrecord')
    
    selected_dataset = dataset.shuffle(100,seed=12)
    selected_dataset = selected_dataset.take(20000)
    for batch in selected_dataset:
        classes_images = []
        
        img = batch['image'].numpy()
        brightness.append(np.mean(img))

        classes_images.append(len(batch['groundtruth_classes'].numpy()))
        ground_truth_classes = batch['groundtruth_classes'].numpy()
        for class_ in ground_truth_classes:
            class_ = label_map[class_]
            dic_classes[class_]+=1
            
        num_obj_img.append(len(ground_truth_classes))
        
        # check scenes 
        th = np.sum(np.sum(img))/(640*640*3)
        
        if th > 50:
            scene_type['day']+=1
        else:
            scene_type['night']+=1
          
    # Plotting
    plot_target(brightness  ,'hist' ,'brightness')
    plot_target(num_obj_img ,'hist' ,'object_per_image')
    plot_target(dic_classes ,'bar'  ,'class_distribution')
    plot_target(scene_type ,'bar'  ,'scences')