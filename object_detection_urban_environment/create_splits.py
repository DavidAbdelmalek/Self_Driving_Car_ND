import argparse
import glob
import os
import random
import shutil

import numpy as np

from utils import get_module_logger,get_dataset


def split(data_dir,train_size,test_size,val_size):
    """
    Create three splits from the processed records. The files should be moved to new folders in the 
    same directory. This folder should be named train, val and test.

    args:
        - data_dir [str]: data directory, /mnt/data
    """
    # set seed for random split later
    random.seed(2021)
    
    # Get dataset
    ds_size = len(glob.glob(data_dir + '/preprocessed_data/*.tfrecord'))
    
    # create the directry
    for _dir in ["train", "val", "test"]:
        os.makedirs(data_dir+_dir, exist_ok=True)
        
    logger.info(f'Split dataset into {train_size}% training records, {test_size}% training records, and {val_size}% training records ')
    
   
    train_size = int(ds_size * (train_size/100))
    test_size = int(ds_size * (test_size/100))
    val_size = int(ds_size * (val_size/100))
    
    dic = {'train':train_size,
           'test':test_size,
           'val':val_size}
    

    for split_type,size in dic.items() :
        dataset = os.listdir(data_dir + '/preprocessed_data/')
        
        logger.info(f'{size} {split_type} records')
        logger.info(f'Moving {split_type} split to ./data/{split_type}/ ..')
        
        batch_selected = random.sample(dataset, size)
        for batch in batch_selected:
            shutil.move(data_dir + '/preprocessed_data/'+batch,data_dir + f'/{split_type}/')
        logger.info('-'*80)

    return

    
if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description='Split data into training / validation / testing')
    parser.add_argument('--data_dir', required=True,
                        help='data directory')
    args = parser.parse_args()

    logger = get_module_logger(__name__)
    logger.info('Creating splits...')
    split(data_dir = args.data_dir,train_size = 80,test_size = 10,val_size = 10)
 