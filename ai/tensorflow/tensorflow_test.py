import tensorflow as tf
import tensorflow_datasets as tfds
# from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.layers import Conv2D, Dense, BatchNormalization, Activation, MaxPool2D, GlobalAveragePooling2D, Add
from tensorflow.keras import Model

from tensorflow_train import *

import time


def main():
    # load dataset
    dataset, info = tfds.load('mnist', as_supervised = True, with_info = True)
    dataset_test, dataset_train = dataset['test'], dataset['train']
    
    batch_size = 1

    dataset_train = dataset_train.map(convert_types).shuffle(10000).batch(batch_size)
    dataset_test = dataset_test.map(convert_types).batch(batch_size)
    
    
    # define model 
    model = ResNet18((28, 28, 1), 10)
    model.build(input_shape = (None, 28, 28, 1))
    # model.summary()
    
    model.load_weights('save_models/tensorflow/tensorflow_mnist_resnet18')

    total = 0

    for test_image, test_label in dataset_test:
        
        for i in range(20):
            start_time = time.time()
            outputs = model(test_image)
            end_time = time.time() - start_time
            
            print(f"Tensorflow Inference Time {i} : {end_time}")

            if i > 9:
                total += end_time
        
        break
    
    print(f"Tensorflow Average Inference Time : {total/10}")

if __name__ == "__main__":
    main()