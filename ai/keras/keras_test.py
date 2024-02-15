import os
import time
import numpy as np
from keras import layers
from keras.datasets import mnist
from keras.utils import to_categorical

from keras.models import Model, load_model

from keras_train import *

os.environ['CUDA_VISIBLE_DEVICES'] = '0'



def main():
    
    (x_train, y_train), (x_test, y_test) = mnist.load_data()
    y_train_onehot = to_categorical(y_train, num_classes=None, dtype='float32')
    print("Train Set Size = {} images".format(y_train.shape[0]))
    print("Test Set Size = {} images".format(y_test.shape[0]))
    
    # model = resnet18(y_train_onehot)
    model = load_model("save_models/keras")

    total = 0

    for test_images in x_test:
        for i in range(20):
            start_time = time.time()
            outputs = model.predict(test_images.reshape((-1,28,28,1)), verbose=0)
            end_time = time.time() - start_time
            
            print(f"Keras Inference Time {i} : {end_time}")

            if i > 9:
                total += end_time
            
        break
    
    print(f"Keras Average Inference Time : {total/10}")
    
if __name__ == "__main__":
    main()