import argparse
import os
import numpy as np
from skimage import io
from skimage.transform import resize
from collections import OrderedDict
from PIL import Image
import cv2
import time

# Torch
import torch
from torch import nn
import torchvision.models as models
from torch.utils.model_zoo import load_url as load_state_dict_from_url
import torch.optim as optim
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from torchvision.utils import save_image

# ONNX: pip install onnx, onnxruntime
try:
    import onnx
    import onnxruntime as rt
except ImportError as e:
    raise ImportError(f'Please install onnx and onnxruntime first. {e}')

# CUDA & TensorRT
# import pycuda.driver as cuda 
from cuda import cuda 
import pycuda.autoinit
import tensorrt as trt

TRT_LOGGER = trt.Logger()

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


def trt_inference(engine, context, data):  
    
    nInput = np.sum([engine.binding_is_input(i) for i in range(engine.num_bindings)])
    nOutput = engine.num_bindings - nInput
    print('nInput:', nInput)
    print('nOutput:', nOutput)
    
    for i in range(nInput):
        print("Bind[%2d]:i[%2d]->" % (i, i), engine.get_binding_dtype(i), engine.get_binding_shape(i), context.get_binding_shape(i), engine.get_binding_name(i))
    for i in range(nInput,nInput+nOutput):
        print("Bind[%2d]:o[%2d]->" % (i, i - nInput), engine.get_binding_dtype(i), engine.get_binding_shape(i), context.get_binding_shape(i), engine.get_binding_name(i))
        
    bufferH = []
    bufferH.append(np.ascontiguousarray(data.reshape(-1)))
    
    for i in range(nInput, nInput + nOutput):
        bufferH.append(np.empty(context.get_binding_shape(i), dtype=trt.nptype(engine.get_binding_dtype(i))))
    
    bufferD = []
    for i in range(nInput + nOutput):
        bufferD.append(cuda.cuMemAlloc(bufferH[i].nbytes)[1])

    for i in range(nInput):
        cuda.cuMemcpyHtoD(bufferD[i], bufferH[i].ctypes.data, bufferH[i].nbytes)
    
    context.execute_v2(bufferD)

    for i in range(nInput, nInput + nOutput):
        cuda.cuMemcpyDtoH(bufferH[i].ctypes.data, bufferD[i], bufferH[i].nbytes)
        
    for b in bufferD:
        cuda.cuMemFree(b)  
    
    return bufferH



def main(tensorrt_engine_path,
         batch_size,
         img_size):
    # data transform 구성
    transform = transforms.Compose([
        # transforms.RandomCrop(32, padding=4),
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))])

    test_transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))])

    # MNIST 데이터셋 로드
    trainset = datasets.MNIST(root='./data', 
                                            train=True,
                                            download=True, 
                                            transform=transform)

    trainloader = torch.utils.data.DataLoader(trainset, 
                                              batch_size=1, 
                                              shuffle=True) 

    testset = datasets.MNIST(root='./data', 
                                           train=False, 
                                           download=True, 
                                           transform=test_transform)
    
    testloader = torch.utils.data.DataLoader(testset, 
                                             batch_size=1,
                                             shuffle=False)
    

    data_iter = iter(testloader)
    torch_images, class_list = next(data_iter)
    torch_images = torch_images.cpu().numpy()
    torch_images = torch_images.astype(np.float32)
    
    # Read the engine from the file and deserialize
    with open(tensorrt_engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime: 
        engine = runtime.deserialize_cuda_engine(f.read())    
    
    context = engine.create_execution_context()

    # TensorRT inference
    context.set_binding_shape(0, (batch_size, img_size[0], img_size[1], img_size[2]))
    
    total = 0

    for i in range(20):
        start_time = time.time()
        trt_outputs = trt_inference(engine, context, torch_images[0])
        trt_outputs = np.array(trt_outputs[1]).reshape(batch_size, -1)
        end_time = time.time() - start_time

        print(f"TensorRT Inference Time {i} : {end_time}")

        if i > 9:
            total += end_time

    print(f"TensorRT Average Inference Time : {total/10}")


    
if __name__ == '__main__': 
    main("save_models/trt_fp8_mnist_resnet18.engine",
         1,
         [1,28,28])
