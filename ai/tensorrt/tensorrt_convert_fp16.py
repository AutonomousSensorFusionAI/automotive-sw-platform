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
import pycuda.driver as cuda 
# from cuda import cuda 
import pycuda.autoinit
import tensorrt as trt

TRT_LOGGER = trt.Logger()

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


def string_to_bool(args):

    if args.dynamic_axes.lower() in ('true'): args.dynamic_axes = True
    else: args.dynamic_axes = False

    return args

def get_transform(img_size):
    options = []
    # options.append(transforms.Resize((img_size[1], img_size[2])))
    options.append(transforms.Resize((28, 28)))
    options.append(transforms.Grayscale(1))
    options.append(transforms.ToTensor())
    #options.append(transforms.Normalize(mean=[0.5,0.5,0.5],std=[0.5,0.5,0.5]))
    transform = transforms.Compose(options)
    return transform
    
'''
def load_image(img_path, size):
    img_raw = io.imread(img_path)
    img_raw = np.rollaxis(img_raw, 2, 0)
    img_resize = resize(img_raw / 255, size, anti_aliasing=True)
    img_resize = img_resize.astype(np.float32)
    return img_resize, img_raw
    '''

def load_image_folder(folder_path, img_size, batch_size):
    transforming = get_transform(img_size)
    dataset = datasets.ImageFolder(folder_path, transform=transforming)
    data_loader = torch.utils.data.DataLoader(dataset,
                                             batch_size=batch_size,
                                             shuffle=True,
                                             num_workers=1)
    data_iter = iter(data_loader)
    torch_images, class_list = next(data_iter)
    print('class:', class_list)
    print('torch images size:', torch_images.size())
    save_image(torch_images[0], 'sample.png')
    
    return torch_images.cpu().numpy()

def build_engine(onnx_model_path, tensorrt_engine_path, engine_precision, dynamic_axes, \
	img_size, batch_size, min_engine_batch_size, opt_engine_batch_size, max_engine_batch_size):
    
    # Builder
    logger = trt.Logger(trt.Logger.ERROR)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    profile = builder.create_optimization_profile()
    config = builder.create_builder_config()
    #config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 3 << 30)
    # Set FP16 
    if engine_precision == 'FP16':
        config.set_flag(trt.BuilderFlag.FP16)
    
    # Onnx parser
    parser = trt.OnnxParser(network, logger)
    if not os.path.exists(onnx_model_path):
        print("Failed finding ONNX file!")
        exit()
    print("Succeeded finding ONNX file!")
    with open(onnx_model_path, "rb") as model:
        if not parser.parse(model.read()):
            print("Failed parsing .onnx file!")
            for error in range(parser.num_errors):
            	print(parser.get_error(error))
            exit()
        print("Succeeded parsing .onnx file!")
    
    # Input
    inputTensor = network.get_input(0) 
    # Dynamic batch (min, opt, max)
    print('inputTensor.name:', inputTensor.name)
    if dynamic_axes:
        profile.set_shape(inputTensor.name, (min_engine_batch_size, img_size[0], img_size[1], img_size[2]), \
        	(opt_engine_batch_size, img_size[0], img_size[1], img_size[2]), \
        	(max_engine_batch_size, img_size[0], img_size[1], img_size[2]))
        print('Set dynamic')
    else:
        profile.set_shape(inputTensor.name, (batch_size, img_size[0], img_size[1], img_size[2]), \
        	(batch_size, img_size[0], img_size[1], img_size[2]), \
        	(batch_size, img_size[0], img_size[1], img_size[2]))
    config.add_optimization_profile(profile)
    #network.unmark_output(network.get_output(0))
    
    # Write engine
    engineString = builder.build_serialized_network(network, config)
    if engineString == None:
        print("Failed building engine!")
        exit()
    print("Succeeded building engine!")
    with open(tensorrt_engine_path, "wb") as f:
        f.write(engineString)
        
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

def main():
    
    batch_size = 1
    
    img_size = [1, 28, 28]
    
    onnx_model_path = 'save_models/onnx_mnist_resnet18.onnx'
    
    tensorrt_engine_path = 'save_models/trt_fp16_mnist_resnet18.engine'
    
    dynamic_axes = True
    
    engine_precision = 'FP16' # 'FP16' or 'FP32'
    # engine_precision = 'FP32' # 'FP16' or 'FP32'
    
    min_engine_batch_size = 1
    
    opt_engine_batch_size = 1
    
    max_engine_batch_size = 8
    
    engine_workspace = 1024
    
    
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
    
    # Build TensorRT engine
    build_engine(onnx_model_path, tensorrt_engine_path, engine_precision, dynamic_axes, \
    	img_size, batch_size, min_engine_batch_size, opt_engine_batch_size, max_engine_batch_size)
    

    
if __name__ == '__main__': 
    main()