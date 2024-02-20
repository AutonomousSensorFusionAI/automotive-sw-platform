import time

import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

import torch.nn as nn
import torch.optim as optim

import onnx
import onnxruntime

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


def to_numpy(tensor):
    return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()


def main(onnx_model_path,
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
    trainset = torchvision.datasets.MNIST(root='./data', 
                                            train=True,
                                            download=True, 
                                            transform=transform)

    trainloader = torch.utils.data.DataLoader(trainset, 
                                              batch_size=1, 
                                              shuffle=True) 

    testset = torchvision.datasets.MNIST(root='./data', 
                                           train=False, 
                                           download=True, 
                                           transform=test_transform)
    
    testloader = torch.utils.data.DataLoader(testset, 
                                             batch_size=1,
                                             shuffle=False)
    

    data_iter = iter(testloader)
    torch_images, class_list = next(data_iter)
    torch_images = torch_images.cpu().numpy()
    
    # ONNX inference
    onnx_model = onnx.load(onnx_model_path)
    sess = onnxruntime.InferenceSession(onnx_model_path, providers=['CUDAExecutionProvider'])

    input_all = [node.name for node in onnx_model.graph.input]
    input_initializer = [
        node.name for node in onnx_model.graph.initializer
    ]
    net_feed_input = list(set(input_all) - set(input_initializer))
    assert len(net_feed_input) == 1

    sess_input = sess.get_inputs()[0].name
    sess_output = sess.get_outputs()[0].name

    total = 0

    for i in range(20):
        onnx_start_time = time.time()
        onnx_result = sess.run([sess_output], {sess_input: torch_images})[0]
        onnx_end_time = time.time() - onnx_start_time

        print(f"ONNX Inference Time {i} : {onnx_end_time}")

        if i > 9:
            total += onnx_end_time

    print(f"ONNX Average Inference Time : {total/10}")


    
if __name__ == '__main__': 
    main("save_models/onnx_mnist_resnet18.onnx",
         1,
         [1,28,28])