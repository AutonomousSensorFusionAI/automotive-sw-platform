import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

import torch.nn as nn
import torch.optim as optim

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

from torch import onnx

def convert_onnx(model):
    model.eval()
    
    dummy_input = torch.randn(16, 1, 28, 28, requires_grad=True).to(device)
    
    # torch.onnx.export(
    #     model.cuda(),
    #     dummy_input,
    #     "save_models/onnx_mnist_resnet18.onnx",
    #     export_params=True,
    #     # opset_verison=10,
    #     do_constant_folding=True,
    #     input_names=['modelInput'],
    #     output_names=['modelOutput'],
    #     dynamic_axes={'modelInput' : {0 : 'batch_size'},
    #                   'modelOutput' : {0 : 'batch_size'}}
    #     )
    
    
    output_onnx = "save_models/onnx_mnist_resnet18.onnx"
    input_names = ["input"]
    output_names = ["output"]
    
    dynamic_axes = {'input' : {0 : 'batch_size'},
                    'output' : {0 : 'batch_size'}}

    # inputs = torch.randn(2, 3, 256, 256).to(device)

    torch_out = torch.onnx.export(
        model.cuda(), 
        dummy_input, 
        output_onnx, 
        export_params=True, 
        do_constant_folding=True,
        verbose=False,
        input_names=input_names, 
        output_names=output_names, 
        opset_version=11, 
        dynamic_axes = dynamic_axes)
    
    print("Model has been converted to ONNX")
    
    
if __name__ == "__main__":
    model = torchvision.models.resnet18(pretrained=False) # 구조만 불러오고
    num_ftrs = model.fc.in_features # fc의 입력 노드 수를 산출 (512개)
    model.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False) # grayscale 적용
    model.fc = nn.Linear(num_ftrs, 10) # fc를 nn.Linear(num_ftrs, 10)로 대체
    model = model.to(device)
    model.load_state_dict(torch.load('./save_models/pytorch_mnist_resnet18.pth'))
    print(model)
    
    convert_onnx(model)