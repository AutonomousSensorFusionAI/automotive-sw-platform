import time

import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

import torch.nn as nn
import torch.optim as optim

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


def main():
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
                                              batch_size=16, 
                                              shuffle=True) 

    testset = torchvision.datasets.MNIST(root='./data', 
                                           train=False, 
                                           download=True, 
                                           transform=test_transform)
    
    testloader = torch.utils.data.DataLoader(testset, 
                                             batch_size=16,
                                             shuffle=False)

    
    # 모델 구성
    model = torchvision.models.resnet18(pretrained=False) # 구조만 불러오고
    num_ftrs = model.fc.in_features # fc의 입력 노드 수를 산출 (512개)
    model.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False) # grayscale 적용
    model.fc = nn.Linear(num_ftrs, 10) # fc를 nn.Linear(num_ftrs, 10)로 대체
    model = model.to(device)
    model.load_state_dict(torch.load('./save_models/pytorch_mnist_resnet18.pth'))
    # print(model)

    # 추론
    correct = 0
    total = 0

    
    with torch.no_grad():
        model.eval()
        for data in testloader:
            images, labels = data[0].to(device), data[1].to(device)
            
            total = 0

            for i in range(20):
                start_time = time.time()
                outputs = model(images)
                end_time = time.time() - start_time
        
                print(f"Pytorch Inference Time {i} : {end_time}")

                if i > 9:
                    total += end_time
            
            _, predicted = torch.max(outputs.data, 1)
            # print(f"Real : {labels}")
            # print(f"Pred : {predicted}")
            # total += labels.size(0)
            # correct += (predicted == labels).sum().item()
            break
    
    
    # print('Accuracy of the network on the 10000 test images: %d %%' % (100 * correct / total))
    print(f"Pytorch Average Inference Time : {total/10}")


if __name__=='__main__':
    main()