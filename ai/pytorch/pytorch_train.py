import os
import tqdm

import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

import torch.nn as nn
import torch.optim as optim

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


def main():
    """
    학습 환경
    Ubuntu 22.04
    NVIDIA GeForce 4090 GPU 사용
    위 환경에서 MNIST 데이터셋을 ResNet18 모델에 훈련
    훈련된 모델은 pytorch 모델로 이를 다른 포맷의 모델로 변환하여 Jetson 보드에서 테스트를 진행
    """
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
    print(model)

    # 학습
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-4, weight_decay=1e-2)

    for epoch in tqdm.tqdm(range(20)):

        running_loss = 0.0
        for data in trainloader:
            
            inputs, labels = data[0].to(device), data[1].to(device)
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()

        cost = running_loss / len(trainloader)        
        print('[%d] loss: %.3f' %(epoch + 1, cost))  

    os.makedirs("save_models", exist_ok=True)
    torch.save(model.state_dict(), './save_models/pytorch_mnist_resnet18.pth')      

    # 학습된 모델 상태(stae_dict()) .pth 에 저장 ! 
    print('Finished Training')


if __name__=='__main__':
    main()