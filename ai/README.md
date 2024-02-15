# 인공지능 모델 6종

## PyTorch
### Train
```bash
python pytorch/pytorch_train.py
```
### Test
```bash
python pytorch/pytorch_test.py
```

## Tensorflow
### Train
```bash
python tensorflow/tensorflow_train.py
```
### Test
```bash
python tensorflow/tensorflow_test.py
```

## Keras
### Keras 모델 다운로드
variables 폴더 내 파일의 크기가 커서 구글 드라이브에서 다운로드
```bash
mkdir save_models/keras/variables

wget --load-cookies ~/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies ~/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=14Y3y7wrVkHYQns9w_NEwryaoalShRoSv' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=14Y3y7wrVkHYQns9w_NEwryaoalShRoSv" -O save_models/keras/variables/variables.index && rm -rf ~/cookies.txt

wget --load-cookies ~/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies ~/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1plkx_ic5GVy3xYz8r7LAbtNfoxkbP_ot' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1plkx_ic5GVy3xYz8r7LAbtNfoxkbP_ot" -O save_models/keras/variables/variables.data-00000-of-00001 && rm -rf ~/cookies.txt
```
### Train
```bash
python keras/keras_train.py
```
### Test
```bash
python keras/keras_test.py
```

## ONNX
### Convert
```bash
python onnx/onnx_convert.py
```
### Test
```bash
python onnx/onnx_test.py
```

## TensorRT
### FP16
#### Convert
```bash
python tensorrt/tensorrt_convert_fp16.py
```
#### Test
```bash
python tensorrt/tensorrt_test_fp16.py
```
### FP32
#### Convert
```bash
python tensorrt/tensorrt_convert_fp32.py
```
#### Test
```bash
python tensorrt/tensorrt_test_fp32.py
```

## PaddlePaddle
### Train
```bash
python paddle/paddle_train.py
```
### Test
```bash
python paddle/paddle_test.py
```