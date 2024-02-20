import itertools
import time
# import cv2
import numpy as np
import paddle
import paddle.nn as nn
import paddle.vision.transforms as T
from paddle.static import InputSpec
from paddle.vision.datasets import MNIST
import paddle.fluid as fluid
# from paddle.vision.models import resnet18
from paddle.utils.download import get_weights_path_from_url
from paddle.inference import Config
from paddle.inference import create_predictor
from paddle.inference import PrecisionType

from paddle_train import *

def init_predictor(
    model_file,
    params_file,
    run_mode,
    use_dynamic_shape=0
    ):
    config = Config(model_file, params_file)

    config.enable_memory_optim()

    gpu_precision = PrecisionType.Float32
    if run_mode == "gpu_fp16":
        gpu_precision = PrecisionType.Half

    config.enable_use_gpu(1000, 0, gpu_precision)

    if run_mode == "trt_fp32":
        config.enable_tensorrt_engine(
            workspace_size=1 << 30,
            max_batch_size=1,
            min_subgraph_size=5,
            precision_mode=PrecisionType.Float32,
            use_static=False,
            use_calib_mode=False,
        )
    elif run_mode == "trt_fp16":
        config.enable_tensorrt_engine(
            workspace_size=1 << 30,
            max_batch_size=1,
            min_subgraph_size=5,
            precision_mode=PrecisionType.Half,
            use_static=False,
            use_calib_mode=False,
        )
    elif run_mode == "trt_int8":
        config.enable_tensorrt_engine(
            workspace_size=1 << 30,
            max_batch_size=1,
            min_subgraph_size=5,
            precision_mode=PrecisionType.Int8,
            use_static=False,
            use_calib_mode=True,
        )
    if use_dynamic_shape:
        names = ["inputs"]
        min_input_shape = [[1, 3, 112, 112]]
        max_input_shape = [[1, 3, 448, 448]]
        opt_input_shape = [[1, 3, 224, 224]]

        config.set_trt_dynamic_shape_info(
            {names[0]: min_input_shape[0]},
            {names[0]: max_input_shape[0]},
            {names[0]: opt_input_shape[0]},
        )

    predictor = create_predictor(config)
    return predictor


def run(predictor, img):
    # copy img data to input tensor
    input_names = predictor.get_input_names()
    for i, name in enumerate(input_names):
        input_tensor = predictor.get_input_handle(name)
        input_tensor.reshape(img[i].shape)
        input_tensor.copy_from_cpu(img[i])

    # do the inference
    predictor.run()

    results = []
    # get out data from output tensor
    output_names = predictor.get_output_names()
    for i, name in enumerate(output_names):
        output_tensor = predictor.get_output_handle(name)
        output_data = output_tensor.copy_to_cpu()
        results.append(output_data)

    return results


def resize_short(img, target_size):
    """ resize_short """
    percent = float(target_size) / min(img.shape[0], img.shape[1])
    resized_width = int(round(img.shape[1] * percent))
    resized_height = int(round(img.shape[0] * percent))
    resized = cv2.resize(img, (resized_width, resized_height))
    return resized


def crop_image(img, target_size, center):
    """ crop_image """
    height, width = img.shape[:2]
    size = target_size
    if center == True:
        w_start = (width - size) / 2
        h_start = (height - size) / 2
    else:
        w_start = np.random.randint(0, width - size + 1)
        h_start = np.random.randint(0, height - size + 1)
    w_end = w_start + size
    h_end = h_start + size
    img = img[int(h_start):int(h_end), int(w_start):int(w_end), :]
    return img


def preprocess(img):
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]
    img = resize_short(img, 28)
    img = crop_image(img, 28, True)
    # bgr-> rgb && hwc->chw
    img = img[:, :, ::-1].astype('float32').transpose((2, 0, 1)) / 255
    img_mean = np.array(mean).reshape((3, 1, 1))
    img_std = np.array(std).reshape((3, 1, 1))
    img -= img_mean
    img /= img_std
    return img[np.newaxis, :]


def main():
    
    paddle.enable_static()

    transform = T.Compose([
            T.Grayscale(),
            T.Transpose(),
            T.Normalize([127.5], [127.5]),
        ])

    
    mnist_test = MNIST(
        mode="test",
        transform=transform,  # apply transform to every image
        # backend="cv2",  # use OpenCV as image transform backend
    )
    
    
    model_file = "save_models/paddle/paddle_mnist_resnet18.pdmodel"
    params_file = "save_models/paddle/paddle_mnist_resnet18.pdiparams"
    
    
    predictor = init_predictor(
        model_file,
        params_file,
        'gpu_fp16'
    )
    
    
    for img, label in itertools.islice(iter(mnist_test), 5):  # only show first 5 images
        # img = preprocess(img)
        print(type(img))
        print(img.shape)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        total = 0

        for i in range(20):
            start_time = time.time()
            # results = exe.run(inference_program,
            #     feed={feed_target_names[0]: img},
            #     fetch_list=fetch_targets)

            outputs = run(predictor, [img.reshape((1, 1, 28, 28))])
            end_time = time.time() - start_time
            
            print(f"PaddlePaddle Inference Time {i} : {end_time}")

            if i > 9:
                total += end_time
        
        break
    
    print(f"PaddlePaddle Average Inference Time : {total/10}")
    
    
if __name__ == "__main__":
    main()