#!/bin/bash

# 입력 파일들의 배열
inputs=("--ros-args -p test:=10 -p pic:=0" "--ros-args -p test:=10 -p pic:=1" "--ros-args -p test:=10 -p pic:=2" "--ros-args -p test:=10 -p pic:=3")

mkdir -p results

# 각 입력 파일에 대해 프로그램 실행
for input in "${inputs[@]}"
do
    # 현재 날짜와 시간을 파일 이름에 포함
    timestamp=$(date +"%Y%m%d_%H%M%S")
    output_file="results/image_pong_${timestamp}_${input// /_}.txt"
    {
        echo "Input: $input"
        echo "=========================="

    for i in {1..3}
    do
        echo "Run: $i"
        echo "------------------------"
        ros2 run test_pubsub image_pong $input --log-level info > >(tee -a "$output_file") 2>&1 | sed 's/\x1b\[[0-9;]*[mGKH]//g'
        echo "------------------------"
        sleep 30
    done
    } | tee "$output_file"
    
    echo "Output saved to $output_file"
    echo "============================================="
done
