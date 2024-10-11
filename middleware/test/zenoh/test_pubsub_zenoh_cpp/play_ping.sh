#!/bin/bash

# 입력 파일들의 배열
test=100
qos_sets=(
    # Priority CongestionControl Reliability
    "0 -1 -1"
    "1 -1 -1"
    "2 -1 -1"
    "3 -1 -1"
    "4 -1 -1"
    "5 -1 -1"
    "6 -1 -1"
    "-1 0 -1"
    "-1 1 -1"
    "-1 -1 0"
    "-1 -1 1"
)
pics=(0 1 2 3)

mkdir -p results

# 현재 날짜와 시간을 파일 이름에 포함
timestamp=$(date +"%Y%m%d_%H%M%S")
output_file="results/image_ping_${timestamp}.txt"

{
    # QoS 세트에 대한 반복
    for qos_set in "${qos_sets[@]}"
    do
        read -r priority congestion reliability <<< "$qos_set"

        # 각 사진에 대해 반복
        for pic in "${pics[@]}"
        do
            echo "QoS: Priority=$priority, Congestion=$congestion, Reliability=$reliability"
            echo "Picture: $pic"
            echo "Test Count: $test"
            echo "=========================="

            for i in {1..3}
            do
                sleep 5
                echo "Run: $i"
                echo "------------------------"
                ./build/image_ping -test $test -pic $pic -priority $priority -congestion $congestion -reliability $reliability
                echo "------------------------"
                sleep 60
            done
            echo "============================================="
        done
    done
}| tee "$output_file"