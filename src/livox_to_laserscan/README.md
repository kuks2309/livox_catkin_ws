# Livox to LaserScan

Livox MI360 LiDAR를 2D LaserScan으로 변환하는 패키지입니다.

## 설치

### 의존성 설치
```bash
sudo apt-get install ros-noetic-pointcloud-to-laserscan
```

### 패키지 빌드
```bash
cd ~/2025_one_fifth_catkin_ws
catkin_make
source devel/setup.bash
```

## 사용법

### 기본 사용
```bash
roslaunch livox_to_laserscan livox_to_scan.launch
```

### 파라미터 조정
```bash
roslaunch livox_to_laserscan livox_to_scan.launch \
    cloud_in:=/your_pointcloud_topic \
    scan_out:=/your_scan_topic \
    min_height:=-0.2 \
    max_height:=1.5
```

## 토픽

### 입력
- `/livox/lidar` (sensor_msgs/PointCloud2): Livox 포인트 클라우드

### 출력  
- `/scan` (sensor_msgs/LaserScan): 2D 레이저 스캔

## 파라미터

- `min_height`: 최소 높이 (기본값: -0.3m)
- `max_height`: 최대 높이 (기본값: 1.8m)  
- `range_min`: 최소 거리 (기본값: 0.5m)
- `range_max`: 최대 거리 (기본값: 80.0m)
- `target_frame`: 타겟 프레임 (기본값: base_link)

## move_base와 함께 사용

```bash
# 1. Livox를 LaserScan으로 변환
roslaunch livox_to_laserscan livox_to_scan.launch

# 2. move_base 실행
roslaunch move_base move_base.launch
```

## 주의사항

1. TF 설정에서 `livox_frame`과 `base_link` 간 변환이 정확해야 합니다
2. 높이 필터링 범위를 로봇 환경에 맞게 조정하세요
3. 실시간 성능을 위해 `concurrency_level`을 조정할 수 있습니다
