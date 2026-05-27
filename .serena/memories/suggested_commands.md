# 실행 명령

## 시뮬레이션 실행
```bash
cd /home/rokey/Rokey_isaac-sim/main_isaac
isaac-python main.py
```
또는 편의 스크립트:
```bash
bash /home/rokey/Rokey_isaac-sim/run_sim.sh
```

## 창고 레이아웃 시각화 (개발용, 별도 터미널)
```bash
python3 /home/rokey/Rokey_isaac-sim/grid/warehouse_layout.py
```

## Serena 메모리 일관성 검사
```bash
# 프로젝트 루트에서
serena memories check
```

## ROS2 토픽 확인
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /iw_hub_02/odom
```
