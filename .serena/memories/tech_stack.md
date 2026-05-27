# 기술 스택

## 런타임
- NVIDIA Isaac Sim 5.1.0 (standalone 스크립트 모드)
- Python: Isaac Sim 번들 Python (isaac-python alias)
- ROS2 Humble (통신 레이어)
- ROS_DOMAIN_ID=140

## 실행 환경
- Isaac Sim Python: `~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh`
  alias: `isaac-python`
- Isaac Sim GUI: `~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/isaac-sim.sh`
  alias: `isaac`
- ROS2 source: `source /opt/ros/humble/setup.bash` (bashrc에 자동 적용)

## 주요 의존성
- omni.usd, pxr (USD 씬 조작)
- omni.isaac.core (World, 물리 시뮬레이션)
- omni.isaac.sensor (Camera)
- rclpy (ROS2 Python 클라이언트)
- cv2, numpy (미니맵, ArUco 감지)

## 시뮬레이션 타이밍
- PHYSICS_DT = 1/500 Hz
- RENDERING_DT = 1/50 Hz (= 20ms per render step)
