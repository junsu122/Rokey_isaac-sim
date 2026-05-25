#!/bin/bash
# Isaac Sim ROS2 bridge가 정상 동작하도록 환경 설정 후 main.py 실행
#
# 문제: /opt/ros/humble/setup.bash 를 source 하면 PYTHONPATH에
#       Python 3.10 rclpy가 먼저 등록됨. Isaac Sim (Python 3.11)이
#       rclpy를 import할 때 3.10용 C extension을 발견하고 실패함.
#
# 해결: Isaac Sim 내장 humble/rclpy 경로를 PYTHONPATH 앞에 삽입하면
#       3.11용 _rclpy_pybind11.cpython-311-x86_64-linux-gnu.so 가
#       우선 로드됨.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_BUILD="/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
BRIDGE_EXT="${ISAAC_BUILD}/exts/isaacsim.ros2.bridge"

# 내장 rclpy 경로를 PYTHONPATH 맨 앞에 추가 (시스템 rclpy 3.10 보다 우선)
export PYTHONPATH="${BRIDGE_EXT}/humble/rclpy:${PYTHONPATH}"

# 내장 ROS2 shared lib 경로 추가
export LD_LIBRARY_PATH="${BRIDGE_EXT}/humble/lib:${LD_LIBRARY_PATH}"

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[run_sim] 내장 rclpy (Python 3.11) 경로 설정 완료"
echo "[run_sim] Isaac Sim 실행: main_isaac/main.py"

exec "${ISAAC_BUILD}/python.sh" "${SCRIPT_DIR}/main_isaac/main.py" "$@"
