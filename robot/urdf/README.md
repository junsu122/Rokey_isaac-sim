# 두산 M0609 URDF 설치 방법

## 1. ROS2 패키지에서 URDF 추출

```bash
# ROS2 패키지 클론
git clone https://github.com/doosan-robotics/doosan-robot2.git

# m0609 URDF 파일 위치
doosan-robot2/dsr_description2/urdf/m0609.urdf.xacro

# xacro → urdf 변환 (ROS2 환경에서)
ros2 run xacro xacro m0609.urdf.xacro > m0609.urdf

# 이 폴더에 복사
cp m0609.urdf /path/to/isaac_aruco/urdf/m0609.urdf
```

## 2. Isaac Sim Asset Browser에서 직접 사용 (권장)

Isaac Sim 4.x에는 NVIDIA Robot Learning 에셋이 내장되어 있습니다.
두산 M0609가 없을 경우 아래 대안을 사용하세요:

- Isaac Sim Asset Browser → Robots → Manipulators
- 유사 6축 로봇: Franka Panda, UR10 등으로 테스트 후 교체

## 3. create_scene.py 에서 경로 수정

```python
# create_scene.py 상단
DOOSAN_URDF = ROOT / "urdf" / "m0609.urdf"   # ← 이 경로
```
