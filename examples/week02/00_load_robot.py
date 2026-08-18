import os
import sys
import time

import pybullet as p

# 저장소 루트를 import 경로에 넣어 omx 패키지를 찾게 한다.
# 이렇게 해 두면 어느 폴더에서 실행해도 URDF 를 제대로 찾는다.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from omx.sim import connect, load_robot

# GUI 창 열기 + 바닥
connect(gui=True)
p.setGravity(0, 0, -9.8)

# 로봇 불러오기.
# URDF 안의 메시가 상대경로라서 로딩을 헬퍼에 맡긴다 (omx/sim.py 참고).
# base_yaw=0 이므로 로봇 정면은 월드 +x 방향이다.
robot = load_robot(base_yaw=0)

# 빨간 정육면체 (한 변 4cm), 로봇 앞쪽에 배치
half = 0.02
box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, half, half])
box_vis = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[half, half, half], rgbaColor=[1, 0, 0, 1]
)
box = p.createMultiBody(
    baseMass=0.1,
    baseCollisionShapeIndex=box_col,
    baseVisualShapeIndex=box_vis,
    basePosition=[0.2, 0, half],
)

# 관절 정보 출력
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(i, info[1].decode())

# 시뮬레이션 루프
while True:
    p.stepSimulation()
    time.sleep(1 / 240)
