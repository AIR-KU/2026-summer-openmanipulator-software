"""예제 4. 역기구학(IK) — 원하는 손끝 좌표를 주면 관절 각도를 역산한다.

슬라이더로 목표 좌표(x, y, z)를 움직이면 파란 공이 그 위치로 이동하고,
로봇은 IK로 계산된 관절 각도를 따라가며 손끝을 그 공에 맞춘다.
예제 3(FK)과 정확히 반대 방향의 계산이다.
"""

import os
import sys
import time

import pybullet as p

# 저장소 루트를 import 경로에 넣어 omx 패키지를 찾게 한다.
# 이렇게 해 두면 어느 폴더에서 실행해도 URDF 를 제대로 찾는다.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from omx.sim import connect, load_robot

connect(gui=True)
p.setGravity(0, 0, -9.8)

# URDF 안의 메시가 상대경로라서 로딩을 헬퍼에 맡긴다 (omx/sim.py 참고).
# base_yaw=0 이므로 로봇 정면은 월드 +x 방향이다.
robot = load_robot(base_yaw=0)

ARM_JOINTS = [1, 2, 3, 4]
EE_LINK = 7  # end_effector_link

# 목표 지점을 표시할 파란 공 (충돌 없는 시각용 물체)
marker_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=[0, 0.4, 1, 1])
marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker_vis)

# 목표 좌표 슬라이더 (단위 m, OpenManipulator-X 작업 반경은 대략 30cm)
sx = p.addUserDebugParameter("target x", -0.05, 0.32, 0.20)
sy = p.addUserDebugParameter("target y", -0.25, 0.25, 0.00)
sz = p.addUserDebugParameter("target z", 0.02, 0.40, 0.15)

p.resetDebugVisualizerCamera(
    cameraDistance=0.7, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.15]
)

step = 0
while True:
    target = [
        p.readUserDebugParameter(sx),
        p.readUserDebugParameter(sy),
        p.readUserDebugParameter(sz),
    ]
    p.resetBasePositionAndOrientation(marker, target, [0, 0, 0, 1])

    # IK: 목표 좌표 -> 관절 각도. 결과는 '움직이는 모든 관절'에 대한 값이 나온다.
    ik = p.calculateInverseKinematics(robot, EE_LINK, target)

    # 앞의 4개가 joint1~4에 해당한다 (뒤 2개는 그리퍼)
    for n, idx in enumerate(ARM_JOINTS):
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL, targetPosition=ik[n], force=30.0
        )

    p.stepSimulation()

    # 실제 손끝이 목표에 얼마나 가까운지 (팔이 안 닿는 곳이면 오차가 커진다)
    step += 1
    if step % 120 == 0:
        ee = p.getLinkState(robot, EE_LINK)[4]
        err = sum((a - b) ** 2 for a, b in zip(ee, target)) ** 0.5
        print(
            f"목표 ({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f})  "
            f"실제 ({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f})  오차 {err * 1000:5.1f} mm"
        )

    time.sleep(1 / 240)
