"""예제 3. 순기구학(FK) — 관절 각도에서 손끝 위치가 어떻게 결정되는지 본다.

joint1과 joint2를 사인파로 흔들면서 엔드이펙터의 실제 좌표를 읽어
그 궤적을 공간에 선으로 그린다.
"""

import math
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
EE_LINK = 7  # end_effector_joint 가 붙어 있는 링크

p.resetDebugVisualizerCamera(
    cameraDistance=0.7, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.15]
)

prev_pos = None
t = 0.0

while True:
    # 관절 각도를 시간에 따라 흔든다
    target = [
        0.8 * math.sin(t),  # joint1: 좌우 회전
        -0.6 + 0.4 * math.sin(2 * t),  # joint2: 위아래
        0.4,  # joint3: 고정
        0.6,  # joint4: 고정
    ]
    for idx, angle in zip(ARM_JOINTS, target):
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL, targetPosition=angle, force=30.0
        )

    p.stepSimulation()

    # FK 결과: 엔드이펙터의 월드 좌표 (관절 각도가 정하는 값)
    link_state = p.getLinkState(robot, EE_LINK)
    pos = link_state[4]  # worldLinkFramePosition

    # 손끝이 지나간 자리를 초록 선으로 남긴다
    if prev_pos is not None:
        p.addUserDebugLine(prev_pos, pos, lineColorRGB=[0, 1, 0], lineWidth=2, lifeTime=6)
    prev_pos = pos

    if int(t * 240) % 60 == 0:  # 0.25초마다 좌표 출력
        print(f"t={t:5.2f}  ee = ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) m")

    t += 1 / 240
    time.sleep(1 / 240)
