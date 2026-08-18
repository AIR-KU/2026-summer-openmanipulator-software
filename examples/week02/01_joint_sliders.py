"""예제 1. 슬라이더로 관절 직접 움직여보기.

GUI 좌측 슬라이더를 끌면 각 관절이 그 각도로 움직인다.
로봇의 관절 구성과 회전축을 눈으로 익히는 것이 목적.
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

# 움직일 수 있는 관절만 추린다 (FIXED 제외)
movable = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    if info[2] != p.JOINT_FIXED:
        movable.append(
            {
                "index": i,
                "name": info[1].decode(),
                "type": info[2],
                "lower": info[8],
                "upper": info[9],
                "max_force": info[10],
                "max_vel": info[11],
            }
        )

# 관절마다 슬라이더 하나씩 (시작값은 0)
for j in movable:
    j["slider"] = p.addUserDebugParameter(j["name"], j["lower"], j["upper"], 0.0)
    unit = "rad" if j["type"] == p.JOINT_REVOLUTE else "m"  # 그리퍼는 직진 관절이라 미터
    print(f"{j['index']}  {j['name']:<12} 범위 [{j['lower']:+.2f}, {j['upper']:+.2f}] {unit}")

# 카메라를 로봇에 맞춰 초기화
p.resetDebugVisualizerCamera(
    cameraDistance=0.7, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.15]
)

while True:
    for j in movable:
        target = p.readUserDebugParameter(j["slider"])
        p.setJointMotorControl2(
            robot,
            j["index"],
            p.POSITION_CONTROL,
            targetPosition=target,
            force=j["max_force"],
            maxVelocity=j["max_vel"],
        )
    p.stepSimulation()
    time.sleep(1 / 240)
