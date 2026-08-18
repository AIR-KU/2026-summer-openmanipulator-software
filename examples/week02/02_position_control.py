"""예제 2. 위치 제어로 목표 자세 보내기.

미리 정해둔 자세들을 순서대로 명령하고, 목표 각도와 실제 각도의 차이를 관찰한다.
force / maxVelocity 값을 바꿔가며 로봇이 어떻게 달라지는지 보는 것이 핵심.
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

ARM_JOINTS = [1, 2, 3, 4]  # joint1 ~ joint4

# 목표 자세 목록 (joint1~4, 단위 rad)
POSES = {
    "home": [0.0, -1.0, 0.4, 0.6],
    "위로 뻗기": [0.0, -0.6, -0.3, 0.9],
    "왼쪽 보기": [math.pi / 3, -0.8, 0.3, 0.5],
    "앞으로 숙이기": [0.0, 0.4, 0.2, 0.7],
    "오른쪽 보기": [-math.pi / 3, -0.8, 0.3, 0.5],
}

FORCE = 30.0  # 관절이 낼 수 있는 최대 토크 [Nm] — 줄이면 중력에 못 버틴다
MAX_VELOCITY = 1.0  # 관절 최대 속도 [rad/s] — 키우면 급하게 움직인다

p.resetDebugVisualizerCamera(
    cameraDistance=0.7, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.15]
)


def go_to(target, seconds=2.0):
    """목표 각도를 명령하고 seconds 동안 시뮬레이션을 돌린다."""
    for idx, angle in zip(ARM_JOINTS, target):
        p.setJointMotorControl2(
            robot,
            idx,
            p.POSITION_CONTROL,
            targetPosition=angle,
            force=FORCE,
            maxVelocity=MAX_VELOCITY,
        )
    for _ in range(int(seconds * 240)):
        p.stepSimulation()
        time.sleep(1 / 240)


while True:
    for name, target in POSES.items():
        go_to(target)

        # 목표 각도 vs 실제 각도 — 중력과 토크 한계 때문에 완전히 일치하지 않는다
        actual = [p.getJointState(robot, i)[0] for i in ARM_JOINTS]
        errors = [a - t for a, t in zip(actual, target)]
        print(f"[{name}]")
        print("  목표:", " ".join(f"{v:+.3f}" for v in target))
        print("  실제:", " ".join(f"{v:+.3f}" for v in actual))
        print("  오차:", " ".join(f"{v:+.3f}" for v in errors))
