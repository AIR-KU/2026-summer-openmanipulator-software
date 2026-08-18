"""예제 5. 그리퍼 여닫기.

OpenManipulator-X의 그리퍼는 손가락 두 개가 각각 별도의 직진(prismatic) 관절이다.
URDF에는 gripper_sub 가 gripper 를 따라가도록 mimic 태그가 있지만,
PyBullet은 mimic 을 무시하므로 두 관절에 같은 값을 직접 명령해야 한다.
(gripper_sub 는 축 방향이 반대라, 같은 값을 주면 서로 반대쪽으로 벌어진다.)
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
GRIPPER_JOINTS = [5, 6]  # gripper, gripper_sub

OPEN = 0.019  # URDF 상한 = 활짝 벌림
CLOSED = -0.010  # URDF 하한 = 완전히 오므림

# 팔을 적당한 자세로 세워두고 그리퍼만 관찰한다
for idx, angle in zip(ARM_JOINTS, [0.0, -0.6, 0.2, 0.4]):
    p.setJointMotorControl2(robot, idx, p.POSITION_CONTROL, targetPosition=angle, force=30.0)

p.resetDebugVisualizerCamera(
    cameraDistance=0.35, cameraYaw=40, cameraPitch=-20, cameraTargetPosition=[0.2, 0, 0.25]
)


def set_gripper(width, steps=180):
    """두 손가락 관절에 같은 값을 명령한다."""
    for idx in GRIPPER_JOINTS:
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL, targetPosition=width, force=20.0
        )
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1 / 240)


while True:
    for name, width in [("열기", OPEN), ("닫기", CLOSED)]:
        set_gripper(width)
        actual = [p.getJointState(robot, i)[0] for i in GRIPPER_JOINTS]
        print(f"[{name}] 명령 {width:+.3f} m -> 실제 {actual[0]:+.4f} / {actual[1]:+.4f} m")
