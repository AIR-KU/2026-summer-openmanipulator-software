"""예제 6. Pick & Place — 4번(IK)과 5번(그리퍼)을 합친 최종 실습.

접근 -> 잡기 -> 들어올리기 -> 옮기기 -> 놓기 순서로 빨간 큐브를 옮긴다.

[이 예제의 두 가지 현실적 타협]

1) 큐브를 바닥이 아니라 8cm 높이의 테이블 위에 올려둔다.
   이 로봇의 손가락 충돌 형상은 손끝(end_effector_link)보다 3.5cm 아래까지
   뻗어 있어서, 바닥의 물체를 잡으러 가면 손가락이 먼저 바닥을 파고든다.

2) 잡기를 '고정 제약(constraint)'으로 모사한다.
   URDF의 손가락 충돌 형상은 오목한 실제 손가락을 볼록 껍질(convex hull)로
   근사한 것이라, 손가락 안쪽 면이 제대로 만들어지지 않는다. 그래서 물리적으로
   꽉 쥐어도 큐브가 튕겨 나가 파지가 매우 불안정하다.
   실제 로봇 시뮬레이션에서도 흔히 쓰는 방법대로, 그리퍼가 닫힐 때 손끝과 물체를
   고정 제약으로 묶고 열 때 푼다. 팔의 움직임(IK)과 제어는 전부 실제 물리이고,
   '손가락과 물체 사이의 마찰 접촉'만 제약으로 대신하는 것이다.
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
GRIPPER_JOINTS = [5, 6]
EE_LINK = 7

OPEN = 0.019
CLOSED = -0.010

TABLE_H = 0.08  # 테이블 높이
HALF = 0.015  # 큐브 반변 (한 변 3cm)
CUBE_Z = TABLE_H + HALF  # 큐브 중심 높이
GRASP_Z = CUBE_Z + 0.01  # 잡을 때 손끝 높이
LIFT_Z = CUBE_Z + 0.12  # 들어올릴 높이

PICK = [0.20, 0.00]
PLACE = [0.18, 0.11]

# 테이블
TABLE_HALF = [0.08, 0.13, TABLE_H / 2]
table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=TABLE_HALF)
table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=TABLE_HALF, rgbaColor=[0.8, 0.8, 0.8, 1])
p.createMultiBody(0, table_col, table_vis, [0.19, 0.03, TABLE_H / 2])

# 빨간 큐브
box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[HALF] * 3)
box_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[HALF] * 3, rgbaColor=[1, 0, 0, 1])
box = p.createMultiBody(0.03, box_col, box_vis, [PICK[0], PICK[1], CUBE_Z])
p.changeDynamics(box, -1, lateralFriction=1.5)

p.resetDebugVisualizerCamera(
    cameraDistance=0.6, cameraYaw=60, cameraPitch=-25, cameraTargetPosition=[0.15, 0.05, 0.1]
)

grasp_constraint = None


def set_gripper(width, steps=180):
    for idx in GRIPPER_JOINTS:
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL, targetPosition=width, force=30.0
        )
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1 / 240)


def move_to(x, y, z, steps=500):
    """IK로 손끝을 (x, y, z)로 보낸다."""
    ik = p.calculateInverseKinematics(robot, EE_LINK, [x, y, z])
    for n, idx in enumerate(ARM_JOINTS):
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL, targetPosition=ik[n], force=50.0, maxVelocity=1.5
        )
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1 / 240)


def attach():
    """그리퍼를 닫고 큐브를 손끝에 고정한다."""
    global grasp_constraint
    set_gripper(CLOSED)

    ee_pos, ee_orn = p.getLinkState(robot, EE_LINK)[4:6]
    box_pos, box_orn = p.getBasePositionAndOrientation(box)

    # 손끝 좌표계에서 본 큐브의 상대 위치/자세를 그대로 고정한다
    inv_pos, inv_orn = p.invertTransform(ee_pos, ee_orn)
    rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, box_pos, box_orn)

    grasp_constraint = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=EE_LINK,
        childBodyUniqueId=box,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=rel_pos,
        childFramePosition=[0, 0, 0],
        parentFrameOrientation=rel_orn,
    )


def release():
    """제약을 풀고 그리퍼를 연다."""
    global grasp_constraint
    if grasp_constraint is not None:
        p.removeConstraint(grasp_constraint)
        grasp_constraint = None
    set_gripper(OPEN)


def cube_pos():
    return p.getBasePositionAndOrientation(box)[0]


while True:
    print("1. 그리퍼 열고 큐브 위로 접근")
    set_gripper(OPEN)
    move_to(PICK[0], PICK[1], GRASP_Z + 0.08)

    print("2. 하강")
    move_to(PICK[0], PICK[1], GRASP_Z)

    print("3. 잡기")
    attach()

    print("4. 들어올리기")
    move_to(PICK[0], PICK[1], LIFT_Z)
    print(f"   큐브 높이: {cube_pos()[2] * 100:.1f} cm  (테이블 위면 {CUBE_Z * 100:.1f} cm)")

    print("5. 놓을 위치로 이동")
    move_to(PLACE[0], PLACE[1], LIFT_Z)
    move_to(PLACE[0], PLACE[1], GRASP_Z)

    print("6. 놓기")
    release()
    move_to(PLACE[0], PLACE[1], LIFT_Z)
    x, y, z = cube_pos()
    print(f"   큐브 최종 위치: ({x:+.3f}, {y:+.3f}, {z:+.3f})\n")

    time.sleep(1.0)
    p.resetBasePositionAndOrientation(box, [PICK[0], PICK[1], CUBE_Z], [0, 0, 0, 1])
