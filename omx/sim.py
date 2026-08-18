"""PyBullet 로딩 헬퍼.

URDF 를 띄우고 관절 인덱스를 찾는 정형화된 부분을 모아 둔다.
예제마다 같은 코드를 반복해서 적지 않도록.
"""

import os

import pybullet as p
import pybullet_data

from omx.constants import BASE_YAW, E_LEN, P_LEN
from omx.kinematics import ARM_JOINT_NAMES
from omx.paths import URDF_PATH


def connect(gui=True):
    """PyBullet 접속. GUI 를 못 여는 환경이면 DIRECT 로 내려간다."""
    if gui:
        try:
            p.connect(p.GUI)
        except p.error:
            print("  [알림] GUI 를 열 수 없어 DIRECT 모드로 실행합니다.")
            p.connect(p.DIRECT)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    return p


def load_robot(with_plane=True, base_yaw=BASE_YAW, base_position=(0, 0, 0)):
    """로봇 URDF 를 띄우고 body id 반환.

    URDF 안의 메시 경로가 ../meshes/*.stl 상대경로라서
    URDF 폴더로 잠깐 이동한 뒤 로드해야 STL 이 붙는다.
    """
    if with_plane:
        p.loadURDF("plane.urdf")

    cwd = os.getcwd()
    os.chdir(os.path.dirname(URDF_PATH))
    try:
        robot = p.loadURDF(
            URDF_PATH,
            basePosition=list(base_position),
            baseOrientation=p.getQuaternionFromEuler([0, 0, base_yaw]),
            useFixedBase=True,
        )
    finally:
        os.chdir(cwd)
    return robot


def find_indices(robot):
    """joint1~4 와 엔드이펙터 링크의 인덱스를 이름으로 찾기.

    URDF 가 바뀌면 인덱스도 바뀌므로 숫자를 박아 두지 않는다.
    """
    arm, ee = [], None
    for i in range(p.getNumJoints(robot)):
        name = p.getJointInfo(robot, i)[1].decode()
        if name in ARM_JOINT_NAMES:
            arm.append(i)
        if "end_effector" in name:
            ee = i
    if ee is None:
        ee = arm[-1] if arm else 0
    return arm, ee


def set_pose(robot, arm_joints, q):
    """관절각을 물리 없이 바로 반영 (자세 확인용)"""
    for idx, angle in zip(arm_joints, q):
        p.resetJointState(robot, idx, angle)


def pen_tip_position(robot, wrist_link):
    """FK 로 되짚은 펜 끝의 월드 좌표.

    손목 링크(link5) 프레임에서 펜 끝은 (P_LEN, 0, -E_LEN) 에 있다.
    링크가 뻗은 방향으로 P_LEN, 거기서 직각 아래로 E_LEN.

    IK 가 계산한 관절각을 넣고 이 값을 읽으면 원래 목표점이 나와야 한다.
    두 값의 차이가 곧 IK 의 위치 오차다.
    """
    st = p.getLinkState(robot, wrist_link, computeForwardKinematics=True)
    tip, _ = p.multiplyTransforms(st[4], st[5], (P_LEN, 0.0, -E_LEN), [0, 0, 0, 1])
    return tip


def check_limits_against_urdf(robot, limits):
    """코드의 관절 제한이 URDF 보다 넓은 곳이 있으면 경고 출력."""
    warned = False
    for k, name in enumerate(ARM_JOINT_NAMES):
        for i in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, i)
            if info[1].decode() != name:
                continue
            u_lo, u_hi = info[8], info[9]
            lo, hi = limits[k]
            if lo < u_lo - 1e-6 or hi > u_hi + 1e-6:
                if not warned:
                    print("\n=== 코드 제한이 URDF 보다 넓은 관절 ===")
                    warned = True
                print(f"  {name}  코드 [{lo:+.4f}, {hi:+.4f}]"
                      f"  vs  URDF [{u_lo:+.4f}, {u_hi:+.4f}]")
    return warned
