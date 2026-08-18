"""IK 수식이 맞는지 확인하는 검증 스크립트.

수식이나 상수를 고쳤으면 이걸 먼저 돌려서 네 가지가 다 통과하는지 본다.

  A. 영점 자세      모든 관절 0 일 때의 자세. PHI 를 정하는 기준점.
  B. 합 항등식      th2 + th3 + th4 가 목표점과 무관하게 상수인가.
                    손 계산으로도 확인할 수 있는 가장 강력한 검사.
  C. IK -> FK 왕복  IK 로 푼 각도를 넣었을 때 펜 끝이 목표점에 놓이는가.
                    오차 벡터까지 찍으므로 어긋난 방향을 바로 볼 수 있다.
  D. FK -> IK 왕복  임의 각도로 만든 위치를 IK 에 넣으면 원래 각도가 나오는가.

실행:  python scripts/ik_verify.py
       python scripts/ik_verify.py --no-gui     (창 없이 검사만)
"""

import os
import sys
import time
from math import degrees, sqrt

# 저장소 루트를 import 경로에 넣어 omx 패키지를 찾게 한다
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p

from omx.constants import A_OFF, B_OFF, BETA, E_LEN, L2, OFF, P_LEN
from omx.kinematics import ik, joint_limits, sum_identity, wrap
from omx.paths import URDF_PATH
from omx.sim import (
    connect,
    find_indices,
    load_robot,
    pen_tip_position,
    set_pose,
)

# Windows 콘솔(cp949)에서 기호가 깨지지 않도록
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass


# 종이 위 여러 점 - 정면, 좌, 우, 가까운 곳, 먼 곳
TARGETS = [
    (0.20, 0.00, 0.0),
    (0.18, 0.06, 0.0),
    (0.18, -0.06, 0.0),
    (0.16, 0.00, 0.0),
    (0.24, 0.00, 0.0),
]


def check_zero_pose(robot, arm, ee):
    """[A] 모든 관절 0 일 때 어떤 자세인지. 규약 상수를 정하는 기준점."""
    print("\n" + "=" * 58)
    print("  A. 영점 자세 (모든 관절 = 0)")
    print("=" * 58)

    set_pose(robot, arm, [0, 0, 0, 0])
    st = p.getLinkState(robot, ee, computeForwardKinematics=True)
    pos = st[4]
    pitch = degrees(p.getEulerFromQuaternion(st[5])[1])

    print(f"  EE 위치   x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")
    print(f"  EE pitch  {pitch:+.2f} deg")
    print()
    print("  pitch 가 0 근처면  -> link5 수평 -> PHI = 0 이 맞음")
    print("  0 이 아니면        -> 그 값을 PHI 에 반영")


def check_sum_identity(targets=TARGETS):
    """[B] 합 항등식. 목표가 어디든 th2+th3+th4 는 상수여야 한다."""
    print("\n" + "=" * 58)
    print("  B. 합 항등식  (th2 + th3 + th4 = 상수?)")
    print("=" * 58)

    sums = []
    for t in targets:
        q = ik(*t)
        if q is None:
            continue
        s = sum_identity(q)
        sums.append(s)
        print(f"  {str(t):<26} 합 = {degrees(s):+8.3f} deg")

    if not sums:
        print("  도달 가능한 점이 없습니다. P_LEN / E_LEN 을 확인하세요.")
        return False

    spread = degrees(max(sums) - min(sums))
    print("  " + "-" * 50)
    print(f"  편차: {spread:.6f} deg")
    ok = spread < 0.01
    print("  통과 - 합이 일정합니다" if ok
          else "  실패 - SIGN_234 / OFF / PHI 를 확인하세요")
    return ok


def check_ik_fk(robot, arm, wrist, targets=TARGETS):
    """[C] IK -> FK. 펜 끝을 FK 로 되짚어 목표점과 비교한다.

    오차 벡터까지 찍는 이유: 크기만 보면 어디가 틀렸는지 알 수 없지만,
    모든 목표점에서 방향과 크기가 같은 오차가 나오면 그건 계산 실수가
    아니라 빠뜨린 평행이동이다.
    """
    print("\n" + "=" * 58)
    print("  C. IK -> FK 왕복 검증 (펜 끝 기준)")
    print("=" * 58)
    print(f"  {'목표 (x, y, z)':<24}{'오차 벡터 [mm]':<28}{'크기':>8} {'pitch':>9}")
    print("  " + "-" * 68)

    errs = []
    for t in targets:
        q = ik(*t)
        if q is None:
            print(f"  {str(t):<24}{'도달 불가':<28}")
            continue

        set_pose(robot, arm, q)
        tip = pen_tip_position(robot, wrist)
        st = p.getLinkState(robot, wrist, computeForwardKinematics=True)
        pitch = degrees(p.getEulerFromQuaternion(st[5])[1])

        d = [1000 * (a - b) for a, b in zip(tip, t)]
        mag = sqrt(sum(v * v for v in d))
        errs.append((d, mag))
        print(f"  {str(t):<24}"
              f"({d[0]:+6.2f}, {d[1]:+6.2f}, {d[2]:+6.2f})       "
              f"{mag:6.2f} {pitch:+8.2f}")

    print()
    if not errs:
        print("  도달 가능한 점이 없습니다.")
        return

    worst = max(m for _, m in errs)
    print(f"  최대 오차 {worst:.3f} mm")
    if worst < 0.01:
        print("  통과 - 펜 끝이 목표점에 정확히 놓입니다.")
    else:
        # 모든 점의 오차 벡터가 같은 방향/크기인지 본다
        first = errs[0][0]
        uniform = all(
            max(abs(a - b) for a, b in zip(first, d)) < 0.01 for d, _ in errs
        )
        if uniform:
            print(f"  오차가 모든 점에서 ({first[0]:+.2f}, {first[1]:+.2f},"
                  f" {first[2]:+.2f}) mm 로 동일합니다.")
            print("  -> 계산 실수가 아니라 빠뜨린 평행이동입니다.")
            print("     constants.py 의 APPLY_X_OFFSET 을 True 로 켜 보세요.")
        else:
            print("  오차가 점마다 다릅니다 -> 수식이나 치수를 확인하세요.")
    print("  pitch 는 모든 점에서 일정해야 정상 (펜 자세가 안 변하므로)")


def check_roundtrip(robot, arm, wrist):
    """[D] FK -> IK. IK 와 FK 가 서로 역함수인지.

    임의 관절각으로 자세를 만들고, 그때의 펜 끝 위치를 IK 에 다시 넣어
    원래 각도가 돌아오는지 본다.
    """
    print("\n" + "=" * 58)
    print("  D. FK -> IK 왕복")
    print("=" * 58)

    for q_test in ([0.0, 0.3, -0.6, 0.3], [0.4, 0.2, -0.4, 0.2]):
        set_pose(robot, arm, q_test)
        tip = pen_tip_position(robot, wrist)

        q_back = ik(*tip)
        if q_back is None:
            print(f"  [실패] {[f'{a:+.2f}' for a in q_test]} -> 역산 실패")
            print(f"         펜 끝 {tuple(f'{v:+.4f}' for v in tip)}"
                  f" 이 관절 제한 밖이거나 도달 불가")
            continue

        diff = max(abs(wrap(a - b)) for a, b in zip(q_test, q_back))
        mark = "통과" if degrees(diff) < 1.0 else "실패"
        print(f"  [{mark}] 원본 {[f'{a:+.3f}' for a in q_test]}")
        print(f"         역산 {[f'{a:+.3f}' for a in q_back]}"
              f"   최대차 {degrees(diff):.3f} deg")

    print()
    print("  검사 C 에 평행이동 오차가 남아 있으면 여기서도 그만큼 어긋난다.")


def main():
    gui = "--no-gui" not in sys.argv

    print(f"\n  L2  = {L2:.5f} m      (sqrt({A_OFF}^2 + {B_OFF}^2))")
    print(f"  BETA = {degrees(BETA):.2f} deg")
    print(f"  OFF  = {degrees(OFF):.2f} deg")
    print(f"  P_LEN = {P_LEN:.4f} m   <- 실측")
    print(f"  E_LEN = {E_LEN:.4f} m   <- 실측")
    print(f"\n  URDF: {URDF_PATH}")

    print("\n  적용 중인 관절 제한")
    for i, (lo, hi) in enumerate(joint_limits(), start=1):
        print(f"    joint{i}  [{degrees(lo):+7.2f}, {degrees(hi):+7.2f}] deg")

    connect(gui=gui)
    robot = load_robot()
    arm, ee = find_indices(robot)
    print(f"\n  팔 관절 인덱스: {arm}    EE 링크 인덱스: {ee}")

    wrist = arm[-1]          # joint4 가 붙은 링크(link5). 펜은 여기 물려 있다.

    check_zero_pose(robot, arm, ee)
    check_sum_identity()
    check_ik_fk(robot, arm, wrist)
    check_roundtrip(robot, arm, wrist)

    if p.getConnectionInfo()["connectionMethod"] != p.GUI:
        p.disconnect()
        return

    print("\n  창을 닫으면 종료됩니다.")
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / 240.0)   # CPU 100% 점유 방지
    except (KeyboardInterrupt, p.error):
        pass


if __name__ == "__main__":
    main()
