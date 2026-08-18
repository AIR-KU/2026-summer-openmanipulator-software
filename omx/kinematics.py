"""OpenManipulator-X 해석적 역기구학.

펜 끝 목표 좌표 (x, y, z) 를 넣으면 관절각 [th1, th2, th3, th4] 가 나온다.

풀이는 두 덩어리다.
  A. 수평 회전   : theta1 하나로 끝난다.            -> [수식 1]
  B. r-z 평면 2링크 : 코사인 법칙으로 psi2, psi3 를
                     구한 뒤 관절각으로 옮긴다.     -> [수식 2~4]

기호
  r          = sqrt(x^2 + y^2)      베이스에서 잰 수평 거리
  (rw, zw)   펜 끝에서 역산한 손목 위치
  (dr, dz)   어깨를 원점으로 다시 잰 손목 위치
  D          어깨 <-> 손목 직선거리
  alpha      코사인 법칙으로 구한 삼각형 내각
  psi2, psi3 기하 좌표계의 각도 (관절각이 아니다)
  theta_i    실제 관절각. psi 에 영점 오프셋과 팀 부호를 적용한 값

치수는 전부 constants.py 에서 온다. 수치를 고칠 일이 있으면 그쪽을 보라.
"""

import xml.etree.ElementTree as ET
from math import acos, atan2, cos, hypot, pi, sin

from omx.constants import (
    APPLY_X_OFFSET,
    BASE_YAW,
    BETA,
    E_LEN,
    ELBOW_UP,
    JOINT1_FULL_TURN,
    L1,
    L2,
    L3,
    NO_BACKWARD_ARM,
    OFF,
    P_LEN,
    PHI,
    SIGN_234,
    TH1_OFFSET,
    TH1_SIGN,
    X_OFF,
)
from omx.paths import URDF_PATH

ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4")


def wrap(angle):
    """각도를 -pi ~ +pi 로 접기"""
    return (angle + pi) % (2 * pi) - pi


# ══════════════════════════════════════════════════════════════
#  제약조건 (1) — 관절 제한
#      URDF 의 <limit> 을 그대로 읽는다. 손으로 베껴 적으면
#      URDF 와 어긋나므로 파싱해서 가져온다.
# ══════════════════════════════════════════════════════════════

def load_joint_limits(urdf_path=URDF_PATH, names=ARM_JOINT_NAMES):
    """URDF 에서 관절 제한을 [(하한, 상한), ...] 로 읽기"""
    found = {}
    for j in ET.parse(urdf_path).getroot().iter("joint"):
        name = j.get("name")
        if name not in names:
            continue
        el = j.find("limit")
        if el is not None and el.get("lower") and el.get("upper"):
            found[name] = (float(el.get("lower")), float(el.get("upper")))

    missing = [n for n in names if n not in found]
    if missing:
        raise ValueError(f"URDF 에 {missing} 의 <limit> 이 없습니다: {urdf_path}")
    return [found[n] for n in names]


def _build_limits():
    """URDF 제한에 constants.py 의 제약 스위치를 얹은 최종 제한"""
    limits = load_joint_limits()

    if JOINT1_FULL_TURN:                      # 사각지대 없애기
        limits[0] = (-pi, pi)

    if NO_BACKWARD_ARM:                       # 상완이 뒤로 젖혀지지 않게
        lo, hi = limits[1]
        limits[1] = (max(lo, -BETA), hi)

    return limits


JOINT_LIMITS = _build_limits()


def joint_limits():
    """현재 적용 중인 관절 제한 [(하한, 상한), ...]"""
    return list(JOINT_LIMITS)


def limit_violations(q):
    """제한을 벗어난 관절 목록. [(관절번호, 각도, 하한, 상한), ...]"""
    out = []
    for i, ang in enumerate(q[:len(JOINT_LIMITS)]):
        lo, hi = JOINT_LIMITS[i]
        if ang < lo or ang > hi:
            out.append((i + 1, ang, lo, hi))
    return out


# ══════════════════════════════════════════════════════════════
#  [수식 1]  베이스 회전
#
#      theta1 = TH1_SIGN * atan2(y, x) + TH1_OFFSET
#
#  atan2(y, x) 는 월드 방위각. 로봇을 BASE_YAW 만큼 돌려 놓았으므로
#  그만큼 빼야 관절값이 된다 (TH1_OFFSET = -BASE_YAW).
# ══════════════════════════════════════════════════════════════

def solve_theta1(x, y, prev_q=None):
    """수평 방위에서 theta1 을 구한다.

    theta1 은 2pi 주기라 해가 여러 개다. 관절 제한 안에 드는 후보만
    남기고, prev_q 가 있으면 그중 이전 자세에서 가장 덜 움직이는 것을
    고른다. wrap 경계에서 관절이 반 바퀴 튀는 것을 막는다.
    """
    raw = TH1_SIGN * atan2(y, x) + TH1_OFFSET

    lo, hi = JOINT_LIMITS[0]
    base = wrap(raw)
    inside = [c for c in (base - 2 * pi, base, base + 2 * pi) if lo <= c <= hi]
    if not inside:
        return base            # 어차피 제한 검사에서 걸린다

    if prev_q is not None:
        return min(inside, key=lambda c: abs(c - prev_q[0]))
    return min(inside, key=abs)


# ══════════════════════════════════════════════════════════════
#  r-z 평면 2링크 문제  ->  psi2, psi3
#
#  1) 펜 끝 -> 손목      rw = r - P_LEN,   zw = z + E_LEN
#     펜이 링크에 직각으로 물려 있어 두 성분이 서로 섞이지 않는다.
#  2) 어깨를 원점으로     dr = rw,          dz = zw - L1
#  3) 코사인 법칙        cos(alpha) = (L2^2 + L3^2 - D^2) / (2 L2 L3)
#     psi3 = -(pi - alpha)        (ELBOW_UP = True 기준)
#  4) 어깨 각도
#     psi2 = atan2(dz, dr) - atan2(L3 sin(psi3), L2 + L3 cos(psi3))
# ══════════════════════════════════════════════════════════════

def solve_geometry(r, z):
    """r-z 평면 2링크를 풀어 (psi2, psi3) 반환. 도달 불가면 None."""
    # 1) 펜 끝 -> 손목 역산
    rw = r - P_LEN
    zw = z + E_LEN

    # 2) 어깨를 원점으로 다시 재기
    dr = rw
    dz = zw - L1
    D = hypot(dr, dz)          # 삼각형의 세 번째 변

    # ── 제약조건 (2) 도달 가능 조건 ────────────────────────────
    #    |L2 - L3| <= D <= L2 + L3
    #    acos 를 부르기 **전에** 반드시 확인해야 한다.
    if not (abs(L2 - L3) <= D <= L2 + L3):
        return None

    # 3) 코사인 법칙 -> 팔꿈치
    cos_alpha = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))   # 부동소수 오차 방어
    alpha = acos(cos_alpha)                       # 삼각형 내각 (0 ~ pi)

    # 제약조건 (3) 팔꿈치 분기는 궤적 전체에서 하나로 통일한다
    psi3 = -(pi - alpha) if ELBOW_UP else (pi - alpha)

    # 4) 어깨
    psi2 = atan2(dz, dr) - atan2(L3 * sin(psi3), L2 + L3 * cos(psi3))

    return psi2, psi3


# ══════════════════════════════════════════════════════════════
#  [수식 2~4]  기하각 -> 관절각
#
#      theta2 = SIGN_234 * (psi2 - OFF)
#      theta3 = SIGN_234 * (psi3 + OFF)
#      theta4 = SIGN_234 * (PHI - psi2 - psi3)
#
#  OFF = pi/2 - BETA 는 꺾인 링크 때문에 생기는 영점 오프셋이고,
#  SIGN_234 는 팀이 정한 회전 방향이다.
#
#  세 식을 더하면 psi 가 전부 소거되어
#      theta2 + theta3 + theta4 = SIGN_234 * PHI  (상수)
#  가 된다. 목표점이 어디든 이 합은 변하지 않는다.  -> 합 항등식
# ══════════════════════════════════════════════════════════════

def to_joint_angles(psi2, psi3):
    """기하각 -> 관절각. 영점 오프셋 보정 + 부호 규약 적용"""
    th2 = SIGN_234 * (psi2 - OFF)
    th3 = SIGN_234 * (psi3 + OFF)
    th4 = SIGN_234 * (PHI - psi2 - psi3)
    return th2, th3, th4


# ══════════════════════════════════════════════════════════════
#  전체 IK
# ══════════════════════════════════════════════════════════════

def shoulder_offset_world():
    """어깨 기둥이 베이스 중심축에서 밀린 양을 월드 좌표로 환산.

    X_OFF 는 베이스 프레임의 +x 방향 고정 오프셋이고, 로봇을 BASE_YAW
    만큼 돌려 놓았으므로 월드에서는 그만큼 회전한 방향이 된다.
    BASE_YAW = pi/2 이면 (0, X_OFF) — 즉 월드 +y 로 12mm.
    """
    if not APPLY_X_OFFSET:
        return 0.0, 0.0
    return X_OFF * cos(BASE_YAW), X_OFF * sin(BASE_YAW)


def ik(x, y, z, apply_limits=True, prev_q=None):
    """펜 끝 목표 (x, y, z) -> 관절각 [th1, th2, th3, th4]. 불가면 None.

    z 는 펜 끝 높이. 종이가 베이스 평면이면 종이에 닿는 높이가 z = 0.

    apply_limits=False 로 부르면 관절 제한을 무시한다.
    기하적으로 안 되는 것과 제한 때문에 안 되는 것을 구분할 때 쓴다.

    prev_q 를 주면 theta1 의 주기 해 중 이전 자세에 가장 가까운 것을 고른다.
    """
    # 제약조건 (4) 어깨 오프셋. APPLY_X_OFFSET 가 False 면 아무 일도 없다.
    ox, oy = shoulder_offset_world()
    x, y = x - ox, y - oy

    geo = solve_geometry(hypot(x, y), z)
    if geo is None:
        return None                       # 제약 (2) 도달 불가

    th1 = solve_theta1(x, y, prev_q)
    th2, th3, th4 = to_joint_angles(*geo)

    q = [th1, wrap(th2), wrap(th3), wrap(th4)]
    if apply_limits and limit_violations(q):
        return None                       # 제약 (1) 관절 제한 초과
    return q


def sum_identity(q):
    """합 항등식 값 th2 + th3 + th4. 목표점과 무관하게 상수여야 한다."""
    return q[1] + q[2] + q[3]
