"""OpenManipulator-X 역기구학 시각화 도구 (baseline).

실행하면 두 단계로 진행된다.
  1단계: 2D 평면(위에서 본 모습)에 도달 가능 영역이 뜬다.
         마우스로 점을 찍으면 순서대로 이어진다. 창을 닫으면 다음 단계.
  2단계: PyBullet 3D 창이 뜨고 로봇이 그 점들을 따라간다.
         슬라이더로 재생을 조절하고, 화면 좌측 텍스트로 각도를 본다.

실행:  python scripts/ik_viewer.py

IK 수식과 치수는 이 파일에 없다. omx/kinematics.py 와 omx/constants.py
에 있고, 여기서는 그것을 가져다 쓰기만 한다.
"""

import math
import os
import sys
import time

# 저장소 루트를 import 경로에 넣어 omx 패키지를 찾게 한다
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import numpy as np

from omx.constants import (
    A_OFF,
    B_OFF,
    BETA,
    E_LEN,
    HOME,
    L2,
    OFF,
    P_LEN,
    Z_PAPER,
    Z_UP,
)
from omx.kinematics import ik, joint_limits, limit_violations, sum_identity
from omx.paths import URDF_PATH

# Windows 콘솔(cp949)에서 화살표/기호가 깨지지 않도록
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass


# ============================================================
# 1. 2D 평면에서 점 찍기
# ============================================================

def scan_workspace(step=0.01, limit=0.36):
    """
    z = Z_PAPER 평면을 훑어 격자점을 두 갈래로 나눈다.
      (xs, ys) - 관절 제한까지 통과한 점
      (bx, by) - 기하적으로는 닿지만 관절 제한에 걸리는 점
    """
    xs, ys, bx, by = [], [], [], []
    grid = np.arange(-limit, limit + step, step)
    for gx in grid:
        for gy in grid:
            if ik(gx, gy, Z_PAPER, apply_limits=False) is None:
                continue                        # 기하적으로 불가
            if ik(gx, gy, Z_PAPER) is not None:
                xs.append(gx)
                ys.append(gy)
            else:
                bx.append(gx)
                by.append(gy)
    return xs, ys, bx, by


def pick_targets():
    """2D 창을 띄우고 사용자가 찍은 점들을 리스트로 반환"""
    print("작업 영역 스캔 중...")
    wx, wy, bx, by = scan_workspace()
    print(f"도달 가능 격자점 {len(wx)}개 "
          f"(관절 제한에 걸려 제외된 점 {len(bx)}개)")

    picked = []

    fig, ax = plt.subplots(figsize=(8, 8))
    if bx:
        ax.scatter(bx, by, s=8, c="#e0e0e0", label="joint limit", zorder=0)
    ax.scatter(wx, wy, s=8, c="#bcd9f2", label="reachable", zorder=1)
    ax.plot(0, 0, "ks", markersize=12, label="robot base", zorder=3)
    # theta1 = 0 이 +y 를 향하므로 정면 화살표도 12시 방향
    ax.arrow(0, 0, 0, 0.06, head_width=0.012, color="k", zorder=3)
    ax.text(0, 0.075, "+y (front, th1=0)", ha="center", fontsize=9)

    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.4, 0.4)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Click = add point   |   Drag = pan   |   Wheel = zoom\n"
                 "close this window when done", fontsize=11)
    ax.legend(loc="upper right")

    line, = ax.plot([], [], "r-o", markersize=6, linewidth=1.5, zorder=4)
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                   va="top", fontsize=10, family="monospace")

    def redraw():
        if picked:
            line.set_data([q[0] for q in picked], [q[1] for q in picked])
        else:
            line.set_data([], [])
        info.set_text(f"points: {len(picked)}")
        fig.canvas.draw_idle()

    # --- 드래그 팬 / 휠 줌 ---------------------------------------
    # 누른 채로 DRAG_PIXELS 이상 움직이면 화면 이동,
    # 그보다 적게 움직이고 떼면 점 찍기로 본다.
    DRAG_PIXELS = 4
    drag = {"xy": None, "moved": False}

    def on_press(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        drag["xy"] = (event.x, event.y)
        drag["moved"] = False

    def on_motion(event):
        if drag["xy"] is None or event.x is None:
            return
        px, py = drag["xy"]
        if not drag["moved"]:
            if math.hypot(event.x - px, event.y - py) < DRAG_PIXELS:
                return
            drag["moved"] = True

        # 픽셀 이동량을 데이터 좌표 이동량으로 환산
        inv = ax.transData.inverted()
        x0d, y0d = inv.transform((px, py))
        x1d, y1d = inv.transform((event.x, event.y))
        dx, dy = x0d - x1d, y0d - y1d

        xlo, xhi = ax.get_xlim()
        ylo, yhi = ax.get_ylim()
        ax.set_xlim(xlo + dx, xhi + dx)
        ax.set_ylim(ylo + dy, yhi + dy)

        drag["xy"] = (event.x, event.y)     # 기준점 갱신
        fig.canvas.draw_idle()

    def on_release(event):
        pressed, moved = drag["xy"], drag["moved"]
        drag["xy"] = None
        if pressed is None or moved:        # 화면을 옮긴 것이면 점 안 찍음
            return
        if event.inaxes is not ax or event.xdata is None:
            return

        x, y = float(event.xdata), float(event.ydata)
        if ik(x, y, Z_PAPER) is None:
            # 기하적으로 안 되는 것과 관절 제한에 걸린 것을 구분해서 알려준다
            raw = ik(x, y, Z_PAPER, apply_limits=False)
            if raw is None:
                print(f"  ({x:+.3f}, {y:+.3f}) 도달 불가 (팔 길이) - 무시")
            else:
                bad = "  ".join(
                    f"th{i}={math.degrees(a):+.1f}"
                    f"[{math.degrees(lo):+.0f},{math.degrees(hi):+.0f}]"
                    for i, a, lo, hi in limit_violations(raw)
                )
                print(f"  ({x:+.3f}, {y:+.3f}) 관절 제한 초과 - {bad}")
            return
        picked.append((x, y))
        print(f"  #{len(picked)}  ({x:+.3f}, {y:+.3f})")
        redraw()

    def on_scroll(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        scale = 0.85 if event.button == "up" else 1 / 0.85
        cx, cy = event.xdata, event.ydata
        xlo, xhi = ax.get_xlim()
        ylo, yhi = ax.get_ylim()
        ax.set_xlim(cx + (xlo - cx) * scale, cx + (xhi - cx) * scale)
        ax.set_ylim(cy + (ylo - cy) * scale, cy + (yhi - cy) * scale)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    redraw()
    plt.show()

    return picked


# ============================================================
# 2. 궤적 만들기
# ============================================================

def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def fill_joint_gaps(traj, max_step=math.radians(3.0)):
    """
    인접 샘플의 관절 차이가 max_step 을 넘으면 그 사이를 관절 공간에서
    쪼개 채운다. 도달 불가 구간이 삭제돼 생긴 끊김을, 각도를 순서대로
    훑는 부드러운 이동으로 바꿔 준다.

    채워 넣은 샘플의 위치는 직교 보간이라 실제 FK 와 다르다. 그래서
    pen_down=False 로 두어 그리지 않고 이동만 하게 한다.
    """
    if len(traj) < 2:
        return traj, 0

    out = [traj[0]]
    added = 0
    for pos_b, q_b, pen_b in traj[1:]:
        pos_a, q_a, _ = out[-1]
        gap = max(abs(x - y) for x, y in zip(q_a, q_b))
        if gap > max_step:
            n = int(gap / max_step) + 1
            for k in range(1, n):
                t = k / n
                q_mid = [x + (y - x) * t for x, y in zip(q_a, q_b)]
                p_mid = tuple(x + (y - x) * t for x, y in zip(pos_a, pos_b))
                out.append((p_mid, q_mid, False))
                added += 1
        out.append((pos_b, q_b, pen_b))
    return out, added


def build_trajectory(points, steps=50):
    """
    HOME -> 점1 위 로 한 번 진입한 뒤,
    점1 -> ... -> 점N -> 점N 위 -> 점1 위 구간을 계속 반복할 수 있게
    시작과 끝이 같은 닫힌 루프로 만든다.

    반환: (traj, loop_start)
      traj       - (위치, 각도, 펜다운여부) 샘플 리스트
      loop_start - 반복 구간이 시작하는 traj 인덱스.
                   재생이 끝에 닿으면 0 이 아니라 여기로 돌아간다.
    """
    if not points:
        return [], 0

    above_first = (points[0][0], points[0][1], Z_PAPER + Z_UP)
    above_last = (points[-1][0], points[-1][1], Z_PAPER + Z_UP)

    approach = [HOME, above_first]                      # 진입 (1회)
    loop = [above_first]                                # 반복 구간
    loop += [(px, py, Z_PAPER) for px, py in points]
    loop.append(above_last)
    loop.append(above_first)                            # 시작점 복귀 -> 닫힘

    dropped = [0]            # IK 가 안 풀려 빠진 샘플 수
    carry = [None]           # 직전 관절값 (theta1 주기 해 선택용)

    def sample(waypoints):
        out = []
        for a, b in zip(waypoints[:-1], waypoints[1:]):
            for i in range(steps + 1):
                pos = lerp(a, b, i / steps)
                q = ik(*pos, prev_q=carry[0])
                if q is None:
                    dropped[0] += 1
                    continue
                carry[0] = q
                pen_down = pos[2] <= Z_PAPER + 1e-6
                out.append((pos, q, pen_down))
        return out

    head = sample(approach)
    tail = sample(loop)

    # 끊긴 곳을 관절 공간 보간으로 메운다. 진입/루프를 따로 처리해
    # loop_start 가 채워진 샘플만큼 밀리는 것까지 반영한다.
    head, add_h = fill_joint_gaps(head)
    tail, add_t = fill_joint_gaps(tail)

    traj = head + tail
    loop_start = len(head)
    if loop_start >= len(traj):     # 루프 구간이 통째로 걸러진 경우 방어
        loop_start = 0

    if dropped[0]:
        print(f"  [경고] 관절 제한/도달불가로 샘플 {dropped[0]}개가 빠졌습니다.")
        print(f"         끊긴 구간은 관절 보간 {add_h + add_t}개로 메웠습니다"
              f" (그 구간은 펜을 들고 이동).")
        worst, at = 0.0, 0
        for i in range(len(traj) - 1):
            d = max(abs(a - b) for a, b in zip(traj[i][1], traj[i + 1][1]))
            if d > worst:
                worst, at = d, i
        if worst > math.radians(5):
            print(f"         그래도 남은 최대 점프 {math.degrees(worst):.1f}도"
                  f" (샘플 {at} -> {at + 1})")

    return traj, loop_start


# ============================================================
# 3. PyBullet 재생
# ============================================================

def run_pybullet(traj, points, loop_start=0):
    import pybullet as pb

    from omx.sim import (
        check_limits_against_urdf,
        connect,
        find_indices,
        load_robot,
    )

    if not traj:
        print("궤적이 비어 있어 3D 재생을 건너뜁니다.")
        return

    connect(gui=True)
    # COV_ENABLE_GUI 를 0 으로 하면 슬라이더 패널까지 통째로 사라진다.
    # 켜 두고, 거슬리는 좌상단 미리보기 창만 따로 끈다.
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # 이게 켜져 있으면 좌클릭 드래그가 로봇 끌기로 먹혀서 카메라가
    # 안 움직인다. 끄면 드래그가 온전히 화면 조작이 된다.
    pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)

    # 로봇이 +y 를 향하므로 카메라도 90도 같이 돌려 정면이 잡히게
    CAM_TARGET = [0, 0.15, 0.1]
    CAM_YAW = 140
    pb.resetDebugVisualizerCamera(
        cameraDistance=0.7, cameraYaw=CAM_YAW, cameraPitch=-30,
        cameraTargetPosition=CAM_TARGET,
    )

    robot = load_robot()

    print("\n=== 관절 목록 ===")
    for i in range(pb.getNumJoints(robot)):
        print(f"  [{i}] {pb.getJointInfo(robot, i)[1].decode()}")

    check_limits_against_urdf(robot, joint_limits())

    arm_joints, ee_link = find_indices(robot)
    print(f"\n  ARM_JOINTS = {arm_joints}    EE_LINK = {ee_link}")

    if len(arm_joints) < 4:
        print("  joint1~4 를 찾지 못했습니다. URDF 를 확인하세요.")
        pb.disconnect()
        return

    # 목표점 표시
    for px, py in points:
        pb.addUserDebugLine([px, py, Z_PAPER], [px, py, Z_PAPER + 0.015],
                            [1, 0, 0], 3, 0)

    # 슬라이더
    sl_prog = pb.addUserDebugParameter("progress", 0.0, 1.0, 0.0)
    sl_speed = pb.addUserDebugParameter("speed", 0.0, 3.0, 0.6)
    sl_play = pb.addUserDebugParameter("auto play (0=off)", 0, 1, 1)

    # 카메라 슬라이더. 화면 조작은 PyBullet 내장 마우스 동작에 맡기고,
    # 이 슬라이더는 실제로 움직였을 때만 카메라를 덮어쓴다.
    sl_yaw = pb.addUserDebugParameter("cam yaw", -180, 180, CAM_YAW)
    sl_pitch = pb.addUserDebugParameter("cam pitch", -89, 89, -30)
    sl_dist = pb.addUserDebugParameter("cam dist", 0.1, 2.0, 0.7)
    cam_prev = None

    # 사이드 텍스트 (한 줄에 하나씩)
    n_lines = 9
    text_ids = []
    for i in range(n_lines):
        text_ids.append(pb.addUserDebugText(
            "", [0.0, 0.26, 0.34 - i * 0.028],
            textColorRGB=[0.1, 0.1, 0.1], textSize=1.1,
        ))

    prev_lines = [None] * n_lines   # 바뀐 줄만 다시 그리기 위한 캐시
    prev_prog = -1.0
    last_pos = None
    trace_ids = []          # 펜 궤적 선들의 id (한 바퀴 돌면 지운다)
    n = len(traj)
    last_i = max(n - 1, 1)  # 0 나눗셈 방지
    auto_i = 0.0            # 재생 위치를 traj 인덱스로 직접 들고 간다
    cycles = 0

    print("\n[마우스] PyBullet 기본 조작 (드래그 = 회전, 휠 = 줌)")
    print("[슬라이더] 우측 패널에 6개 (안 보이면 창 우상단 화살표로 펼치기)")
    print("    progress  - auto play를 0으로 내리면 수동 스크럽")
    print("    speed     - 재생 속도 (0 ~ 3, 기본 0.6)")
    print("    auto play - 0이면 정지, 1이면 자동 재생")
    print("    cam yaw / pitch / dist - 마우스가 안 먹을 때 카메라 조절")
    print("창을 닫으면 종료됩니다.\n")

    while pb.isConnected():
        try:
            playing = pb.readUserDebugParameter(sl_play) > 0.5
            prog_in = pb.readUserDebugParameter(sl_prog)
            cam_now = (pb.readUserDebugParameter(sl_yaw),
                       pb.readUserDebugParameter(sl_pitch),
                       pb.readUserDebugParameter(sl_dist))
        except pb.error:
            break       # 창을 닫는 도중이면 정상 종료

        # 슬라이더를 건드렸을 때만 카메라를 덮어쓴다.
        # 매 프레임 호출하면 마우스로 돌린 게 즉시 되돌려져서 안 된다.
        if cam_prev is not None and cam_now != cam_prev:
            try:
                pb.resetDebugVisualizerCamera(
                    cameraDistance=cam_now[2], cameraYaw=cam_now[0],
                    cameraPitch=cam_now[1], cameraTargetPosition=CAM_TARGET,
                )
            except pb.error:
                break
        cam_prev = cam_now

        if playing:
            speed = pb.readUserDebugParameter(sl_speed)
            auto_i += speed * last_i / 240.0

            if auto_i > last_i:
                # 끝에 닿으면 HOME(인덱스 0)이 아니라 루프 시작으로 되돌린다.
                # 넘친 양을 이어받아 속도가 튀지 않게 한다.
                overshoot = auto_i - last_i
                loop_len = max(last_i - loop_start, 1)
                auto_i = loop_start + (overshoot % loop_len)
                cycles += 1
                last_pos = None
                for tid in trace_ids:       # 궤적만 지우고 슬라이더는 유지
                    pb.removeUserDebugItem(tid)
                trace_ids = []

            idx = min(int(auto_i), n - 1)
            t = auto_i / last_i
        else:
            t = prog_in
            idx = min(int(t * last_i), n - 1)
            auto_i = float(idx)
            if abs(t - prev_prog) > 1e-6:
                last_pos = None

        prev_prog = t
        pos, q, pen_down = traj[idx]

        for i, j in enumerate(arm_joints[:4]):
            pb.resetJointState(robot, j, q[i])

        # 펜 궤적 그리기
        if playing and last_pos is not None:
            color = [0.85, 0.2, 0.1] if pen_down else [0.7, 0.7, 0.7]
            width = 2.5 if pen_down else 1.0
            trace_ids.append(pb.addUserDebugLine(last_pos, pos, color, width, 0))
        last_pos = pos

        # 사이드 텍스트
        lines = [
            f"step {idx+1}/{n}  cyc {cycles}",
            "",
            f"x  {pos[0]:+.4f}",
            f"y  {pos[1]:+.4f}",
            f"z  {pos[2]:+.4f}",
            "",
            f"th1 {math.degrees(q[0]):+7.2f}",
            f"th2 {math.degrees(q[1]):+7.2f}",
            f"th3 {math.degrees(q[2]):+7.2f}",
        ]
        try:
            for i, tid in enumerate(text_ids):
                txt = lines[i] if i < len(lines) else ""
                if txt == prev_lines[i]:
                    continue        # 안 바뀐 줄은 건너뛰기 (프레임률 확보)
                text_ids[i] = pb.addUserDebugText(
                    txt, [0.0, 0.26, 0.34 - i * 0.028],
                    textColorRGB=[0.1, 0.1, 0.1], textSize=1.1,
                    replaceItemUniqueId=tid,
                )
                prev_lines[i] = txt
            pb.stepSimulation()
        except pb.error:
            break       # 창을 닫는 도중이면 정상 종료
        time.sleep(1 / 240.0)


# ============================================================
# 4. 실행
# ============================================================

def main():
    print("=" * 50)
    print("상수")
    print("=" * 50)
    print(f"  L2   = {L2:.5f}  (sqrt({A_OFF}^2 + {B_OFF}^2))")
    print(f"  BETA = {math.degrees(BETA):.2f} deg")
    print(f"  OFF  = {math.degrees(OFF):.2f} deg")
    print(f"  P    = {P_LEN:.4f}   <- 실측 필요")
    print(f"  E    = {E_LEN:.4f}   <- 실측 필요")
    print(f"  URDF = {URDF_PATH}")

    # 합 항등식 검사
    print("\n=== 합 항등식 (목표가 달라도 일정해야 함) ===")
    sums = []
    for tx, ty in [(0.20, 0.0), (0.18, 0.06), (0.22, -0.05)]:
        q = ik(tx, ty, Z_PAPER)
        if q:
            s = sum_identity(q)
            sums.append(s)
            print(f"  ({tx:.2f}, {ty:+.2f})  합 = {math.degrees(s):8.3f} deg")
    if len(sums) >= 2:
        spread = math.degrees(max(sums) - min(sums))
        print(f"  편차 {spread:.5f} deg  ->  "
              f"{'통과' if spread < 1e-4 else '실패 - 계산 확인'}")

    # 창을 못 띄우는 백엔드면 plt.show() 가 그냥 지나가 버린다
    import matplotlib
    if not matplotlib.get_backend().lower().startswith(
            ("tkagg", "qtagg", "qt5agg", "wxagg", "macosx", "nbagg", "webagg")):
        print(f"\n[경고] matplotlib 백엔드가 '{matplotlib.get_backend()}' 입니다.")
        print("       창이 안 뜨면 클릭을 받을 수 없습니다 (tkinter 설치 확인).")

    print("\n2D 창에서 점을 찍고 창을 닫으세요.")
    points = pick_targets()

    if not points:
        print("점을 찍지 않아 종료합니다.")
        return

    traj, loop_start = build_trajectory(points)
    print(f"\n궤적 샘플 {len(traj)}개 생성 "
          f"(진입 {loop_start}개, 반복 구간 {len(traj) - loop_start}개)")

    if not traj:
        print("IK 가 풀린 샘플이 없습니다. P_LEN / E_LEN / HOME 을 확인하세요.")
        return

    try:
        run_pybullet(traj, points, loop_start)
    except Exception:
        import traceback
        print("\nPyBullet 실행 실패:")
        traceback.print_exc()


if __name__ == "__main__":
    main()
