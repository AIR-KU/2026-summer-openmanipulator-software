# 2026 여름방학 OpenManipulator-X 소프트웨어 스터디

OpenManipulator-X(ROBOTIS)를 PyBullet 위에서 다루는 스터디 저장소입니다.
URDF 모델, 주차별 예제, 그리고 해석적 역기구학(IK) baseline 코드가 들어 있습니다.

로봇에 펜을 물려 종이 위의 점을 잇는 것이 목표라서, IK는 **펜 끝 좌표**를 입력으로
받아 **관절각 4개**를 내놓습니다.

---

## 목차

- [빠르게 시작하기](#빠르게-시작하기)
- [폴더 구조](#폴더-구조)
- [실행해 볼 것](#실행해-볼-것)
- [역기구학 수식](#역기구학-수식)
- [제약조건](#제약조건)
- [값을 고칠 때](#값을-고칠-때)
- [출처와 라이선스](#출처와-라이선스)

---

## 빠르게 시작하기

### 1. 저장소 받기

URDF와 메시(STL)가 저장소에 함께 들어 있어 따로 내려받을 것이 없습니다.

```bash
git clone https://github.com/AIR-KU/2026-summer-openmanipulator-software.git
cd 2026-summer-openmanipulator-software
```

### 2. 가상환경 만들기

전역 파이썬을 더럽히지 않도록 가상환경을 씁니다. Python 3.9 이상이면 됩니다.

```bash
python -m venv .venv
```

만든 뒤 활성화합니다. 셸에 따라 명령이 다릅니다.

| 환경 | 활성화 명령 |
|---|---|
| Windows PowerShell | `.venv\Scripts\Activate.ps1` |
| Windows CMD | `.venv\Scripts\activate.bat` |
| Git Bash / macOS / Linux | `source .venv/bin/activate` |

PowerShell에서 실행 정책 오류가 나면 그 창에서 한 번만 아래를 실행하고 다시 활성화하세요.

```bash
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

### 3. 패키지 설치

```bash
pip install -r requirements.txt
```

`pybullet`은 설치할 때 C++ 확장을 빌드합니다. Windows에서 빌드 오류가 나면
[Build Tools for Visual Studio](https://visualstudio.microsoft.com/downloads/)의
"C++를 사용한 데스크톱 개발"을 설치한 뒤 다시 시도하세요.

### 4. 설치 확인

로봇이 창에 뜨면 성공입니다.

```bash
python examples/week02/00_load_robot.py
```

창이 안 뜨고 검사만 하고 싶으면 이쪽을 쓰세요.

```bash
python scripts/ik_verify.py --no-gui
```

### URDF를 원본에서 직접 받고 싶다면

저장소에 이미 포함돼 있으므로 보통은 필요 없습니다. 원본과 대조하거나
다른 버전을 쓰고 싶을 때만 참고하세요.

```bash
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
```

받은 뒤 `open_manipulator_description/urdf/` 와 `meshes/` 를 쓰면 됩니다.
다른 위치의 URDF를 쓰려면 코드를 고칠 필요 없이 환경변수로 지정하세요.

```bash
export OMX_URDF=/절대/경로/open_manipulator_robot.urdf
```

---

## 폴더 구조

```
.
├── omx/                        공용 모듈 (여기가 핵심)
│   ├── constants.py            치수와 규약 상수 — 값을 고치는 곳
│   ├── kinematics.py           IK 수식 — 수식을 고치는 곳
│   ├── paths.py                URDF 경로 해석
│   └── sim.py                  PyBullet 로딩 헬퍼
│
├── scripts/
│   ├── ik_viewer.py            baseline. 2D로 점 찍고 3D로 재생
│   └── ik_verify.py            수식이 맞는지 검사하는 4종 세트
│
├── examples/week02/            주차별 예제 (PyBullet 기본기)
│   ├── 00_load_robot.py        로봇 띄우기
│   ├── 01_joint_sliders.py     슬라이더로 관절 직접 움직이기
│   ├── 02_position_control.py  위치 제어로 목표 자세 보내기
│   ├── 03_forward_kinematics.py  FK — 각도에서 손끝 위치
│   ├── 04_inverse_kinematics.py  IK — 손끝 위치에서 각도 (PyBullet 내장)
│   ├── 05_gripper.py           그리퍼 열고 닫기
│   └── 06_pick_and_place.py    집어서 옮기기
│
├── open_manipulator_description/
│   ├── urdf/open_manipulator_robot.urdf
│   └── meshes/*.stl
│
└── requirements.txt
```

수식과 치수는 `omx/` 안에만 있습니다. `scripts/` 와 `examples/` 는 그것을
가져다 쓰기만 하므로, 값을 고칠 때 여러 파일을 뒤질 일이 없습니다.

---

## 실행해 볼 것

### `scripts/ik_viewer.py` — baseline

```bash
python scripts/ik_viewer.py
```

두 단계로 진행됩니다.

1. **2D 창**에서 위에서 본 작업 영역이 뜹니다. 마우스로 점을 찍으면 순서대로
   이어집니다. 파란 점은 도달 가능, 회색 점은 팔은 닿지만 관절 제한에 걸리는
   곳입니다. 창을 닫으면 다음 단계로 넘어갑니다.
   - 클릭 = 점 추가, 드래그 = 화면 이동, 휠 = 확대/축소
2. **PyBullet 3D 창**이 뜨고 로봇이 그 점들을 따라갑니다. 우측 슬라이더로
   재생 속도와 위치를 조절합니다. 펜이 닿은 구간은 빨간 선, 들고 이동한
   구간은 회색 선으로 그려집니다.

### `scripts/ik_verify.py` — 수식 검사

수식이나 상수를 고쳤으면 **먼저 이걸 돌리세요.**

```bash
python scripts/ik_verify.py --no-gui
```

네 가지를 검사합니다.

| 검사 | 보는 것 | 통과 기준 |
|---|---|---|
| A. 영점 자세 | 모든 관절 0일 때의 자세 | pitch가 0에 가까움 → `PHI = 0` |
| B. 합 항등식 | $\theta_2+\theta_3+\theta_4$ 가 상수인가 | 편차 < 0.01° |
| C. IK → FK | 펜 끝이 목표점에 놓이는가 | 오차 < 0.01mm |
| D. FK → IK | 각도 → 위치 → 각도가 원래대로 돌아오는가 | 최대차 < 1° |

C는 오차의 **크기뿐 아니라 방향 벡터**도 찍습니다. 모든 목표점에서 같은 방향으로
같은 크기의 오차가 나오면 그건 계산 실수가 아니라 빠뜨린 평행이동입니다.
(→ [제약조건 (4)](#4-어깨-오프셋-12mm--현재-알려진-오차))

---

## 역기구학 수식

### 좌표계 약속

- 원점은 로봇 베이스, $z$ 축이 위쪽입니다.
- $\theta_1 = 0$ 일 때 팔이 **+y(12시) 방향**을 향합니다.
  그래서 시뮬레이터에 로봇을 놓을 때 z축으로 +90° 돌려 놓고
  (`BASE_YAW = π/2`), IK는 월드 방위에서 그만큼 빼서 관절값을 만듭니다
  (`TH1_OFFSET = -BASE_YAW`). **이 둘은 항상 부호가 반대여야 합니다.**
- 입력 $(x, y, z)$ 는 **펜 끝** 좌표입니다. 종이가 베이스 평면이면 종이에
  닿는 높이가 $z = 0$ 입니다.

### 치수

`omx/constants.py`에 있고, 전부 URDF의 `<origin xyz>` 와 1:1로 대응합니다.

| 기호 | 값 [m] | 뜻 | 출처 |
|---|---|---|---|
| $L_1$ | 0.0765 | 바닥 → 어깨 | `joint1.z`(0.017) + `joint2.z`(0.0595) |
| $a$ | 0.024 | link2 옆 꺾임 | `joint3` origin의 x |
| $b$ | 0.128 | link2 위 길이 | `joint3` origin의 z |
| $L_3$ | 0.124 | 팔꿈치 → 손목 | `joint4` origin의 x |
| $x_{\text{off}}$ | 0.012 | 어깨 기둥의 앞쪽 오프셋 | `joint1` origin의 x |
| $P$ | 0.110 | 손목 → 펜 축 | **자로 실측** |
| $E$ | 0.045 | 링크 축 → 펜 끝 | **자로 실측** |

link2가 꺾여 있어서 대각선 길이와 꺾인 각도를 따로 씁니다.

$$L_2 = \sqrt{a^2 + b^2} = 0.13023,
\qquad \beta = \operatorname{atan2}(a, b) = 10.62°,
\qquad \text{OFF} = \frac{\pi}{2} - \beta = 79.38°$$

### 푸는 순서

$\theta_1$ 은 수평 회전 하나로 끝나고, 나머지 셋은 $(r, z)$ 평면의 2링크
문제로 내려갑니다.

```
펜 끝 (x, y, z)
   │
   ├─ 수평 방위                              →  θ1                [수식 1]
   │
   └─ r = √(x²+y²) 로 평면 축소
        ↓  펜 끝 → 손목      rw = r − P,  zw = z + E
        ↓  어깨 기준으로      dr = rw,     dz = zw − L1
        ↓  D = √(dr² + dz²)                  ← 도달 판정은 여기서
        ↓  코사인 법칙                        →  ψ3
        ↓  어깨 각도                          →  ψ2
        └─ 영점 오프셋 + 부호 규약 적용       →  θ2, θ3, θ4     [수식 2~4]
```

펜이 링크에 **직각**으로 물려 있어서 $P$ 와 $E$ 가 서로 섞이지 않고
$r$ 성분과 $z$ 성분에 따로 들어갑니다. 이게 이 풀이가 간단해지는 이유입니다.

### 중간 단계

$$r = \sqrt{x^2+y^2}, \qquad r_w = r - P, \qquad z_w = z + E$$

$$d_r = r_w, \qquad d_z = z_w - L_1, \qquad D = \sqrt{d_r^2 + d_z^2}$$

$$\cos\alpha = \frac{L_2^2 + L_3^2 - D^2}{2 L_2 L_3},
\qquad \alpha = \arccos(\cos\alpha)$$

$$\psi_3 = -(\pi - \alpha) \qquad \text{(팔꿈치 위 분기)}$$

$$\psi_2 = \operatorname{atan2}(d_z, d_r)
- \operatorname{atan2}\big(L_3 \sin\psi_3, L_2 + L_3 \cos\psi_3\big)$$

$\psi_2, \psi_3$ 는 순수 기하각이지 관절각이 아닙니다. 여기에 영점 오프셋과
팀 부호 규약을 씌워야 관절각이 됩니다.

### 수식 4가지

$$\boxed{\theta_1 = s_1 \cdot \operatorname{atan2}(y, x) + c_1}$$

$$\boxed{\theta_2 = s \cdot (\psi_2 - \gamma)}$$

$$\boxed{\theta_3 = s \cdot (\psi_3 + \gamma)}$$

$$\boxed{\theta_4 = s \cdot (\Phi - \psi_2 - \psi_3)}$$

기호와 코드의 대응은 이렇습니다.

| 기호 | 코드 (`omx/constants.py`) | 값 | 뜻 |
|---|---|---|---|
| $s_1$ | `TH1_SIGN` | +1 | $\theta_1$ 의 회전 방향 |
| $c_1$ | `TH1_OFFSET` | $-\pi/2$ | 베이스를 돌려 놓은 만큼 되빼기. 항상 $-$`BASE_YAW` |
| $s$ | `SIGN_234` | −1 | 팀 규약이 안쪽 방향이므로 부호 반전 |
| $\gamma$ | `OFF` | 79.38° | 꺾인 링크 때문에 생기는 영점 오프셋. $\pi/2 - \beta$ |
| $\Phi$ | `PHI` | 0 | 공구 자세각. link5가 수평일 때 펜이 수직이므로 0 |

### 합 항등식

수식 2~4를 그냥 더하면 $\psi_2, \psi_3$ 가 전부 소거됩니다.

$$\theta_2 + \theta_3 + \theta_4
= s (\psi_2 - \gamma) + s (\psi_3 + \gamma) + s (\Phi - \psi_2 - \psi_3)
= s \Phi$$

**목표점이 어디든 이 합은 상수입니다.** 펜 자세가 변하지 않으니 당연한 결과인데,
손 계산으로도 확인할 수 있어서 가장 강력한 검사가 됩니다. 이 값이 흔들리면
`SIGN_234`, `OFF`, `PHI` 중 하나가 틀린 것입니다.

`ik_verify.py`의 검사 B가 바로 이것입니다.

---

## 제약조건

### (1) 관절 제한

URDF의 `<limit>` 을 파싱해서 씁니다. 손으로 베껴 적으면 URDF와 어긋나므로
코드에 숫자를 박아 두지 않습니다.

| 관절 | 하한 | 상한 |
|---|---|---|
| joint1 | −162.00° | +162.00° |
| joint2 | −102.60° | +90.00° |
| joint3 | −54.00° | +79.20° |
| joint4 | −102.60° | +117.00° |

IK가 푼 각도 중 하나라도 이 범위를 벗어나면 `None` 을 돌려줍니다.
`ik(..., apply_limits=False)` 로 부르면 무시하고 풀어서, **기하적으로 안 되는
것**과 **제한 때문에 안 되는 것**을 구분할 수 있습니다. `ik_viewer` 의 2D 창에서
회색 점이 후자입니다.

여기에 스위치 두 개를 얹을 수 있습니다 (`omx/constants.py`).

| 스위치 | 기본 | 켜면 |
|---|---|---|
| `JOINT1_FULL_TURN` | `False` | joint1을 ±180°로 넓힘 |
| `NO_BACKWARD_ARM` | `False` | joint2 하한을 $-\beta$ 로 올림 |

- **`JOINT1_FULL_TURN`** — URDF의 ±162°는 324°만 도는 것이라, 로봇 정반대
  방향에 36° 폭의 **사각지대**가 생깁니다. 실물이 한 바퀴 도는 것을 확인했다면
  켜서 사각지대를 없앨 수 있습니다.
- **`NO_BACKWARD_ARM`** — 상완이 수직을 넘어 뒤로 젖혀지지 않게 막습니다.
  상완각 $= (90° - \beta) - \theta_2$ 이므로 $\theta_2 = -\beta$ 가 정확히
  수직이고, 그보다 작으면 뒤로 꺾입니다. URDF 하한(−102.6°)을 그대로 쓰면
  팔이 완전히 뒤로 돌아간 자세도 통과합니다. 종이 위($z=0$) 작업 영역은
  이 제약을 걸어도 줄지 않습니다.

### (2) 도달 가능 조건

두 링크로 만든 삼각형이 성립해야 합니다.

$$|L_2 - L_3| \le D \le L_2 + L_3$$

$D$ 가 너무 멀면 팔이 안 닿고, 너무 가까우면 접어도 못 닿습니다.
**이 검사를 `arccos` 앞에서 반드시 해야 합니다.** 안 하면 $|\cos\alpha| > 1$ 로
`math domain error` 가 납니다.

통과한 뒤에도 부동소수 오차로 $\cos\alpha$ 가 아주 살짝 1을 넘을 수 있어서
$[-1, 1]$ 로 한 번 더 잘라 냅니다.

### (3) 팔꿈치 분기

$\alpha$ 가 정해져도 팔꿈치를 위로 꺾는 해와 아래로 꺾는 해, **두 개**가 있습니다.
`ELBOW_UP = True` 로 위쪽 하나만 씁니다.

**한 궤적 안에서는 반드시 하나로 통일해야 합니다.** 중간에 분기가 바뀌면
팔이 순간적으로 뒤집힙니다.

비슷한 이유로 $\theta_1$ 도 $2\pi$ 주기라 해가 여러 개입니다. 관절 제한 안에
드는 후보만 남기고, 직전 자세가 있으면 그중 **가장 덜 움직이는 것**을 고릅니다
(`ik(..., prev_q=...)`). 이걸 안 하면 각도가 $\pm\pi$ 경계를 넘을 때 관절이
반 바퀴 튑니다.

### (4) 어깨 오프셋 12mm — 현재 알려진 오차

> **이 저장소의 baseline은 이 오차를 안고 있습니다. 실물 검증 뒤 켜세요.**

URDF의 `joint1` origin이 `xyz="0.012 0 0.017"` 입니다. 즉 어깨 기둥이 베이스
중심축에서 **12mm 앞으로** 나가 있는데, 위 수식은 어깨가 중심축 위에 있다고
보고 풉니다.

이 12mm는 회전축(z)에 실려 도는 값이 아니라 **베이스 프레임에 고정된**
오프셋입니다. `BASE_YAW = +90°` 로 로봇을 돌려 놓았으므로 월드에서는
**+y 방향 12mm** 가 됩니다.

`ik_verify.py` 의 검사 C를 돌리면 이렇게 나옵니다.

```
  목표 (x, y, z)          오차 벡터 [mm]              크기
  (0.20,  0.00, 0.0)     ( +0.00, +12.00,  +0.00)    12.00
  (0.18,  0.06, 0.0)     ( +0.00, +12.00,  +0.00)    12.00
  (0.18, -0.06, 0.0)     ( +0.00, +12.00,  +0.00)    12.00
  (0.24,  0.00, 0.0)     ( -0.00, +12.00,  +0.00)    12.00
```

목표점이 어디든 **크기와 방향이 똑같습니다.** 계산이 틀린 게 아니라
평행이동 하나를 빠뜨린 것이라는 뜻입니다.

`omx/constants.py` 의 `APPLY_X_OFFSET` 을 `True` 로 켜면 목표점에서 이
오프셋을 먼저 빼고 풉니다. 시뮬레이터 기준으로 오차가 **12mm → 0.000mm**,
검사 D도 최대차 0.000°로 통과합니다.

```python
APPLY_X_OFFSET = True    # omx/constants.py
```

기본값을 `False` 로 둔 이유는 실물에서도 같은지 아직 확인하지 않았기
때문입니다. 실측으로 확인한 뒤 켜세요.

### 제약 요약

| # | 제약 | 어디서 걸리나 | 위반하면 |
|---|---|---|---|
| 1 | 관절 제한 | `limit_violations()` | `ik()` 가 `None` |
| 2 | $\|L_2-L_3\| \le D \le L_2+L_3$ | `solve_geometry()` | `ik()` 가 `None` |
| 3 | 팔꿈치 분기 통일 | `ELBOW_UP` | 궤적 중간에 팔이 뒤집힘 |
| 4 | 어깨 오프셋 12mm | `APPLY_X_OFFSET` | 펜 끝이 +y로 12mm 밀림 |

---

## 값을 고칠 때

1. **`omx/constants.py`** 만 고칩니다. 수식 자체를 바꾸는 게 아니라면
   다른 파일은 건드릴 일이 없습니다.
2. 고쳤으면 **바로 검사를 돌립니다.**

   ```bash
   python scripts/ik_verify.py --no-gui
   ```

3. B(합 항등식)와 C(펜 끝 오차)가 통과하면 `ik_viewer.py` 로 눈으로 확인합니다.

자주 고치게 되는 값은 이 둘입니다.

- **`P_LEN`, `E_LEN`** — 펜 홀더를 새로 만들 때마다 자로 재서 갱신. 이게
  틀리면 작업 영역 자체가 통째로 어긋납니다.
- **`PHI`** — 검사 A에서 영점 pitch가 0이 아니면 그 값을 넣습니다.

수식을 고칠 일이 생기면 `omx/kinematics.py` 하나만 보면 됩니다. `ik_viewer.py`
와 `ik_verify.py` 는 둘 다 이 모듈을 가져다 쓰므로, 한 군데를 고치면 양쪽에
같이 반영됩니다.

---

## 출처와 라이선스

- URDF와 메시는 ROBOTIS의 [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
  에서 가져왔습니다 (Apache License 2.0).
- URDF를 고쳤다면 어디를 왜 고쳤는지 커밋 메시지에 남겨 주세요. `omx/constants.py`
  의 치수는 URDF 값을 그대로 옮겨 적은 것이라, URDF가 바뀌면 같이 바뀌어야 합니다.
