"""URDF 파일 위치 찾기.

저장소를 어디에 클론하든, 스크립트를 어느 폴더에서 실행하든
같은 URDF 를 가리키도록 저장소 루트를 기준으로 경로를 만든다.
"""

import os

# omx/paths.py -> omx/ -> 저장소 루트
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

URDF_REL = os.path.join(
    "open_manipulator_description", "urdf", "open_manipulator_robot.urdf"
)

MESH_DIR = os.path.join(REPO_ROOT, "open_manipulator_description", "meshes")


def resolve_urdf():
    """실제로 존재하는 URDF 의 절대경로를 반환.

    환경변수 OMX_URDF 가 있으면 그 값을 우선한다.
    (다른 URDF 로 실험할 때 코드를 안 고치고 바꿀 수 있게)
    """
    candidates = [
        os.environ.get("OMX_URDF", ""),          # 환경변수로 직접 지정
        os.path.join(REPO_ROOT, URDF_REL),       # 저장소 안 (기본)
        os.path.join(os.getcwd(), URDF_REL),     # 현재 작업 폴더
    ]
    for cand in candidates:
        if cand and os.path.isfile(cand):
            return os.path.abspath(cand)

    tried = "\n".join(f"    - {c}" for c in candidates if c)
    raise FileNotFoundError(
        "URDF 를 찾을 수 없습니다. 다음 경로를 확인했습니다:\n"
        f"{tried}\n"
        "  저장소를 통째로 클론했는지 확인하세요.\n"
        "  다른 위치의 URDF 를 쓰려면 환경변수 OMX_URDF 에 절대경로를 넣으세요."
    )


URDF_PATH = resolve_urdf()
