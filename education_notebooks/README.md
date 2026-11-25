# ROS2 자율주행 교육실습 노트북 (Standalone)

`ros3_ws` 프로젝트의 **실제 소스 코드(func_lib)** 를 그대로 불러와, ROS2/하드웨어 없이
샘플 이미지·가상 데이터만으로 자율주행 알고리즘을 학습하는 Jupyter 실습입니다.

## 구성
| 노트북 | 주제 | 사용 코드 |
|--------|------|----------|
| `00_시작하기.ipynb` | 환경 준비·샘플 데이터 확인 | — |
| `01_카메라인지.ipynb` | 차선 검출·BEV·신호등 HSV | `camera_perception_func_lib.py` |
| `02_판단_경로계획.ipynb` | CubicSpline 경로·조향각 | `path_planner_node.py`, `decision_making_function_lib.py` |
| `03_라이다인지.ipynb` | 장애물 감지·방향보정·안정화 | `lidar_perception_func_lib.py` |
| `04_시리얼_통합.ipynb` | 시리얼 프로토콜·전체 통합 | `protocol_convert_func_lib.py`, `motion_planner_node.py` |

각 노트북은 **개념 설명 → 실제 코드 실행 → 🎯 미션(직접 수정) → ▶ 정답 예시** 구조입니다.

## 실행 방법 (conda 권장)
```bash
# 1) environment.yml 로 전용 가상환경 생성 (이름: ros3_edu)
cd education_notebooks
conda env create -f environment.yml

# 2) 주피터 커널로 등록 (한 번만)
conda run -n ros3_edu python -m ipykernel install --user \
    --name ros3_edu --display-name "Python (ros3_edu)"

# 3) 환경 활성화 후 주피터 실행
conda activate ros3_edu
jupyter notebook
```
노트북을 연 뒤, 우측 상단 커널을 **"Python (ros3_edu)"** 로 선택하세요.
VS Code / Antigravity IDE에서는 `.ipynb`를 열고 커널 선택기에서 `ros3_edu` 를 고르면 됩니다.

> pip만 쓰려면: `pip install jupyter opencv-python numpy scipy matplotlib pyserial`

## 순서
`00 → 01 → 02 → 03 → 04` 순서로 진행하세요. 각 노트북 맨 위의 **[공통 설정]** 셀을 먼저 실행해야
프로젝트 코드와 샘플 데이터 경로가 잡힙니다.

## 요구 사항
- Python 3.8+
- 저장소의 `src/` 및 샘플 이미지(`.../Collected_Datasets/sample_dataset/`)가 원위치에 있어야 합니다.
