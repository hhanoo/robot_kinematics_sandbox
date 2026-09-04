# robot_kinematics_sandbox<!-- omit from toc -->

**CAD(STL/DAE)와 DH 파라미터만으로 로봇을 직접 모델링하고, MoveIt 없이 구현한 FK / IK / 궤적 생성을 RViz로 검증하는 ROS 2 샌드박스**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docker/)

---

## 목차<!-- omit from toc -->

- [데모](#데모)
- [개요](#개요)
  - [프로젝트 목적](#프로젝트-목적)
  - [주요 구성요소](#주요-구성요소)
  - [적용 가능 영역](#적용-가능-영역)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [프로젝트 구조](#프로젝트-구조)
- [구현 상세](#구현-상세)
  - [robot_description](#robot_description)
  - [robot_kinematics](#robot_kinematics)
  - [robot_trajectory](#robot_trajectory)
  - [robot_bringup](#robot_bringup)
  - [robot_interfaces](#robot_interfaces)
  - [robot_control](#robot_control)
- [빠른 시작](#빠른-시작)
  - [Option 1: Docker (권장)](#option-1-docker-권장)
  - [Option 2: Native](#option-2-native)
- [시스템 요구사항](#시스템-요구사항)
  - [필수](#필수)
  - [소프트웨어 의존성](#소프트웨어-의존성)
- [설치](#설치)
  - [Method 1: Docker (권장)](#method-1-docker-권장)
  - [Method 2: Native](#method-2-native)
- [빌드](#빌드)
  - [전체 빌드](#전체-빌드)
  - [특정 패키지 빌드](#특정-패키지-빌드)
  - [클린 빌드](#클린-빌드)
- [실행](#실행)
  - [전체 시스템 실행 (권장)](#전체-시스템-실행-권장)
  - [개별 실행](#개별-실행)
  - [Docker Commands](#docker-commands)
- [사용법](#사용법)
  - [워크플로우](#워크플로우)
  - [1. 모델 확인](#1-모델-확인)
  - [2. 단위 테스트](#2-단위-테스트)
  - [3. 데모 재생](#3-데모-재생)
  - [4. 시퀀스 수정](#4-시퀀스-수정)
  - [5. 런타임 제어](#5-런타임-제어)
- [설정](#설정)
  - [Docker 설정 (`docker/config.sh`)](#docker-설정-dockerconfigsh)
  - [Launch 인자](#launch-인자)
- [API / 인터페이스](#api--인터페이스)
- [문제 해결](#문제-해결)
  - [1. RViz 창이 뜨지 않음](#1-rviz-창이-뜨지-않음)
  - [2. run.sh 실행 시 이미지 없음 오류](#2-runsh-실행-시-이미지-없음-오류)
  - [3. robot_kinematics 모듈 import 오류](#3-robot_kinematics-모듈-import-오류)
- [로드맵](#로드맵)
- [라이선스](#라이선스)
- [Maintainer](#maintainer)

---

## 데모

<!-- docs/demo.gif 캡처 추가 예정 -->

---

## 개요

### 프로젝트 목적

특정 로봇이나 플래너에 종속되지 않는 범용 로봇 기구학과 궤적 생성 실험용 기준 프로젝트로, CAD 파일과 DH 파라미터만 주어진 상황에서 ROS description 작성부터 기구학 알고리즘 구현과 검증까지의 전체 파이프라인을 다루는 구조임.  
현재 대상 로봇은 UR10e이며, FK / Jacobian / IK를 라이브러리에 의존하지 않고 numpy로 직접 구현해 원리를 학습하는 것이 목표임.

```
CAD (DAE) + DH parameters
  → URDF / xacro (custom description)
  → FK / Jacobian / DLS IK (custom implementation)
  → Joint / Cartesian trajectory
  → JointState streaming
  → RViz visualization
```

### 주요 구성요소

- **robot_description** (xacro): 표준 DH 파라미터를 URDF로 변환한 UR10e 모델과 DAE 메쉬, RViz 설정
- **robot_kinematics** (Python): DH 기반 FK / 기하학적 Jacobian / DLS 반복 IK로, ROS import 없는 순수 numpy 코어
- **robot_trajectory** (Python): 5차 다항식 관절 궤적과 직선과 원호 Cartesian 경로 생성으로, ROS import 없는 순수 numpy 코어
- **robot_bringup** (Python): 데모 시퀀스 빌더와 JointState 스트리밍 노드, RViz 런치
- **robot_interfaces** (srv): 런타임 제어 서비스 정의 (MoveJ / MoveL)로, 표준에 없는 pose 목표 서비스만 최소 정의
- **robot_control** (Python): 목표 pose를 서비스로 받아 IK, 궤적을 실행하는 motion_server로, 상태머신은 ROS 무관 순수 Python
- **docker** (Bash): ROS 2 Humble 개발 컨테이너 표준 구성 (build/run/commands)

### 적용 가능 영역

- 산업용 매니퓰레이터 기구학 검증
- CAD 기반 로봇 모델 초기 검증
- 커스텀 궤적 생성 알고리즘 테스트
- 연구 및 교육 목적의 로봇 시뮬레이션

---

## 주요 기능

**DH → URDF 모델링**: 표준 DH 한 행을 revolute + fixed 조인트 쌍으로 전개하는 xacro 매크로로 기구학과 메쉬 정렬을 분리

**DH 기반 FK**: 관절각 → base_link~tool0 동차변환. xacro 전개 URDF 체인과 무작위 관절각 100개 대조로 1e-6 이내 일치 검증

**기하학적 Jacobian**: 각 관절 축의 `[z × (pₑ−pᵢ); z]` 열로 구성한 6×6 행렬이며 수치미분(중앙차분)과 대조 검증

**DLS 반복 IK**: `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` 업데이트로 특이점에서도 발산하지 않는 수치 IK. 미수렴 시 예외 없이 `IKResult` 반환

**5차 다항식 관절 궤적**: 5차 다항식 rest-to-rest 프로파일을 관절 속도와 가속도 한계 기반으로 시간 파라미터화

**Cartesian 경로**: 위치 LERP + 자세 SLERP 직선, 자세 고정 원호 경로를 직전 해 시드 IK로 관절 점프 없는 연속 관절 경로로 변환

**RViz 데모 재생**: home 이동 → IK 목표 도달 → 직선 → 원 그리기 → 복귀 시퀀스를 50 Hz JointState로 스트리밍

**서비스 기반 런타임 제어**: `move_j`(IK 1회 + 관절 5차 다항식), `move_l`(직선 경로, 실행 전 전 waypoint IK) 서비스로 목표 pose를 받아 수락/거부를 즉시 응답하고, busy 중 새 목표는 거부하며 `stop`으로 즉시 정지

**테스트 기반 검증**: 순수 코어를 ROS 런타임 없이 pytest 61케이스로 검증 (FK 19 / 궤적 20 / 시퀀스 8 / 제어 14)

---

## 시스템 구조

```
   ┌─────────────────────────────────────────────────────────────┐
   │ Pure numpy cores: robot_kinematics (FK/Jacobian/DLS IK)     │
   │                   robot_trajectory (quintic/line/circle)    │
   └──────────────┬───────────────────────────────┬──────────────┘
                  │ import                        │ import
   ┌──────────────┴─────────────┐   ┌─────────────┴──────────────┐
   │ robot_bringup              │   │ robot_control              │
   │ demo_player                │   │ motion_server              │
   │ (pre-built demo sequence)  │   │ (move_j / move_l / stop)   │
   └──────────────┬─────────────┘   └─────────────┬──────────────┘
                  │                               │
                  └───────────────┬───────────────┘
                                  │ /joint_states (50 Hz)
                                  ▼
   ┌─────────────────────────────────────────────────────────────┐
   │ robot_state_publisher (URDF from robot_description xacro)   │
   └──────────────────────────────┬──────────────────────────────┘
                                  │ /tf
                                  ▼
   ┌─────────────────────────────────────────────────────────────┐
   │ RViz2 (view_robot.rviz / control.rviz)                      │
   └─────────────────────────────────────────────────────────────┘
```

**데이터 흐름**

[데모 재생] demo_sequence (FK/IK/궤적) → demo_player → /joint_states → robot_state_publisher → /tf → RViz  
[런타임 제어] move_j, move_l, stop 서비스 → motion_server → IK+궤적 → /joint_states, 상태는 /motion_state, /tool_pose로 발행  
[FK 검증] DH 테이블 → fk() ↔ xacro 전개 URDF 체인 (pytest 대조)  
[IK 검증] 무작위 q → fk() → solve_ik() → fk() 왕복 오차 (pytest 대조)

---

## 프로젝트 구조

```
robot_kinematics_sandbox/
├── src/
│   ├── robot_description/           # UR10e 모델 (xacro / 메쉬 / RViz)
│   │   ├── urdf/ur10e.urdf.xacro    # 표준 DH → URDF 변환 매크로
│   │   ├── meshes/                  # base_link.dae, link1~6.dae
│   │   ├── rviz/view_robot.rviz     # RViz 레이아웃
│   │   └── launch/view.launch.py    # 모델 뷰어 (joint_state_publisher_gui)
│   │
│   ├── robot_kinematics/            # 기구학 코어 (ROS import 없음)
│   │   ├── robot_kinematics/
│   │   │   ├── dh.py                # UR10e 표준 DH 테이블 + 링크 변환
│   │   │   ├── fk.py                # FK (base_link → tool0 프레임)
│   │   │   ├── jacobian.py          # 기하학적 Jacobian (6x6)
│   │   │   ├── ik.py                # DLS 반복 IK + rotation_vector
│   │   │   └── jog.py               # jog 한 스텝: twist → 관절 증분
│   │   └── test/                    # FK / Jacobian / IK / jog pytest (23)
│   │
│   ├── robot_trajectory/            # 궤적 생성 코어 (ROS import 없음)
│   │   ├── robot_trajectory/
│   │   │   ├── joint_traj.py        # 5차 다항식 프로파일 + 시간 파라미터화
│   │   │   └── cartesian_traj.py    # SLERP / 직선 / 원호 + 시드 IK 변환
│   │   └── test/                    # 궤적 pytest (20)
│   │
│   ├── robot_bringup/               # RViz 데모 실행 환경
│   │   ├── robot_bringup/
│   │   │   ├── demo_sequence.py     # 데모 시퀀스 빌더 (순수 numpy)
│   │   │   └── demo_player.py       # /joint_states 50 Hz 스트리밍 노드
│   │   ├── launch/demo.launch.py    # rsp + demo_player + RViz
│   │   └── test/                    # 시퀀스 pytest (8)
│   │
│   ├── robot_interfaces/            # 런타임 제어 srv 정의
│   │   └── srv/                     # MoveJ.srv, MoveL.srv (pose + duration)
│   │
│   └── robot_control/               # 런타임 제어 (서비스 goal 실행)
│       ├── robot_control/
│       │   ├── state_machine.py     # idle/moving/jog 전이 (순수 Python)
│       │   ├── conversions.py       # Pose ↔ 4x4 행렬 (Shepperd)
│       │   ├── backend.py           # SimBackend: /joint_states 발행 대행
│       │   ├── motion_server.py     # move_j/move_l/stop + 재생 타이머
│       │   ├── marker_server.py     # RViz 인터랙티브 마커 목표 지정
│       │   └── teleop_keyboard.py   # 키보드 jog (jog_twist 발행)
│       ├── launch/control.launch.py # rsp + motion_server + RViz
│       ├── rviz/control.rviz        # 제어용 RViz 레이아웃
│       └── test/                    # 상태머신과 변환 pytest (14)
│
├── docker/
│   ├── Dockerfile                   # ROS 2 Humble desktop + xacro/RViz/numpy
│   ├── build.sh                     # 이미지 빌드
│   ├── run.sh                       # 컨테이너 실행/재사용 (X11, 저장소 마운트)
│   ├── entrypoint.sh                # 종료 시 소유권 복원
│   ├── commands.sh                  # 컨테이너 내부 명령 (build, run-demo 등)
│   └── config.sh.example            # 이미지/컨테이너/도메인 설정 템플릿
│
├── docs/                            # 이론과 유도 문서 (코드와 1:1 대응)
│   ├── README.md                    # 인덱스, 읽는 순서, 표기 규약
│   ├── img/                         # 이론 문서 그림 (SVG)
│   ├── robot_description.md         # DH → URDF 변환
│   ├── robot_kinematics.md          # DH / FK / Jacobian / DLS IK
│   └── robot_trajectory.md          # 5차 다항식 / SLERP / pose 경로 / seed IK
│
└── README.md
```

---

## 구현 상세

> 각 알고리즘의 이론적 배경과 유도 과정은 [docs/](docs/)에 정리해 둠.

### robot_description

> 이론: [docs/robot_description.md](docs/robot_description.md)

- **urdf/ur10e.urdf.xacro**
  - 표준 DH 한 행 (θ, d, a, α)를 조인트 쌍으로 전개하는 `dh_revolute` 매크로가 핵심
  - revolute 조인트: `Tz(d)` 이동 + z축 회전(θ) / 뒤따르는 fixed 조인트: `Tx(a)·Rx(α)`
  - URDF 체인 전체가 표준 DH 곱과 동일해지는 구조
  - 기구학은 DH 값만으로 결정되고, 각 링크의 visual origin은 메쉬 정렬 전용이라 FK/IK에 영향 없음

### robot_kinematics

> 이론: [docs/robot_kinematics.md](docs/robot_kinematics.md)

- **dh.py**
  - UR10e 표준 DH 테이블 (a, d, α)와 링크 변환 `dh_transform = Rz(θ)·Tz(d)·Tx(a)·Rx(α)`
  - xacro 프로퍼티와 같은 값 유지가 규약이며, 어긋나면 FK-URDF 대조 테스트가 실패하도록 설계
- **fk.py**
  - `fk_frames(q)`: DH 행렬 누적 곱으로 base부터 각 관절까지 중간 프레임 (7, 4, 4) 반환하며 Jacobian 계산에 재사용
  - `fk(q)`: 마지막 프레임 (base_link → tool0)
  - 검증: 영점 자세 폐형식 `(a2+a3, −(d4+d6), d1−d5)` + xacro 전개 URDF 체인과 무작위 100 자세 대조 (오차 < 1e-6)
- **jacobian.py**
  - 열 i가 `[zᵢ × (pₑ−pᵢ); zᵢ]`인 기하학적 Jacobian (6×6)
  - 관절 i가 돌면 이후 링크 전체가 축 zᵢ로 회전하는 강체 → 손끝 속도 `v = ω × r` 원리를 열로 쌓은 것
  - 검증: 중앙차분 수치미분과 무작위 20 자세 대조, q=0 손목 특이점 rank 손실 확인
- **ik.py**
  - `rotation_vector(R)`: 로드리게스 역연산으로 자세 오차를 axis×angle 3-벡터로 압축
    - 비대칭 부분 → sinθ, 축, trace → cosθ, 180° 근처는 `(R+I)/2 = aaᵀ` 폴백으로 축 복원
  - `solve_ik`: DLS 업데이트 `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` + 스텝 상한 반복
    - λ² 항이 있어 특이점에서도 역행렬이 항상 존재해 발산하지 않는 구조
    - 미수렴 시 예외 대신 `IKResult(success, q, pos_error, rot_error, iterations)` 반환
  - 검증: FK→IK 왕복 (위치 < 1mm, 자세 < 0.1°), 도달 불가 목표의 정상 실패, 특이점 시작 안정성

### robot_trajectory

> 이론: [docs/robot_trajectory.md](docs/robot_trajectory.md)

- **joint_traj.py**
  - rest-to-rest 5차 다항식 프로파일 `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` (양끝 속도와 가속도 0)
  - 피크 속도 `15Δq/8T`, 피크 가속도 `10Δq/√3T²`에서 한계 만족 최소 시간을 관절별 역산 → 가장 느린 관절 기준 공통 duration
  - 위치, 속도, 가속도를 해석적으로 샘플링해 `JointTrajectory(t, q, qd, qdd)` 반환
- **cartesian_traj.py**
  - `slerp(R0, R1, s) = R0·exp(s·log(R0ᵀR1))`: rotation_vector(log)와 로드리게스 공식(exp)의 조합
  - `linear_pose_path`: 위치 LERP + 자세 SLERP 직선 경로
  - `circle_pose_path`: 축과 중심 기준 원호 경로 (자세 고정)
  - `cartesian_to_joint`: waypoint마다 직전 해를 시드로 IK를 풀어 관절 점프 없는 연속 경로 생성, 실패 시 waypoint 인덱스 보고

### robot_bringup

- **demo_sequence.py**
  - zero → home(5차 다항식) → IK 목표 pose(5차 다항식) → 직선 10 cm → 반지름 8 cm 원 한 바퀴 → home 복귀
  - 세그먼트 경계 중복 행을 제거하고 인덱스 범위(`Segment`)와 함께 dt 간격 관절 행렬 하나로 반환
  - Cartesian 구간 IK 실패 시 세그먼트명과 waypoint 번호를 담은 RuntimeError로 즉시 중단
- **demo_player.py**
  - 시작 시 시퀀스를 통째로 빌드해 IK 문제를 실행 전에 노출 (fail fast)
  - 타이머로 `/joint_states` 50 Hz 발행, 세그먼트 진입 로그, 종료 시 `loop` 파라미터에 따라 반복
- **launch/demo.launch.py**
  - xacro 전개 결과를 robot_state_publisher에 전달, demo_player, RViz 동시 기동
  - `use_rviz:=false`로 헤드리스 실행 지원

### robot_interfaces

- **srv/MoveJ.srv, srv/MoveL.srv** (동일 형태)
  - 요청 `geometry_msgs/Pose target` + `float64 duration` (0 = 속도와 가속도 한계 기반 최소 시간), 응답 `success` + `message`
  - pose 목표를 받는 표준 srv가 없어 이 빈틈만 최소로 정의하며, 상태와 명령 토픽은 전부 표준 메시지 사용

### robot_control

- **state_machine.py**
  - idle / moving / jog 전이만 담당하는 ROS 무관 클래스로, 시간을 float로 주입받아 pytest 단독 검증
  - moving 중 새 목표는 `busy: moving`으로 즉시 거부, jog는 deadman timeout(0.3 s)으로 idle 복귀
- **conversions.py**
  - quaternion ↔ 회전행렬 (Shepperd 방법) + `Pose` ↔ 4×4 동차변환
- **backend.py**
  - `SimBackend`: 관절 상태를 소유하고 `/joint_states` 발행을 대행하는 드라이버 대역으로, 이후 Gazebo/실로봇 전환 시 이 클래스만 교체되는 경계
- **motion_server.py**
  - `move_j`: IK 1회 + 관절 5차 다항식 / `move_l`: 직선 pose 경로를 실행 전에 전 waypoint IK로 검증 (fail fast)
  - 서비스는 수락/거부만 즉시 응답하고 완료는 `/motion_state`의 idle 복귀로 확인하는 규약
  - 매 tick `/joint_states`, `/tool_pose`(FK) 발행, `stop`은 현 위치 유지 후 idle
- **launch/control.launch.py**
  - robot_state_publisher + motion_server + RViz 동시 기동, `use_rviz:=false` 지원

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/robot_kinematics_sandbox.git
cd robot_kinematics_sandbox

# 2. 이미지 빌드 (config.sh는 example에서 자동 생성)
./docker/build.sh

# 3. 컨테이너 실행 (이미 실행 중이면 자동 attach)
./docker/run.sh

# 4. 컨테이너 내부에서 빌드 및 데모 실행
build
run-demo
```

### Option 2: Native

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/robot_kinematics_sandbox.git
cd robot_kinematics_sandbox

# 2. 빌드
colcon build --symlink-install
source install/setup.bash

# 3. 실행
ros2 launch robot_bringup demo.launch.py
```

---

## 시스템 요구사항

### 필수

| 항목   | 요구사항                    |
| ------ | --------------------------- |
| OS     | Ubuntu 22.04 LTS            |
| ROS 2  | Humble Hawksbill            |
| Python | 3.10 이상                   |
| Docker | 20.10 이상 (Docker 사용 시) |

### 소프트웨어 의존성

**ROS 2 패키지:**

- xacro
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

**Python 패키지:**

- numpy
- pytest (테스트)

---

## 설치

### Method 1: Docker (권장)

```bash
./docker/build.sh
./docker/run.sh
```

### Method 2: Native

#### 0. 프로젝트 루트로 이동

```bash
git clone https://github.com/hhanoo/robot_kinematics_sandbox.git
cd robot_kinematics_sandbox
```

#### 1. 시스템 의존성 설치

```bash
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui ros-humble-rviz2 \
    python3-numpy python3-pytest
```

#### 2. rosdep 의존성 설치

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 빌드

### 전체 빌드

```bash
colcon build --symlink-install
source install/setup.bash
```

### 특정 패키지 빌드

```bash
colcon build --symlink-install --packages-select robot_kinematics
```

### 클린 빌드

```bash
rm -rf build install log
colcon build --symlink-install
```

---

## 실행

### 전체 시스템 실행 (권장)

```bash
# 데모 시퀀스 재생
ros2 launch robot_bringup demo.launch.py

# 런타임 제어 (서비스로 목표 지정)
ros2 launch robot_control control.launch.py
```

### 개별 실행

디버깅 목적의 개별 노드 실행함.

```bash
# 모델 뷰어 (joint_state_publisher_gui 슬라이더로 관절 조작)
ros2 launch robot_description view.launch.py

# 데모 플레이어만 (RViz 없이)
ros2 launch robot_bringup demo.launch.py use_rviz:=false

# 노드 단독 실행
ros2 run robot_bringup demo_player
ros2 run robot_control motion_server
```

### Docker Commands

전체 command 정의는 [commands.sh](docker/commands.sh)를 참고하세요.

| Command           | 설명                                              | 참고                                                            |
| ----------------- | ------------------------------------------------- | --------------------------------------------------------------- |
| `build`           | `colcon build --symlink-install` + overlay source | -                                                               |
| `test-kinematics` | FK / Jacobian / IK 단위 테스트 (pytest)           | [robot_kinematics/test/](src/robot_kinematics/test/)            |
| `test-trajectory` | 궤적 생성 단위 테스트 (pytest)                    | [robot_trajectory/test/](src/robot_trajectory/test/)            |
| `test-control`    | 상태머신과 변환 단위 테스트 (pytest)              | [robot_control/test/](src/robot_control/test/)                  |
| `run-view`        | UR10e 모델 뷰어 (RViz + 슬라이더)                 | [view.launch.py](src/robot_description/launch/view.launch.py)   |
| `run-demo`        | FK/IK/궤적 데모 시퀀스 재생 (RViz)                | [demo.launch.py](src/robot_bringup/launch/demo.launch.py)       |
| `run-control`     | 런타임 제어 (motion_server + RViz)                | [control.launch.py](src/robot_control/launch/control.launch.py) |
| `source-config`   | `docker/config.sh` 재로드                         | -                                                               |
| `cmd-help`        | 명령 목록 출력 (셸 진입 시 자동 출력)             | -                                                               |

---

## 사용법

### 워크플로우

```
view model ──▶ unit tests ──▶ play demo ──▶ edit sequence ──▶ runtime control
    │              │              │               │                  │
 run-view   test-kinematics    run-demo    demo_sequence.py     run-control
            test-trajectory                                    + service call
              test-control
```

### 1. 모델 확인

```bash
run-view
```

joint_state_publisher_gui 슬라이더로 각 관절을 움직여 DH 기반 URDF와 메쉬 정렬을 확인함.

### 2. 단위 테스트

```bash
test-kinematics   # 19 cases: FK vs URDF 체인, Jacobian vs 수치미분, IK 왕복
test-trajectory   # 20 cases: 5차 다항식 경계조건/한계, 경로 기하, 관절 연속성
```

### 3. 데모 재생

```bash
build
run-demo
```

RViz에서 zero → home → IK 목표 → 직선 → 원 → home 순서의 시퀀스가 반복 재생되는 구조임.

### 4. 시퀀스 수정

[demo_sequence.py](src/robot_bringup/robot_bringup/demo_sequence.py) 상단 상수(HOME, LINE_OFFSET, CIRCLE_RADIUS, V_MAX 등)를 수정해 동작을 변경하며, `--symlink-install` 빌드라 재빌드 없이 재실행하면 반영됨.

### 5. 런타임 제어

```bash
run-control
```

별도 셸에서 현재 pose를 확인한 뒤 목표를 서비스로 지정하는데, orientation은 echo 값을 그대로 재사용하고 position만 옮김.

```bash
# 현재 tool0 pose 확인
ros2 topic echo /tool_pose --once

# 목표 pose로 이동 (duration 0 = 한계 기반 최소 시간)
ros2 service call /motion_server/move_j robot_interfaces/srv/MoveJ \
  "{target: {position: {x: -0.59, y: -0.17, z: 0.68}, orientation: {x: ..., y: ..., z: ..., w: ...}}, duration: 0.0}"

# 진행 상태 확인 / 즉시 정지
ros2 topic echo /motion_state
ros2 service call /motion_server/stop std_srvs/srv/Trigger
```

moving 중 새 목표는 `busy: moving`으로 거부되고, 도달 불가 목표는 `IK failed ...` 사유와 함께 시작 전에 거부되는 구조임.

---

## 설정

### Docker 설정 (`docker/config.sh`)

```bash
IMAGE_NAME="robot-kinematics-sandbox:latest"   # Docker 이미지 이름
CONTAINER_NAME="robot-kinematics-sandbox"      # 컨테이너 이름
ROS_DOMAIN_ID="98"                             # ROS 2 도메인 분리
XAUTHORITY_PATH="$HOME/.Xauthority"            # RViz X11 인증 경로
```

### Launch 인자

| 인자       | 기본값 | 대상 launch                        | 설명                |
| ---------- | ------ | ---------------------------------- | ------------------- |
| `use_rviz` | `true` | demo.launch.py / control.launch.py | RViz 동시 실행 여부 |

---

## API / 인터페이스

**ROS 2 인터페이스**

| 이름                    | 타입                              | 설명                                                  |
| ----------------------- | --------------------------------- | ----------------------------------------------------- |
| `/joint_states`         | Topic (sensor_msgs/JointState)    | 관절각 50 Hz 발행 (demo_player 또는 motion_server)    |
| `/motion_server/move_j` | Service (robot_interfaces/MoveJ)  | 목표 pose로 관절 5차 다항식 이동, 수락/거부 즉시 응답 |
| `/motion_server/move_l` | Service (robot_interfaces/MoveL)  | 목표 pose로 직선 이동, 실행 전 전 waypoint IK 검증    |
| `/motion_server/stop`   | Service (std_srvs/Trigger)        | 현 위치 즉시 정지                                     |
| `/motion_state`         | Topic (std_msgs/String)           | idle / moving / jog : 상태 변화 시 + 1 Hz             |
| `/tool_pose`            | Topic (geometry_msgs/PoseStamped) | 현재 tool0 FK 결과 (base_link 기준)                   |
| `demo_player.rate`      | Parameter (double)                | 발행 주기 [Hz], 시퀀스 샘플링 주기와 공유             |
| `demo_player.loop`      | Parameter (bool)                  | 시퀀스 종료 시 반복 여부                              |
| `motion_server.*`       | Parameter                         | rate / home / v_max / a_max / linear_speed            |

**라이브러리 API (순수 Python)**

| 이름                                  | 소속                            | 설명                                   |
| ------------------------------------- | ------------------------------- | -------------------------------------- |
| `fk(q)` / `fk_frames(q)`              | robot_kinematics.fk             | 관절각 → tool0 pose / 중간 프레임 전체 |
| `jacobian(q)`                         | robot_kinematics.jacobian       | 기하학적 Jacobian (6x6)                |
| `solve_ik(target, q0)`                | robot_kinematics.ik             | DLS 반복 IK → `IKResult`               |
| `quintic_joint_trajectory(q0, qf)`    | robot_trajectory.joint_traj     | 한계 기반 5차 다항식 궤적              |
| `linear_pose_path(T0, T1, n)`         | robot_trajectory.cartesian_traj | 직선 pose 경로 (LERP + SLERP)          |
| `circle_pose_path(T0, c, axis, a, n)` | robot_trajectory.cartesian_traj | 원호 pose 경로 (자세 고정)             |
| `cartesian_to_joint(poses, q_seed)`   | robot_trajectory.cartesian_traj | 시드 IK 연속 관절 경로 변환            |
| `build_demo_sequence(dt)`             | robot_bringup.demo_sequence     | 데모 전체 관절 시퀀스 생성             |
| `MotionStateMachine`                  | robot_control.state_machine     | idle/moving/jog 전이 (ROS 무관)        |
| `pose_to_matrix` / `matrix_to_pose`   | robot_control.conversions       | Pose ↔ 4×4 동차변환 (Shepperd)         |

**네트워크 구성**

| 항목          | 값   | 비고                        |
| ------------- | ---- | --------------------------- |
| ROS_DOMAIN_ID | `98` | `docker/config.sh`에서 변경 |

---

## 문제 해결

### 1. RViz 창이 뜨지 않음

증상:

```
qt.qpa.xcb: could not connect to display
```

해결:

```bash
# 호스트에서 X11 허용 후 컨테이너 재진입
xhost +local:docker
./docker/run.sh
```

> `docker/config.sh`의 `XAUTHORITY_PATH`가 실제 `~/.Xauthority` 경로와 일치하는지 확인함.

### 2. run.sh 실행 시 이미지 없음 오류

증상:

```
Error: Image robot-kinematics-sandbox:latest not found.
```

해결:

```bash
./docker/build.sh
```

### 3. robot_kinematics 모듈 import 오류

증상:

```
ModuleNotFoundError: No module named 'robot_kinematics'
```

해결:

- 컨테이너 내부에서 `build` 실행 후 새 셸로 재진입하거나 `source /ros2_ws/install/setup.bash`
- 소스 트리에서 pytest만 돌릴 때는 각 패키지 디렉토리에서 실행 (`test-kinematics` / `test-trajectory` 명령 권장)

---

## 로드맵

- [x] DH 기반 가상 6축 로봇 URDF 작성
- [x] STL 기반 UR10e 모델 정렬
- [x] 수치 IK (DLS) 구현
- [x] Cartesian trajectory → joint trajectory 변환
- [x] JointState 기반 RViz 재생
- [x] 서비스 기반 런타임 제어 (move_j / move_l / stop)
- [x] 인터랙티브 마커 목표 지정
- [ ] 키보드 텔레옵 (Cartesian jog)
- [ ] 캡슐 근사 자기충돌 검사
- [ ] Gazebo 연동 (ros2_control)
- [ ] MuJoCo 연동
- [ ] Isaac Sim 연동
- [ ] 실로봇 연동 인터페이스 정리

---

## 라이선스

이 프로젝트는 Apache-2.0 라이선스로 배포됩니다.

---

## Maintainer

**hhanoo** (woo980711@gmail.com)
