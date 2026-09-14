# robot_kinematics_sandbox<!-- omit from toc -->

**CAD(STL/DAE)와 DH 파라미터만으로 로봇을 직접 모델링하고, MoveIt 없이 구현한 FK / IK / 궤적 생성을 RViz로 검증하는 ROS 2 샌드박스**

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
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
  - [6. Gazebo 연동](#6-gazebo-연동)
  - [7. MuJoCo 연동](#7-mujoco-연동)
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
  → Runtime control (service / RViz marker / keyboard jog)
```

### 주요 구성요소

- **robot_description** (xacro): 표준 DH 파라미터를 URDF로 변환한 UR10e 모델과 DAE/STL 메쉬, 관성, RViz 설정
- **robot_kinematics** (Python): URDF 체인 기반 FK / 기하학적 Jacobian / DLS 반복 IK / jog 스텝 / 자기충돌 검사로, ROS import 없는 순수 numpy 코어
- **robot_trajectory** (Python): 5차 다항식 관절 궤적과 직선과 원호 Cartesian 경로 생성으로, ROS import 없는 순수 numpy 코어
- **robot_bringup** (Python): 데모 시퀀스 빌더와 JointState 스트리밍 노드, RViz 런치
- **robot_interfaces** (srv): 런타임 제어 서비스 정의 (MoveJ / MoveL)로, 표준에 없는 pose 목표 서비스만 최소 정의
- **robot_control** (Python): 목표 pose를 서비스로 받아 IK, 궤적을 실행하는 motion_server와 RViz 마커, 키보드 jog 클라이언트로, 상태머신은 ROS 무관 순수 Python
- **docker** (Bash): ROS 2 Jazzy + Gazebo Harmonic + MuJoCo 개발 컨테이너 표준 구성 (build/run/commands)

### 적용 가능 영역

- 산업용 매니퓰레이터 기구학 검증
- CAD 기반 로봇 모델 초기 검증
- 커스텀 궤적 생성 알고리즘 테스트
- 연구 및 교육 목적의 로봇 시뮬레이션

---

## 주요 기능

**DH → URDF 모델링**: 표준 DH 한 행을 revolute + fixed 조인트 쌍으로 전개하는 xacro 매크로로 기구학과 메쉬 정렬을 분리

**URDF 기반 FK**: URDF에서 읽은 체인으로 관절각 → base_link~tool0 동차변환을 계산하고 조인트 축도 URDF에 적힌 값을 그대로 쓰므로, 6축 로봇은 URDF 교체만으로 적용됨

**기하학적 Jacobian**: 각 관절 축의 `[z × (pₑ−pᵢ); z]` 열로 구성한 6×6 행렬이며 수치미분(중앙차분)과 대조 검증

**DLS 반복 IK**: `Δq = Jᵀ(JJᵀ + λ²I)⁻¹e` 업데이트로 특이점에서도 발산하지 않는 수치 IK로, 미수렴 시 예외 없이 `IKResult`를 반환

**5차 다항식 관절 궤적**: 5차 다항식 rest-to-rest 프로파일을 관절 속도와 가속도 한계 기반으로 시간 파라미터화

**Cartesian 경로**: 위치 LERP + 자세 SLERP 직선, 자세 고정 원호 경로를 직전 해 시드 IK로 관절 점프 없는 연속 관절 경로로 변환

**RViz 데모 재생**: home 이동 → IK 목표 도달 → 직선 → 원 그리기 → 복귀 시퀀스를 50 Hz JointState로 스트리밍

**서비스 기반 런타임 제어**: `move_j`(IK 1회 + 관절 5차 다항식), `move_l`(직선 경로, 실행 전 전 waypoint IK) 서비스로 목표 pose를 받아 수락/거부를 즉시 응답하고, busy 중 새 목표는 거부하며 `stop`으로 즉시 정지

**인터랙티브 마커 목표 지정**: RViz 6-DOF 마커를 드래그해 목표 pose를 놓고 우클릭 메뉴(MoveJ here, MoveL here, Reset to tool)로 실행하며, 드래그만으로는 로봇이 움직이지 않는 구조

**키보드 Cartesian jog**: 별도 셸의 텔레옵이 발행하는 base 프레임 twist를 DLS 한 스텝 `Δq = Jᵀ(JJᵀ + λ²I)⁻¹(v·dt)`로 적분하며, 키를 떼면 deadman timeout(0.3 s)으로 정지하고 특이점 근처에서는 발산 없이 느려짐

**캡슐 근사 자기충돌 검사**: URDF의 collision 메쉬에서 링크마다 캡슐 하나를 맞춰 선분 최단거리로 판정하며, move 궤적은 전 샘플을 미리 검사해 시작 전에 거부하고 jog는 충돌하는 스텝을 버리고 현재 자세를 유지

**테스트 기반 검증**: 순수 코어를 ROS 런타임 없이 pytest 78케이스로 검증 (기구학 35 / 궤적 20 / 시퀀스 9 / 제어 14)

---

## 시스템 구조

```
   ┌─────────────────────────────────────────────────────────────┐
   │ Pure numpy: robot_kinematics (chain/FK/Jacobian/IK/jog/col) │
   │             robot_trajectory (quintic/line/circle)          │
   └──────────────┬───────────────────────────────┬──────────────┘
                  │ import                        │ import
   ┌──────────────┴─────────────┐   ┌─────────────┴──────────────┐
   │ robot_bringup              │   │ robot_control              │
   │ demo_player                │   │ motion_server              │
   │ (pre-built demo sequence)  │   │ (move_j / move_l / stop,   │
   │                            │   │  jog_twist integration)    │
   │                            │   │   ▲ services / jog_twist   │
   │                            │   │ marker_server (RViz menu)  │
   │                            │   │ teleop_keyboard (raw tty)  │
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

```
 goal sources                                    state owner                            visualization

 demo_sequence ──▶ demo_player ──────────────────────────────────┐
                                                                 │
 service call ───── move_j/move_l/stop ─────┐                    │
 marker menu ──▶ marker_server ─────────────┼──▶ motion_server ──┴──▶ /joint_states ──▶ robot_state_publisher ──▶ /tf ──▶ RViz
 keyboard ──▶ teleop_keyboard ─ /jog_twist ─┘          │
                                                       └──▶ /motion_state, /tool_pose (status)
```

[FK 검증] xacro 전개 URDF → chain → fk() ↔ URDF 체인 직접 순회 (pytest 대조)  
[IK 검증] 무작위 q → fk() → solve_ik() → fk() 왕복 오차 (pytest 대조)

---

## 프로젝트 구조

```
robot_kinematics_sandbox/
├── src/
│   ├── robot_description/              # UR10e 모델 (xacro / 메쉬 / RViz)
│   │   ├── urdf/ur10e.urdf.xacro       # 표준 DH → URDF 변환 매크로
│   │   ├── meshes/visual/              # base.dae, shoulder~wrist3.dae
│   │   ├── meshes/collision/           # 감면 STL 7개 (visual과 같은 origin)
│   │   ├── rviz/view_robot.rviz        # RViz 레이아웃
│   │   └── launch/view_robot.launch.py # 모델 뷰어 (joint_state_publisher_gui)
│   │
│   ├── robot_kinematics/               # 기구학 코어 (ROS import 없음)
│   │   ├── robot_kinematics/
│   │   │   ├── chain.py                # URDF → 체인 (조인트 축 / 링크 / 한계)
│   │   │   ├── dh.py                   # 표준 DH 한 행의 링크 변환 (xacro 대조용)
│   │   │   ├── fk.py                   # FK (base_link → tool0 프레임)
│   │   │   ├── jacobian.py             # 기하학적 Jacobian (6x6)
│   │   │   ├── ik.py                   # DLS 반복 IK + rotation_vector
│   │   │   ├── jog.py                  # jog 한 스텝: twist → 관절 증분
│   │   │   └── collision.py            # 캡슐 자기충돌 검사 (메쉬에서 피팅)
│   │   └── test/                       # FK / Jacobian / IK / jog / 충돌 pytest (35)
│   │
│   ├── robot_trajectory/               # 궤적 생성 코어 (ROS import 없음)
│   │   ├── robot_trajectory/
│   │   │   ├── joint_traj.py           # 5차 다항식 프로파일 + 시간 파라미터화
│   │   │   └── cartesian_traj.py       # SLERP / 직선 / 원호 + 시드 IK 변환
│   │   └── test/                       # 궤적 pytest (20)
│   │
│   ├── robot_bringup/                  # RViz 데모 실행 환경
│   │   ├── robot_bringup/
│   │   │   ├── demo_sequence.py        # 데모 시퀀스 빌더 (순수 numpy)
│   │   │   └── demo_player.py          # /joint_states 50 Hz 스트리밍 노드
│   │   ├── launch/demo.launch.py       # rsp + demo_player + RViz
│   │   └── test/                       # 시퀀스 pytest (9)
│   │
│   ├── robot_interfaces/               # 런타임 제어 srv 정의
│   │   └── srv/                        # MoveJ.srv, MoveL.srv (pose + duration)
│   │
│   └── robot_control/                  # 런타임 제어 (서비스 goal 실행)
│       ├── robot_control/
│       │   ├── state_machine.py        # idle/moving/jog 전이 (순수 Python)
│       │   ├── conversions.py          # Pose ↔ 4x4 행렬 (Shepperd)
│       │   ├── backend.py              # Sim/Gazebo 백엔드: 관절 상태 입출력 경계
│       │   ├── mujoco_model.py         # URDF → MuJoCo 모델 (MjSpec)
│       │   ├── motion_server.py        # move_j/move_l/stop + 재생 타이머
│       │   ├── marker_server.py        # RViz 인터랙티브 마커 목표 지정
│       │   └── teleop_keyboard.py      # 키보드 jog (jog_twist 발행)
│       ├── launch/control.launch.py    # rsp + motion_server + marker_server + RViz
│       ├── launch/gazebo.launch.py     # Gazebo + ros2_control + motion_server
│       ├── launch/mujoco.launch.py     # MuJoCo 물리 (노드 내장)
│       ├── config/gazebo_controllers.yaml   # Gazebo: 브로드캐스터 + 위치 컨트롤러
│       ├── config/mujoco_model.yaml    # MuJoCo: armature, 서보 게인, 중력 보상
│       ├── rviz/control.rviz           # 제어용 RViz 레이아웃
│       └── test/                       # 상태머신과 변환 pytest (14)
│
├── docker/
│   ├── Dockerfile                      # ROS 2 Jazzy desktop + ros_gz/ros2_control/mujoco
│   ├── build.sh                        # 이미지 빌드
│   ├── run.sh                          # 컨테이너 실행/재사용 (X11, 저장소 마운트)
│   ├── entrypoint.sh                   # 종료 시 소유권 복원
│   ├── commands.sh                     # 컨테이너 내부 명령 (build, run-demo 등)
│   └── config.sh.example               # 이미지/컨테이너/도메인 설정 템플릿
│
├── docs/                               # 이론과 유도 문서 (코드와 1:1 대응)
│   ├── README.md                       # 인덱스, 읽는 순서, 표기 규약
│   ├── img/                            # 이론 문서 그림 (SVG)
│   ├── robot_description.md            # DH → URDF 변환
│   ├── robot_kinematics.md             # DH / FK / Jacobian / DLS IK
│   └── robot_trajectory.md             # 5차 다항식 / SLERP / pose 경로 / seed IK
│
└── README.md
```

---

## 구현 상세

패키지마다 소개와 파일 목록을 두며, 파일은 코어(ROS 런타임 없이 pytest로 검증되는 모델과 순수 Python 함수), ROS(노드, launch, srv 정의), 검증(pytest)으로 나눔.  
유도와 수식은 [docs/](docs/)에 있으며, 코어 항목 끝의 괄호가 해당 절로 이어짐.

### robot_description

> CAD 메쉬와 표준 DH 파라미터만으로 UR10e를 URDF로 모델링하는 패키지임.  
> DH 값이 기구학을 결정하고 visual과 collision origin은 메쉬 정렬 전용이라 FK와 IK에 영향이 없으며, 뷰어 launch로 정렬을 눈으로 확인함.

- **코어** (이론은 [docs/robot_description.md](docs/robot_description.md))
  - **[ur10e.urdf.xacro](src/robot_description/urdf/ur10e.urdf.xacro)** : `dh_revolute`와 `link_geometry` 매크로, DH 한 행을 revolute + fixed 조인트 쌍으로 전개해 URDF 체인이 표준 DH 곱과 같아지게 하고 링크마다 visual과 collision을 같은 origin으로 붙임 ([2\_해법: 조인트 하나를 둘로 분해](docs/robot_description.md#2-해법-조인트-하나를-둘로-분해))
  - `cylinder_inertial` 매크로가 UR 공식 질량으로 원통 근사 관성을 채우는데, 관성이 없는 링크는 SDF 변환에서 통째로 사라져 Gazebo가 모델을 만들지 못하기 때문임
  - `sim_gazebo:=true`일 때만 world 고정 조인트와 ros2_control, Gazebo 플러그인 블록을 전개하며, 컨트롤러 YAML 경로는 `simulation_controllers` 인자로 받음
  - **[meshes/](src/robot_description/meshes/)** : visual DAE 7개와 collision STL 7개, 같은 프레임에 놓여 있어 origin을 공유하고 collision은 삼각형을 138개에서 1,874개로 줄인 감면 메쉬임
- **ROS**
  - **[view_robot.launch.py](src/robot_description/launch/view_robot.launch.py)** : robot_state_publisher, joint_state_publisher_gui, RViz 동시 기동, 슬라이더로 관절을 움직여 URDF와 메쉬 정렬을 확인하는 뷰어 (`run-view`)

### robot_kinematics

> URDF에서 읽은 체인으로 FK, Jacobian, DLS IK, jog 한 스텝을 numpy만으로 구현한 코어 패키지임.  
> ROS import가 없어 pytest 단독 검증이 가능하고, 모든 함수는 `chain` 인자를 생략하면 번들된 UR10e 기술서를 사용하므로 다른 6축 로봇은 URDF만 바꾸면 됨.

- **코어** (이론은 [docs/robot_kinematics.md](docs/robot_kinematics.md))
  - **[chain.py](src/robot_kinematics/robot_kinematics/chain.py)** : `Chain`, `Segment`, `from_urdf(urdf_xml)`, `load_default()`, base에서 tip까지의 경로를 움직이는 조인트 하나씩으로 줄이고 고정 조인트를 앞뒤 변환에 접어 넣어 조인트 이름과 축, 링크 이름, 한계를 함께 담음
  - **[dh.py](src/robot_kinematics/robot_kinematics/dh.py)** : `dh_transform(θ, d, a, α)`, 표준 DH 한 행의 4×4 링크 변환으로 xacro의 `dh_revolute` 매크로와 같은 계산이며 런타임 경로에는 쓰이지 않음 ([1_DH 파라미터](docs/robot_kinematics.md#1-dh-파라미터-dhpy))
  - **[fk.py](src/robot_kinematics/robot_kinematics/fk.py)** : `fk_joints(q)`, `fk_frames(q)`, `fk(q)`, 체인을 한 번 순회해 링크 프레임과 관절 축, 축 위의 점, tool0 pose 계산 ([2_FK](docs/robot_kinematics.md#2-fk-fkpy))
  - **[jacobian.py](src/robot_kinematics/robot_kinematics/jacobian.py)** : `jacobian(q)`, URDF에 적힌 관절 축으로 관절 속도를 tool0 선속도와 각속도로 잇는 6×6 행렬 ([3_Jacobian](docs/robot_kinematics.md#3-jacobian-jacobianpy))
  - **[ik.py](src/robot_kinematics/robot_kinematics/ik.py)** : `rotation_vector(R)`, `solve_ik(target, q0)`, `IKResult`, 목표 pose를 DLS 반복으로 관절각으로 풀고 미수렴 시 `IKResult`로 보고 ([4_IK](docs/robot_kinematics.md#4-ik-ikpy))
  - **[jog.py](src/robot_kinematics/robot_kinematics/jog.py)** : `jog_step(q, twist, dt)`, twist를 dt 동안 적분한 관절 증분을 DLS 한 스텝으로 계산 ([4.3\_뉴턴법에서 DLS로 확장](docs/robot_kinematics.md#43-뉴턴법에서-dls로-확장))
  - **[collision.py](src/robot_kinematics/robot_kinematics/collision.py)** : `load_model()`, `self_collision_pairs(q)`, `check_self_collision(q)`, URDF의 collision 메쉬에서 링크별 캡슐을 맞추고 선분 최단거리로 자기충돌을 판정 (사용처 motion_server)
- **검증**
  - **[test/](src/robot_kinematics/test/)** : test_fk, test_jacobian, test_ik, test_jog, test_collision 35건, URDF 체인 대조, 수치미분 대조, IK 왕복과 실패 보고, jog 방향과 특이점, 캡슐의 메쉬 포함과 충돌 판정 (`test-kinematics`)

### robot_trajectory

> 관절 공간의 5차 다항식 궤적과 Cartesian 공간의 직선, 원호 경로를 만들고, pose 경로를 seed IK로 관절 경로로 바꾸는 코어 패키지임.  
> robot_kinematics만 import하며 ROS 의존이 없음.

- **코어** (이론은 [docs/robot_trajectory.md](docs/robot_trajectory.md))
  - **[joint_traj.py](src/robot_trajectory/robot_trajectory/joint_traj.py)** : `min_duration`, `quintic_joint_trajectory(q0, qf)`, `JointTrajectory`, 두 관절각 사이의 5차 다항식 궤적과 한계 기반 최소 시간 ([1_5차 다항식 궤적](docs/robot_trajectory.md#1-5차-다항식-궤적-joint_trajpy))
  - **[cartesian_traj.py](src/robot_trajectory/robot_trajectory/cartesian_traj.py)** : SO(3) interpolation, pose 경로, seed IK 변환
    - `slerp(R0, R1, s)` : 두 회전행렬 사이의 geodesic interpolation ([2_SO(3) interpolation](docs/robot_trajectory.md#2-so3-interpolation-cartesian_trajpy))
    - `linear_pose_path`, `circle_pose_path` : 직선(LERP + SLERP)과 원호(자세 고정) pose 경로 생성 ([3_Pose 경로](docs/robot_trajectory.md#3-pose-경로))
    - `cartesian_to_joint(poses, q_seed)`, `CartesianJointPath` : pose 경로를 seed IK로 관절 경로로 변환, 실패 waypoint 보고 ([4\_경로에서 관절로](docs/robot_trajectory.md#4-경로에서-관절로))
- **검증**
  - **[test/](src/robot_trajectory/test/)** : test_joint_traj, test_cartesian_traj 20건, 경계조건과 한계, SLERP 성질, 경로 기하, 관절 연속성 (`test-trajectory`)

### robot_bringup

> 1단계 데모의 조립 계층으로, 코어 함수로 관절 시퀀스를 미리 만들고 이를 `/joint_states`로 재생해 RViz에 보여 줌.  
> 목표를 받아 움직이는 제어는 하지 않으며 그 역할은 robot_control이 맡음.

- **코어**
  - **[demo_sequence.py](src/robot_bringup/robot_bringup/demo_sequence.py)** : `build_demo_sequence(dt)`, `Segment`, `DemoSequence`, zero, home, IK 목표, 직선, 원, home 복귀 순서의 관절 시퀀스를 사전 생성 (호출 `fk`, `solve_ik`, `quintic_joint_trajectory`, `linear_pose_path`, `circle_pose_path`, `cartesian_to_joint`)
- **ROS**
  - **[demo_player.py](src/robot_bringup/robot_bringup/demo_player.py)** : 노드 `demo_player`, 시퀀스를 한 행씩 `/joint_states`로 발행하고 끝나면 `loop`에 따라 반복 (토픽 `/joint_states`, 파라미터 `rate`, `loop`)
  - **[demo.launch.py](src/robot_bringup/launch/demo.launch.py)** : robot_state_publisher, demo_player, RViz 동시 기동, `use_rviz:=false` 헤드리스 지원 (`run-demo`)
- **검증**
  - **[test/](src/robot_bringup/test/)** : test_demo_sequence 9건, 시작과 끝 자세, 관절 연속성, 세그먼트 범위, 직선과 원 기하, 전 구간 무충돌 (`test-bringup`)

### robot_interfaces

> 런타임 제어에 필요한 srv 정의만 담은 ament_cmake 패키지임.  
> pose 목표를 받는 표준 srv가 없어 이 빈틈만 정의하고, 상태와 명령 토픽은 전부 표준 메시지를 사용함.

- **ROS**
  - **[srv/MoveJ.srv, srv/MoveL.srv](src/robot_interfaces/srv/)** : 요청 `Pose target`, `float64 duration`, 응답 `bool success`, `string message`, 목표 pose와 duration 요청에 수락 여부와 사유를 즉시 응답 (`move_j`, `move_l`)

### robot_control

> 2단계 런타임 제어 계층으로, motion_server 하나가 관절 상태를 소유하고 서비스, 마커, 키보드에서 온 목표를 실행함.  
> 드라이버 흉내는 SimBackend 한 클래스에 가둬 Gazebo나 실로봇 전환 시 backend만 바꾸며, 상태머신과 변환은 ROS 무관 순수 Python이라 pytest로 검증함.

- **코어**
  - **[state_machine.py](src/robot_control/robot_control/state_machine.py)** : `MotionStateMachine`, idle, moving, jog 전이와 busy 거부, deadman timeout 판정 (사용처 motion_server)
  - **[conversions.py](src/robot_control/robot_control/conversions.py)** : `pose_to_matrix`, `matrix_to_pose`, Pose와 4×4 동차변환 상호 변환 (Shepperd)
- **ROS**
  - **[backend.py](src/robot_control/robot_control/backend.py)** : `SimBackend`와 `GazeboBackend`, `MujocoBackend`, 관절 상태를 어디서 읽고 명령을 어디로 보낼지만 담당하는 교체 경계 (토픽 `/joint_states`, `/joint_position_controller/commands`)
  - **[mujoco_model.py](src/robot_control/robot_control/mujoco_model.py)** : 같은 URDF를 MjSpec으로 읽어 armature와 gravcomp, 위치 구동기를 채운 MuJoCo 모델을 만듦, MJCF로 저장했다 다시 읽으면 `<inertial>`이 전부 사라져 메쉬 부피로 질량이 재계산되므로 텍스트를 거치지 않음
    - visual 메쉬는 `meshes/visual_mujoco/`의 재질별 OBJ를 collision geom과 같은 origin에 얹고 색은 나란한 `materials.mtl`에서 읽으며, 로봇이 바뀌면 팔레트도 함께 바뀜
    - collision은 group 3으로 숨겨 두어 뷰어에서 `3` 키로 켜서 확인함
  - **[motion_server.py](src/robot_control/robot_control/motion_server.py)** : 노드 `motion_server`, 서비스 목표를 수락 시점에 궤적으로 생성해 자기충돌까지 검사한 뒤 재생, jog 적분과 충돌 스텝 폐기, 상태 발행
    - 서비스 : `move_j`, `move_l`, `stop`
    - 토픽 : `/jog_twist`, `/motion_state`, `/tool_pose`
    - 호출 : `solve_ik`, `quintic_joint_trajectory`, `linear_pose_path`, `cartesian_to_joint`, `jog_step`, `fk`, `self_collision_pairs`, `check_self_collision`
  - **[marker_server.py](src/robot_control/robot_control/marker_server.py)** : 노드 `marker_server`, 마커 우클릭 메뉴를 서비스 호출로 전달하며 드래그만으로는 미동작 (서비스 클라이언트 `move_j`, `move_l`, 토픽 `/tool_pose`)
  - **[teleop_keyboard.py](src/robot_control/robot_control/teleop_keyboard.py)** : 노드 `teleop_keyboard`, 키 입력을 base 프레임 twist로 발행하고 정지는 서버 deadman에 위임 (`run-teleop`, 토픽 `/jog_twist`)
  - **[control.launch.py](src/robot_control/launch/control.launch.py)** : robot_state_publisher, motion_server, marker_server, RViz 동시 기동, `use_rviz:=false` 헤드리스 지원 (`run-control`)
  - **[gazebo.launch.py](src/robot_control/launch/gazebo.launch.py)** : Gazebo 기동 후 모델 스폰, 컨트롤러 활성화, motion_server 순서로 단계 실행, `headless:=true` 서버 전용 지원 (`run-gazebo`)
  - **[mujoco.launch.py](src/robot_control/launch/mujoco.launch.py)** : MuJoCo가 노드 안에서 돌아 스폰도 컨트롤러도 없고 control.launch.py와 노드 구성이 같음 (`run-mujoco`)
  - **[config/gazebo_controllers.yaml](src/robot_control/config/gazebo_controllers.yaml)** : joint_state_broadcaster와 JointGroupPositionController 설정, `update_rate`는 motion_server의 50 Hz tick과 일치
  - **[config/mujoco_model.yaml](src/robot_control/config/mujoco_model.yaml)** : armature와 서보 게인, 중력 보상 여부, 로봇 크기에 따라 달라지는 값이라 URDF를 바꾸면 함께 조정해야 함
- **검증**
  - **[test/](src/robot_control/test/)** : test_state_machine, test_conversions 14건, 상태머신 전이와 busy 거부, deadman, 변환 왕복 (`test-control`)

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
| OS     | Ubuntu 24.04 LTS            |
| ROS 2  | Jazzy                       |
| Python | 3.12 이상                   |
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
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
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

# 런타임 제어 (motion_server + 목표 마커 + RViz, 서비스나 마커로 목표 지정)
ros2 launch robot_control control.launch.py
```

### 개별 실행

디버깅 목적의 개별 노드 실행함.

```bash
# 모델 뷰어 (joint_state_publisher_gui 슬라이더로 관절 조작)
ros2 launch robot_description view_robot.launch.py

# 데모 플레이어만 (RViz 없이)
ros2 launch robot_bringup demo.launch.py use_rviz:=false

# 노드 단독 실행
ros2 run robot_bringup demo_player
ros2 run robot_control motion_server
ros2 run robot_control marker_server

# 키보드 jog (raw 터미널이 필요해 launch에 넣지 않으며, run-control과 별도 셸에서 실행)
ros2 run robot_control teleop_keyboard
```

### Docker Commands

전체 command 정의는 [commands.sh](docker/commands.sh)를 참고하세요.

| Command           | 설명                                              | 참고                                                                      |
| ----------------- | ------------------------------------------------- | ------------------------------------------------------------------------- |
| `build`           | `colcon build --symlink-install` + overlay source | -                                                                         |
| `test-kinematics` | FK / Jacobian / IK / jog 단위 테스트 (pytest)     | [robot_kinematics/test/](src/robot_kinematics/test/)                      |
| `test-trajectory` | 궤적 생성 단위 테스트 (pytest)                    | [robot_trajectory/test/](src/robot_trajectory/test/)                      |
| `test-bringup`    | 데모 시퀀스 단위 테스트 (pytest)                  | [robot_bringup/test/](src/robot_bringup/test/)                            |
| `test-control`    | 상태머신과 변환 단위 테스트 (pytest)              | [robot_control/test/](src/robot_control/test/)                            |
| `run-view`        | UR10e 모델 뷰어 (RViz + 슬라이더)                 | [view_robot.launch.py](src/robot_description/launch/view_robot.launch.py) |
| `run-demo`        | FK/IK/궤적 데모 시퀀스 재생 (RViz)                | [demo.launch.py](src/robot_bringup/launch/demo.launch.py)                 |
| `run-control`     | 런타임 제어 (motion_server + 목표 마커 + RViz)    | [control.launch.py](src/robot_control/launch/control.launch.py)           |
| `run-teleop`      | 키보드 Cartesian jog (별도 셸, `/jog_twist` 발행) | [teleop_keyboard.py](src/robot_control/robot_control/teleop_keyboard.py)  |
| `run-gazebo`      | Gazebo 물리 + ros2_control + motion_server (RViz) | [gazebo.launch.py](src/robot_control/launch/gazebo.launch.py)             |
| `run-mujoco`      | MuJoCo 물리를 motion_server 안에서 구동 (RViz)    | [mujoco.launch.py](src/robot_control/launch/mujoco.launch.py)             |
| `source-config`   | `docker/config.sh` 재로드                         | -                                                                         |
| `cmd-help`        | 명령 목록 출력 (셸 진입 시 자동 출력)             | -                                                                         |

---

## 사용법

### 워크플로우

```
view model ──▶ unit tests ──▶ play demo ──▶ edit sequence ──▶ runtime control ──▶ physics
    │              │              │               │                  │              │
 run-view   test-kinematics    run-demo    demo_sequence.py     run-control    run-gazebo
            test-trajectory                                    + service call    run-mujoco
              test-control                                     + RViz marker
                                                               + run-teleop
```

### 1. 모델 확인

```bash
run-view
```

joint_state_publisher_gui 슬라이더로 각 관절을 움직여 DH 기반 URDF와 메쉬 정렬을 확인함.

### 2. 단위 테스트

```bash
test-kinematics   # 35 cases: FK vs URDF 체인, Jacobian vs 수치미분, IK 왕복, jog 스텝, 자기충돌
test-trajectory   # 20 cases: 5차 다항식 경계조건/한계, 경로 기하, 관절 연속성
test-bringup      # 9 cases: 시작과 끝 자세, 관절 연속성, 세그먼트 범위, 직선과 원 기하, 전 구간 무충돌
test-control      # 14 cases: 상태머신 전이와 busy 거부, deadman timeout, Pose 변환 왕복
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

motion_server, 목표 마커, RViz가 함께 뜨며, 목표는 아래 3가지 방법 중 하나로 지정함.  
moving 중 새 목표는 `busy: moving`으로, 도달 불가 목표는 `IK failed ...`로, 자기충돌이 생기는 궤적은 `self-collision at sample ...`로 시작 전에 거부되는 구조임.

#### 5.1 서비스 호출

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

#### 5.2 인터랙티브 마커

RViz의 파란 구 마커를 드래그해 목표 pose를 놓고, 구를 우클릭해 메뉴 항목을 선택하면 marker_server가 서비스를 호출함.  
드래그만으로는 로봇이 움직이지 않으며, 수락과 거부 사유는 `run-control` 셸의 marker_server 로그에 출력됨.

| 메뉴 항목       | 동작                                   |
| --------------- | -------------------------------------- |
| `MoveJ here`    | 마커 pose로 `move_j` 호출 (duration 0) |
| `MoveL here`    | 마커 pose로 `move_l` 호출 (duration 0) |
| `Reset to tool` | 마커를 현재 `/tool_pose` 위치로 재정렬 |

#### 5.3 키보드 jog

```bash
run-teleop
```

`run-control`과 별도 셸에서 실행하며, 키를 누르는 동안 base 프레임 twist가 `/jog_twist`로 발행되고 motion_server가 매 tick DLS 한 스텝으로 적분함.  
키 릴리즈는 터미널에서 감지할 수 없어 마지막 twist로부터 0.3 s가 지나면 deadman timeout으로 idle에 복귀하는 구조이며, moving 중 키 입력은 무시됨.  
특이점이나 작업공간 경계에 가까워지면 DLS 감쇠로 발산 없이 느려지고, 자기충돌로 들어가는 스텝은 버려 현재 자세를 유지하며 1초에 1번 경고를 남김.

| 키        | 동작                      | 키        | 동작                 |
| --------- | ------------------------- | --------- | -------------------- |
| `w` / `s` | +x / −x                   | `u` / `o` | +rx / −rx            |
| `a` / `d` | +y / −y                   | `i` / `k` | +ry / −ry            |
| `r` / `f` | +z / −z                   | `j` / `l` | +rz / −rz            |
| `+` / `-` | 속도 스케일 ×1.25 / ÷1.25 | `ESC`     | 종료 (Ctrl-C도 가능) |

기본 속도는 0.1 m/s와 0.5 rad/s이며, motion_server는 수신 twist를 `jog_max_linear`(0.25 m/s)와 `jog_max_angular`(1.0 rad/s)로 클램프함.

### 6. Gazebo 연동

```bash
run-gazebo
```

같은 motion_server를 Gazebo 물리 위에서 구동하는데, `backend` 파라미터가 `gazebo`이면 관절 상태를 직접 적분하지 않고 joint_state_broadcaster가 발행한 측정값을 읽고 명령은 위치 컨트롤러로 보냄.  
런치는 Gazebo 기동, 모델 스폰, joint_state_broadcaster, joint_position_controller, motion_server 순으로 앞 단계가 끝난 뒤 다음을 실행하는데, 모델이 존재하기 전에는 컨트롤러가 인터페이스를 점유할 수 없기 때문임.  
URDF는 `sim_gazebo:=true`일 때만 ros2_control과 Gazebo 플러그인 블록을 전개하므로, 뷰어와 기구학 테스트는 시뮬레이터 태그 없이 같은 파일을 그대로 사용함.

```bash
run-gazebo headless:=true use_rviz:=false   # 물리만 검증 (GUI 없음)
run-gazebo world:=empty.sdf                 # 월드 교체
```

목표 지정 방법은 5절과 동일하며, `/joint_states`는 이제 시뮬레이터가 발행함.

### 7. MuJoCo 연동

```bash
run-mujoco
```

같은 motion_server를 MuJoCo 물리 위에서 구동함.  
Gazebo와 달리 MuJoCo는 별도 프로세스가 아니라 노드 안에서 도는 라이브러리라, 스폰도 컨트롤러 기동도 없고 백엔드가 직접 물리를 전진시키며 `/joint_states`까지 발행함.

URDF가 표현하지 못하는 3가지를 [mujoco_model.py](src/robot_control/robot_control/mujoco_model.py)가 모델을 만들며 채우고, 값은 [config/mujoco_model.yaml](src/robot_control/config/mujoco_model.yaml)에서 읽음.

| 항목              | 없으면                     | 이유                                                                         |
| ----------------- | -------------------------- | ---------------------------------------------------------------------------- |
| `armature` 0.1    | 손목 3축이 발산            | 감속기 반사 관성, 손목 링크 관성은 2e-4에 불과해 2 ms 간격으로는 버티지 못함 |
| `gravcomp` 1      | 관절 2·3이 0.0072 rad 처짐 | 중력 보상, 실제 UR 제어기도 수행함                                           |
| `position` 구동기 | 팔이 중력에 무너짐         | URDF에 구동기 개념이 없고 `<mujoco>` 블록의 `<actuator>`는 오류 없이 무시됨  |

자기충돌은 시뮬레이터에서 끄고 캡슐 검사기에 맡김. MuJoCo는 부모가 world인 접촉만은 걸러 내지 않는데, DH 분해가 `base_link`와 `link1` 사이에 가상 링크를 끼워 넣어 0.3 mm 겹친 어깨 메쉬가 접촉으로 잡히고 어깨 관절이 잠김.  
다만 접촉을 끈 geom은 `discardvisual` 기본값 때문에 컴파일에서 사라지므로 이 옵션도 함께 꺼야 화면에 무언가 남음.

시뮬레이터별로 보아야 할 자리는 아래 한 줄씩임.

| 대상   | 런치                 | 설정                          | 백엔드          | visual 메쉬            |
| ------ | -------------------- | ----------------------------- | --------------- | ---------------------- |
| RViz만 | `control.launch.py`  | -                             | `SimBackend`    | `meshes/visual/`       |
| Gazebo | `gazebo.launch.py`   | `config/gazebo_controllers.yaml` | `GazeboBackend` | `meshes/visual/`       |
| MuJoCo | `mujoco.launch.py`   | `config/mujoco_model.yaml`    | `MujocoBackend` | `meshes/visual_mujoco/` |

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

| 인자       | 기본값      | 대상 launch                                | 설명                    |
| ---------- | ----------- | ------------------------------------------ | ----------------------- |
| `use_rviz` | `true`      | demo / control / gazebo / mujoco.launch.py | RViz 동시 실행 여부     |
| `headless` | `false`     | gazebo.launch.py                           | Gazebo GUI 없이 서버만  |
| `world`    | `empty.sdf` | gazebo.launch.py                           | 불러올 Gazebo 월드 파일 |

---

## API / 인터페이스

**ROS 2 인터페이스**

| 이름                    | 타입                               | 설명                                                                                                                   |
| ----------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `/joint_states`         | Topic (sensor_msgs/JointState)     | 관절각 50 Hz 발행 (demo_player 또는 motion_server)                                                                     |
| `/motion_server/move_j` | Service (robot_interfaces/MoveJ)   | 목표 pose로 관절 5차 다항식 이동, 수락/거부 즉시 응답                                                                  |
| `/motion_server/move_l` | Service (robot_interfaces/MoveL)   | 목표 pose로 직선 이동, 실행 전 전 waypoint IK 검증                                                                     |
| `/motion_server/stop`   | Service (std_srvs/Trigger)         | 현 위치 즉시 정지                                                                                                      |
| `/motion_state`         | Topic (std_msgs/String)            | idle / moving / jog : 상태 변화 시 + 1 Hz                                                                              |
| `/tool_pose`            | Topic (geometry_msgs/PoseStamped)  | 현재 tool0 FK 결과 (base_link 기준)                                                                                    |
| `/jog_twist`            | Topic (geometry_msgs/TwistStamped) | base 프레임 jog 속도 명령으로, teleop_keyboard가 발행하고 motion_server가 idle에서만 수락                              |
| `demo_player.rate`      | Parameter (double)                 | 발행 주기 [Hz], 시퀀스 샘플링 주기와 공유                                                                              |
| `demo_player.loop`      | Parameter (bool)                   | 시퀀스 종료 시 반복 여부                                                                                               |
| `motion_server.*`       | Parameter                          | rate / home / v_max / a_max / linear_speed / jog_max_linear / jog_max_angular / jog_deadman_timeout / collision_margin |

**라이브러리 API (순수 Python)** : 함수별 파일과 이론 문서 링크는 구현 상세의 패키지별 목록에서 코어 항목에 정리함 ([구현 상세](#구현-상세)).

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

- [x] DH 기반 가상 6축 로봇 URDF 작성 ([1\_모델 확인](#1-모델-확인))
- [x] STL 기반 UR10e 모델 정렬 ([1\_모델 확인](#1-모델-확인))
- [x] 수치 IK (DLS) 구현 ([2\_단위 테스트](#2-단위-테스트))
- [x] Cartesian trajectory → joint trajectory 변환 ([2\_단위 테스트](#2-단위-테스트))
- [x] JointState 기반 RViz 재생 ([3\_데모 재생](#3-데모-재생))
- [x] 서비스 기반 런타임 제어 (move_j / move_l / stop) ([5.1\_서비스 호출](#51-서비스-호출))
- [x] 인터랙티브 마커 목표 지정 ([5.2\_인터랙티브 마커](#52-인터랙티브-마커))
- [x] 키보드 텔레옵 (Cartesian jog) ([5.3\_키보드 jog](#53-키보드-jog))
- [x] 캡슐 근사 자기충돌 검사 ([5\_런타임 제어](#5-런타임-제어))
- [x] Gazebo 연동 (ros2_control) ([6_Gazebo 연동](#6-gazebo-연동))
- [x] MuJoCo 연동 ([7_MuJoCo 연동](#7-mujoco-연동))
- [ ] Isaac Sim 연동
- [ ] 실로봇 연동 인터페이스 정리

---

## 라이선스

이 프로젝트는 Apache-2.0 라이선스로 배포됩니다.

---

## Maintainer

**hhanoo** (woo980711@gmail.com)
