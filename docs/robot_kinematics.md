# robot_kinematics — DH · FK · Jacobian · IK<!-- omit from toc -->

관절각과 tool0 pose를 잇는 네 가지 계산. 각 절은 **문제 → 유도 → 코드 대응 → 함정 → 검증** 순서.

- [0. 전체 그림](#0-전체-그림)
- [1. DH 파라미터 (`dh.py`)](#1-dh-파라미터-dhpy)
  - [1.1 몇 개의 숫자로 링크를 적을 것인가](#11-몇-개의-숫자로-링크를-적을-것인가)
  - [1.2 링크 변환 행렬 유도](#12-링크-변환-행렬-유도)
  - [1.3 코드: dh\_transform](#13-코드-dh_transform)
  - [1.4 UR10e 테이블 읽기](#14-ur10e-테이블-읽기)
  - [1.5 DH 규약의 함정](#15-dh-규약의-함정)
  - [1.6 검증: 기본 변환](#16-검증-기본-변환)
- [2. FK (`fk.py`)](#2-fk-fkpy)
  - [2.1 관절각에서 pose로](#21-관절각에서-pose로)
  - [2.2 누적곱](#22-누적곱)
  - [2.3 코드: fk\_frames](#23-코드-fk_frames)
  - [2.4 중간 프레임을 전부 반환하는 이유](#24-중간-프레임을-전부-반환하는-이유)
  - [2.5 영점 자세 손계산](#25-영점-자세-손계산)
  - [2.6 검증: 독립 정답 두 개](#26-검증-독립-정답-두-개)
- [3. Jacobian (`jacobian.py`)](#3-jacobian-jacobianpy)
  - [3.1 관절 속도에서 pose 속도로](#31-관절-속도에서-pose-속도로)
  - [3.2 열 공식 유도](#32-열-공식-유도)
  - [3.3 코드: jacobian](#33-코드-jacobian)
  - [3.4 인덱스와 행 순서 규약](#34-인덱스와-행-순서-규약)
  - [3.5 특이점](#35-특이점)
  - [3.6 검증: 수치미분 대조](#36-검증-수치미분-대조)
- [4. IK (`ik.py`)](#4-ik-ikpy)
  - [4.1 pose에서 관절각으로](#41-pose에서-관절각으로)
  - [4.2 pose 오차의 6-벡터화](#42-pose-오차의-6-벡터화)
  - [4.3 뉴턴에서 DLS까지](#43-뉴턴에서-dls까지)
  - [4.4 DLS와 특이점](#44-dls와-특이점)
  - [4.5 코드: solve\_ik](#45-코드-solve_ik)
  - [4.6 IK 사용 시 주의점](#46-ik-사용-시-주의점)
  - [4.7 검증: 왕복과 실패](#47-검증-왕복과-실패)
- [참고 문헌](#참고-문헌)

---

## 0. 전체 그림

네 모듈은 아래로 갈수록 앞의 것을 쌓아 올리는 구조.

```
dh.py         DH 한 행 (a, d, α) + 관절각 θ  →  4x4 링크 변환
   │
   ▼
fk.py         링크 변환 누적곱  →  base_link ~ tool0 pose
   │
   ▼
jacobian.py   FK 중간 프레임 재사용  →  q̇ → twist (6xN)
   │
   ▼
ik.py         Jacobian 반복 선형화  →  목표 pose → 관절각
```

문서 전체 공통 기호.

| 기호                 | 의미                                     |
| -------------------- | ---------------------------------------- |
| $q \in \mathbb{R}^6$ | 관절각 벡터 [rad]                        |
| $T \in SE(3)$        | 4×4 동차변환 (pose = 위치 + 방향)        |
| $R \in SO(3)$        | 3×3 회전행렬                             |
| $p_e$                | end-effector(tool0) 위치                 |
| $z_i,\ p_i$          | 프레임 $i$의 z축 방향과 원점 (base 기준) |
| $J$                  | 기하학적 Jacobian (6×6)                  |
| $e$                  | 목표와 현재 pose의 6D 오차               |

---

## 1. DH 파라미터 (`dh.py`)

### 1.1 몇 개의 숫자로 링크를 적을 것인가

두 프레임 사이 강체 변환의 자유도는 6개(위치 3 + 방향 3). 로봇 링크를 이어 붙일 때는 6개를 다 쓸 필요가 없음.

Denavit–Hartenberg 규약은 **프레임 배치에 제약을 걸어** 자유도를 4개로 줄이는 방식. 규칙은 두 가지.

1. 프레임 $i$의 $z_i$ 축 = **관절 $i{+}1$의 회전축**
2. 프레임 $i$의 $x_i$ 축 = $z_{i-1}$과 $z_i$의 **common normal(공통수직선)** 방향

2번이 핵심. $x_i \perp z_{i-1}$이 강제되므로 프레임 $i{-}1 \to i$ 변환에 y축 이동이나 y축 회전이 등장할 수 없음. 남는 것은 z축 둘, x축 둘.

| 파라미터   | 축  | 의미                                |
| ---------- | --- | ----------------------------------- |
| $\theta_i$ | z   | 관절 회전각 (revolute의 **변수**)   |
| $d_i$      | z   | 링크 offset (축을 따라 떨어진 거리) |
| $a_i$      | x   | 링크 길이 (common normal의 길이)    |
| $\alpha_i$ | x   | 링크 twist (인접 두 축이 이루는 각) |

"6개 필요한데 4개로 줄였다"가 아니라 **프레임을 규칙대로 놓으면 4개로 충분해진다**는 것이 요점. 대가는 프레임 위치를 자유롭게 고를 수 없다는 제약.

### 1.2 링크 변환 행렬 유도

standard(classic) DH의 프레임 $i{-}1 \to i$ 변환은 네 기본 변환의 곱.

$$
{}^{i-1}T_i = R_z(\theta_i)\, T_z(d_i)\, T_x(a_i)\, R_x(\alpha_i)
$$

**"z축 일 먼저, x축 일 나중"**. 앞의 둘은 z축을, 뒤의 둘은 (회전된) x축을 건드림.

> $R_z(\theta)$와 $T_z(d)$는 **교환 가능** — 둘 다 같은 z축에 대한 조작. 문헌에 따라 $T_z(d) R_z(\theta) T_x(a) R_x(\alpha)$로 적기도 하지만 같은 행렬. 이 성질이 [robot_description.md](robot_description.md)의 URDF 변환 근거.

단계별 전개.

$$
R_z(\theta) T_z(d) =
\begin{bmatrix} c\theta & -s\theta & 0 & 0 \\ s\theta & c\theta & 0 & 0 \\ 0 & 0 & 1 & d \\ 0&0&0&1 \end{bmatrix}
$$

$T_x(a)$를 곱하면 이동 성분이 **현재 회전으로 돌려진 채** 더해짐. $R \cdot (a, 0, 0)^\top = (a\,c\theta,\ a\,s\theta,\ 0)^\top$ 이므로

$$
R_z(\theta) T_z(d) T_x(a) =
\begin{bmatrix} c\theta & -s\theta & 0 & a\,c\theta \\ s\theta & c\theta & 0 & a\,s\theta \\ 0 & 0 & 1 & d \\ 0&0&0&1 \end{bmatrix}
$$

$R_x(\alpha)$는 회전 성분에만 오른쪽 곱으로 붙음. $R_z(\theta)R_x(\alpha)$를 계산하면 최종 형태.

$$
{}^{i-1}T_i =
\begin{bmatrix}
c\theta & -s\theta\, c\alpha & s\theta\, s\alpha & a\, c\theta \\
s\theta & c\theta\, c\alpha & -c\theta\, s\alpha & a\, s\theta \\
0 & s\alpha & c\alpha & d \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 1.3 코드: dh_transform

위 행렬을 그대로 옮긴 것.

```python
def dh_transform(theta, d, a, alpha):
    """Standard DH link transform: Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d     ],
            [0.0, 0.0,      0.0,     1.0   ],
        ]
    )
```

네 번의 행렬 곱 대신 **전개 결과를 상수 시간에 채워 넣는 구조**. FK가 관절 수만큼 호출하고 IK가 FK를 매 반복 호출하므로 이 한 함수가 전체 성능을 좌우.

행렬 리터럴은 `# fmt: off` / `# fmt: on`으로 감싸 열 정렬 유지. black 재포맷을 막지 않으면 행렬 모양이 무너져 검토가 어려워짐.

### 1.4 UR10e 테이블 읽기

```python
# Standard DH rows: (a, d, alpha). theta comes from the joint angle.
UR10E_DH = np.array(
    [
        [0.0,      0.1807,   math.pi / 2],   # joint 1
        [-0.6127,  0.0,      0.0        ],   # joint 2
        [-0.57155, 0.0,      0.0        ],   # joint 3
        [0.0,      0.17415,  math.pi / 2],   # joint 4
        [0.0,      0.11985, -math.pi / 2],   # joint 5
        [0.0,      0.11655,  0.0        ],   # joint 6
    ]
)
```

숫자에서 로봇 형상이 읽힘.

- **joint 2, 3의 $a$가 크고 음수** — upper arm 0.6127 m, forearm 0.5716 m로 UR10e 팔 길이 그 자체. $\alpha = 0$ 이라 두 축이 평행 (어깨-팔꿈치가 한 평면에서 움직이는 이유)
- **joint 1, 4, 5의 $\alpha = \pm 90°$** — 축이 직각으로 꺾이는 지점. joint 4·5·6이 한 점 근처에 모여 wrist를 구성
- **$a$의 음수 부호** — common normal 방향을 어느 쪽으로 잡았느냐의 문제일 뿐 물리적 길이는 절댓값. UR 공식 표기를 그대로 따른 것
- **$\theta$ 열 없음** — revolute의 $\theta$는 상수가 아닌 변수라 테이블이 아니라 `q` 인자로 들어옴

### 1.5 DH 규약의 함정

**standard vs modified.** 두 규약은 프레임을 링크의 어느 쪽 끝에 붙이느냐가 다름.

|                    | 변환 순서                                               | 프레임 $i$의 위치  |
| ------------------ | ------------------------------------------------------- | ------------------ |
| standard (classic) | $R_z(\theta_i) T_z(d_i) T_x(a_i) R_x(\alpha_i)$         | 관절 $i{+}1$ 축 위 |
| modified (Craig)   | $R_x(\alpha_{i-1}) T_x(a_{i-1}) R_z(\theta_i) T_z(d_i)$ | 관절 $i$ 축 위     |

**같은 로봇이라도 두 규약의 파라미터 값이 다름.** 이 프로젝트는 standard. 외부 DH 표를 가져올 때 규약 확인을 빠뜨리면 조용히 틀린 FK가 나옴.

**인자 순서.** 테이블 한 행은 `(a, d, alpha)` 순인데 `dh_transform()` 시그니처는 `(theta, d, a, alpha)`. `a`와 `d` 자리가 뒤바뀜. `fk.py`가 이 교환을 처리.

```python
for i, (theta, (a, d, alpha)) in enumerate(zip(q, dh)):
    frames[i + 1] = frames[i] @ dh_transform(theta, d, a, alpha)
#                                                   ^^^^  테이블 순서와 반대
```

바꿔 넣어도 예외 없이 그럴듯한 숫자가 나오므로 FK-URDF 대조 테스트에서만 잡히는 종류의 실수.

**xacro와의 동기화.** `dh.py` 값과 `ur10e.urdf.xacro`의 `xacro:property` 값은 **항상 일치해야 함**. 한쪽만 고치면 계산 결과와 RViz 화면이 어긋남. 규약이 아니라 테스트로 강제되는 구조 ([2.6절](#26-검증-독립-정답-두-개)).

### 1.6 검증: 기본 변환

`test/test_fk.py`의 `TestDHTransform`이 네 기본 변환의 조립을 최소 케이스로 확인.

- `test_pure_d_translation` — $\theta=a=\alpha=0$ 이면 순수 z 이동
- `test_pure_a_translation` — 순수 x 이동
- `test_theta_rotates_about_z` — $\theta = 90°,\ a = 1$ 일 때 $(0, 1, 0)$

마지막 케이스가 **곱의 순서**를 잡아내는 역할. $a$ 이동이 $\theta$ 회전보다 나중이라 x축 offset이 +y로 돌아감. 순서가 뒤집혔다면 $(1, 0, 0)$.

---

## 2. FK (`fk.py`)

### 2.1 관절각에서 pose로

관절각 $q = (\theta_1, \dots, \theta_6)$가 주어졌을 때 base_link 기준 tool0의 pose ${}^{0}T_6$를 구하는 문제. 해가 항상 유일하게 존재하는 쉬운 방향 (어려운 쪽은 [4절](#4-ik-ikpy)).

### 2.2 누적곱

동차변환은 곱으로 합성됨. 프레임을 이어 붙이면

$$
{}^{0}T_6 = {}^{0}T_1(\theta_1)\, {}^{1}T_2(\theta_2)\, \cdots\, {}^{5}T_6(\theta_6)
$$

각 항은 [1.2절](#12-링크-변환-행렬-유도)의 `dh_transform`. 유도랄 것도 없지만 두 가지는 짚어둘 만함.

**곱의 방향.** 왼쪽에서 오른쪽으로 곱하는 것은 각 변환을 **직전 프레임의 로컬 좌표계 기준**으로 적용한다는 뜻. 덕분에 부분곱 ${}^{0}T_k$가 항상 "base에서 본 프레임 $k$"라는 의미를 유지.

**수치 오차.** 6번의 행렬 곱이 부동소수 오차를 누적시키지만 각 행렬이 직교(회전)라 증폭되지 않음. 실제로 무작위 자세 10개의 모든 중간 프레임에서 $RR^\top = I$, $\det R = 1$이 $10^{-9}$ 이내로 유지됨 (`test_rotation_matrices_are_orthonormal`).

### 2.3 코드: fk_frames

```python
def fk_frames(q, dh=None):
    if dh is None:
        dh = UR10E_DH
    q = np.asarray(q, dtype=float)
    frames = np.empty((len(q) + 1, 4, 4))
    frames[0] = np.eye(4)
    for i, (theta, (a, d, alpha)) in enumerate(zip(q, dh)):
        frames[i + 1] = frames[i] @ dh_transform(theta, d, a, alpha)
    return frames


def fk(q, dh=None):
    return fk_frames(q, dh)[-1]
```

`frames[0] = np.eye(4)`가 base 프레임. 단위행렬을 실제로 저장하는 게 낭비 같지만 덕분에 **`frames[i]`가 곧 "프레임 $i$"** 라는 인덱스 규약이 성립. Jacobian이 이 규약을 그대로 사용.

`fk()`는 마지막 프레임만 꺼내는 얇은 wrapper.

### 2.4 중간 프레임을 전부 반환하는 이유

`fk()`만 있으면 될 것 같지만 주 함수는 `fk_frames()`. 이유는 Jacobian.

Jacobian의 열 $i$는 **관절 $i$의 축과 원점**을 필요로 하고 ([3.2절](#32-열-공식-유도)), 그 정보는 중간 프레임 ${}^{0}T_i$에 들어 있음. `fk()`만 제공하면 Jacobian이 관절마다 FK를 다시 돌려야 해서 비용이 $O(n)$에서 $O(n^2)$로 증가.

IK는 반복마다 FK 1회 + Jacobian 1회를 호출하므로 이 차이가 그대로 IK 성능이 됨.

반환 형태는 `(n+1, 4, 4)` numpy 배열. 리스트가 아니라 배열이라 `frames[-1]`, `frames[i][:3, 2]` 같은 슬라이싱이 그대로 통함.

### 2.5 영점 자세 손계산

$q = 0$에서의 tool0 위치는 코드 없이 계산 가능. 테스트 정답으로 쓰이므로 유도해 둠.

$\theta = 0$이면 $c\theta = 1, s\theta = 0$이라 링크 변환이 단순해짐.

$$
{}^{i-1}T_i(0) =
\begin{bmatrix}
1 & 0 & 0 & a \\
0 & c\alpha & -s\alpha & 0 \\
0 & s\alpha & c\alpha & d \\
0&0&0&1
\end{bmatrix}
= \text{Trans}(a, 0, d) \cdot R_x(\alpha)
$$

위치를 하나씩 누적. 각 단계의 이동량은 **그 시점까지의 회전** $R$로 돌려서 더함. UR10e의 $\alpha$는 $\pm 90°$ 아니면 $0$뿐이라 $R$은 항상 $R_x$의 배수.

| 단계 | 로컬 이동   | 누적 회전 $R$ | 누적 위치 $p$                            |
| ---- | ----------- | ------------- | ---------------------------------------- |
| 시작 | —           | $I$           | $(0,\ 0,\ 0)$                            |
| 1    | $(0,0,d_1)$ | $R_x(90°)$    | $(0,\ 0,\ d_1)$                          |
| 2    | $(a_2,0,0)$ | $R_x(90°)$    | $(a_2,\ 0,\ d_1)$                        |
| 3    | $(a_3,0,0)$ | $R_x(90°)$    | $(a_2{+}a_3,\ 0,\ d_1)$                  |
| 4    | $(0,0,d_4)$ | $R_x(180°)$   | $(a_2{+}a_3,\ -d_4,\ d_1)$               |
| 5    | $(0,0,d_5)$ | $R_x(90°)$    | $(a_2{+}a_3,\ -d_4,\ d_1{-}d_5)$         |
| 6    | $(0,0,d_6)$ | $R_x(90°)$    | $(a_2{+}a_3,\ -(d_4{+}d_6),\ d_1{-}d_5)$ |

4단계에서 $R_x(90°)$가 로컬 $+z$ 이동을 전역 $-y$로, 5단계에서 $R_x(180°)$가 $+z$를 $-z$로 보내는 것이 핵심.

$$
p_e(0) = \begin{bmatrix} a_2 + a_3 \\ -(d_4 + d_6) \\ d_1 - d_5 \end{bmatrix}, \qquad R_e(0) = R_x(90°)
$$

숫자를 넣으면 $(-1.184,\ -0.291,\ 0.061)$ m. $a_2, a_3$가 음수라 팔이 $-x$ 방향으로 뻗은 자세.

### 2.6 검증: 독립 정답 두 개

FK는 **서로 독립인 정답 두 개**로 검증. 하나가 틀려도 다른 하나가 잡아내는 구성.

**정답 1 — 손으로 유도한 폐형식** (`TestZeroPose`)
[2.5절](#25-영점-자세-손계산) 결과를 하드코딩해 $10^{-9}$ 이내로 대조. 검증 대상 코드를 거치지 않고 얻은 값이라는 점이 요점.

**정답 2 — xacro가 전개한 URDF 체인** (`TestAgainstURDF`)
`xacro.process_file()`로 URDF를 전개한 뒤 `tool0`에서 부모를 거슬러 올라가며 joint origin과 axis 회전을 직접 합성. 무작위 관절각 100개에서 DH 계산과 $10^{-6}$ 이내로 일치해야 함.

이 테스트가 **RViz가 그리는 로봇과 IK가 푸는 로봇이 같은 로봇임을 보장**. `dh.py`와 xacro 값이 어긋나면 즉시 실패. 두 곳의 상수 동기화 규약이 문서가 아니라 테스트로 강제되는 구조.

> `xacro` 파이썬 모듈이 없으면 `pytest.importorskip`으로 건너뜀. 프로젝트 컨테이너 안에서 실행해야 이 대조가 실제로 도는 점에 주의.

---

## 3. Jacobian (`jacobian.py`)

### 3.1 관절 속도에서 pose 속도로

FK $p_e = f(q)$는 비선형. 하지만 **특정 자세 근방**에서는 관절을 조금 움직였을 때 tool0가 얼마나 움직이는지를 선형으로 근사 가능. 그 선형 사상이 Jacobian.

$$
\begin{bmatrix} v_e \\ \omega_e \end{bmatrix} = J(q)\, \dot q
$$

- 위 3행 → 선속도 $v_e$ [m/s], 아래 3행 → 각속도 $\omega_e$ [rad/s]
- 전부 **base 프레임** 기준
- $J$는 $q$에 의존. 자세가 바뀌면 다시 계산 필요

이 한 장의 행렬이 속도 제어, 특이점 판정, IK([4절](#4-ik-ikpy))의 기반.

### 3.2 열 공식 유도

$J$의 $i$번째 열은 정의상 $\partial(\text{tool0 pose})/\partial q_i$ — **관절 $i$만 단위 속도로 움직였을 때의 tool0 속도**. 나머지 관절이 고정된 상황을 그려보면 답이 바로 나옴.

관절 $i$만 $\dot q_i$로 회전하면 그 관절보다 **바깥쪽 링크 전체가 하나의 강체**가 되어 축선 $(p_i,\ z_i)$를 중심으로 회전. 강체 회전의 각속도는

$$
\omega = \dot q_i\, z_i
$$

그 강체 위 임의의 점 $p_e$의 속도는 강체 운동학의 기본 공식으로 결정.

$$
v = \omega \times (p_e - p_i) = \dot q_i \big( z_i \times (p_e - p_i) \big)
$$

$\dot q_i$로 나누면 열이 됨.

$$
J_i = \begin{bmatrix} z_i \times (p_e - p_i) \\ z_i \end{bmatrix}
$$

여러 관절이 동시에 움직이면 각 기여가 **선형 중첩** (속도는 미분이라 합이 성립). 열을 나란히 세우면 $6 \times n$ 행렬 완성.

> **$p_i$는 축선 위의 아무 점이어도 무방.** $p_i \to p_i + c\, z_i$로 옮겨도 $z_i \times (p_e - p_i)$는 불변 ($z_i \times z_i = 0$). 그래서 "프레임 원점"이라는 편한 선택이 가능.

### 3.3 코드: jacobian

```python
def jacobian(q, dh=None):
    frames = fk_frames(q, dh)
    p_e = frames[-1][:3, 3]
    n = len(frames) - 1
    J = np.zeros((6, n))
    for i in range(n):
        z_i = frames[i][:3, 2]
        p_i = frames[i][:3, 3]
        J[:3, i] = np.cross(z_i, p_e - p_i)
        J[3:, i] = z_i
    return J
```

유도한 식이 거의 그대로 옮겨진 형태.

| 코드                       | 수식                                    |
| -------------------------- | --------------------------------------- |
| `frames[i][:3, 2]`         | $z_i$ — 회전행렬의 3번째 열이 z축 방향  |
| `frames[i][:3, 3]`         | $p_i$ — 동차변환의 4번째 열이 원점 위치 |
| `np.cross(z_i, p_e - p_i)` | $z_i \times (p_e - p_i)$                |

`fk_frames()`를 **한 번만** 호출해 모든 프레임을 재사용하므로 FK 1회 + $O(n)$ 외적으로 종료 ([2.4절](#24-중간-프레임을-전부-반환하는-이유)).

### 3.4 인덱스와 행 순서 규약

**프레임 인덱스가 한 칸 밀림.** 이 코드에서 가장 틀리기 쉬운 부분.

standard DH의 $\theta_i$는 **이전 프레임의 z축** 둘레 회전 ([1.2절](#12-링크-변환-행렬-유도)에서 $R_z(\theta)$가 곱의 맨 앞). 따라서 관절 $i$의 축은 프레임 $i$가 아니라 프레임 $i{-}1$에 존재.

```
frames[0] = I        ← joint 1의 축 (= world z)
frames[1]            ← joint 2의 축
frames[2]            ← joint 3의 축
   ...
frames[5]            ← joint 6의 축
frames[6] = tool0    ← 축으로는 안 쓰임, p_e 로만 사용
```

0-인덱스 루프에서 `i`번째 열(= 관절 `i+1`)이 `frames[i]`를 읽는 이유. `frames[i+1]`로 잘못 쓰면 **FK는 멀쩡한데 Jacobian만 틀림**. IK가 수렴은 하는데 이상하게 느리거나 엉뚱한 해로 가는 식이라 원인 추적이 어려움.

**행 순서는 `[v; ω]`.** screw theory 계열 문헌(Modern Robotics 등)과 일부 라이브러리는 `[ω; v]` 순서를 사용. 이 프로젝트는 `ik.py`의 오차 벡터도 `[dp; rotation_vector(...)]` 순서라 짝이 맞음. KDL, Pinocchio 등 외부 라이브러리와 섞을 때는 확인 필수.

**revolute 전용.** prismatic 관절이면 열이 $[z_i;\ 0]$이어야 하는데 그 분기가 없음. UR10e는 6축 전부 revolute라 문제없지만, DH 테이블만 갈아끼워 prismatic 축이 있는 로봇에 쓰면 조용히 틀린 결과가 나옴.

**"기하학적" Jacobian이라는 이름.** 아래 3행이 실제 각속도 $\omega$이지 Euler angle이나 rotation vector의 시간미분이 아니라는 뜻. 덕분에 표현 특이점(gimbal lock)이 없음. 대신 $\int \omega\, dt$는 자세가 아니므로 각속도 적분으로 방향을 얻을 수는 없음.

### 3.5 특이점

$J$의 rank가 6 미만이면 **어떤 $\dot q$로도 tool0를 움직일 수 없는 방향**이 존재. 그 자세가 singularity(특이점).

$q = 0$이 그런 자세. wrist의 4번과 6번 축이 정렬되어 같은 회전을 만들기 때문에 두 열이 선형종속이 되고, 6개 관절로 5차원 방향밖에 만들지 못함.

특이점 **근처**도 문제. rank는 6이지만 최소 특이값 $\sigma_{\min}$이 0에 가까워, tool0를 조금 움직이려면 관절이 극단적으로 빨리 돌아야 함. 역행렬 기반 IK가 발산하는 지점이자 DLS가 존재하는 이유 ([4.4절](#44-dls와-특이점)).

실무에서는 `np.linalg.svd(J)`의 최소 특이값을 manipulability 지표로 감시.

### 3.6 검증: 수치미분 대조

`test/test_jacobian.py`가 **FK의 중앙차분**을 정답으로 사용. 해석적으로 유도한 Jacobian은 부호나 인덱스 실수가 나기 쉬운데, 사실상 이 대조가 유일하게 믿을 만한 검증법.

```python
J[:3, j] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * EPS)
R_err = Tp[:3, :3] @ Tm[:3, :3].T
w = np.array([R_err[2,1] - R_err[1,2], ...]) / 2.0
J[3:, j] = w / (2 * EPS)
```

위 3행은 위치를 직접 차분하면 되지만 아래 3행은 회전행렬을 뺄 수 없어 처리가 다름. $R(q{+}\epsilon) R(q{-}\epsilon)^\top$ 이라는 **상대 회전**을 만든 뒤 그 미소 rotation vector를 비대칭 성분에서 추출. 미소각에서 $\sin\theta \approx \theta$ 이므로 나눗셈 없이 근사가 성립.

| 테스트                                     | 확인 내용                          |
| ------------------------------------------ | ---------------------------------- |
| `test_shape`                               | 6×6                                |
| `test_matches_finite_differences_random_q` | 무작위 20자세, $10^{-5}$ 이내 일치 |
| `test_singular_at_zero_pose`               | $q=0$에서 rank < 6                 |

마지막은 정확도가 아니라 **성질**을 보는 테스트. wrist 특이점에서 rank가 떨어지지 않는다면 축 배치나 인덱스가 틀렸다는 신호.

---

## 4. IK (`ik.py`)

### 4.1 pose에서 관절각으로

목표 pose $T^\ast$가 주어졌을 때 $f(q) = T^\ast$를 만족하는 $q$를 찾는 문제. FK와 달리 까다로움.

- **해가 여러 개** — 6축 로봇은 보통 최대 8개의 해 분기 (shoulder 좌/우, elbow 위/아래, wrist flip)
- **해가 없을 수 있음** — 작업공간 밖 목표
- **해석해가 로봇마다 다름** — UR처럼 폐형식이 존재하는 구조도 있지만 유도가 로봇 전용이라 재사용 불가

이 프로젝트는 **수치 해법**을 선택. 로봇이 바뀌어도 DH 테이블 교체로 끝나고, Jacobian이라는 기존 도구를 재사용하며, 이후 단계에서 Isaac Lab `DifferentialIKController(ik_method="dls")`와 같은 수식을 쓰게 되기 때문. 대가는 반복 비용과 국소 수렴(seed 의존).

### 4.2 pose 오차의 6-벡터화

반복법을 쓰려면 "목표까지 얼마나 남았나"를 벡터 하나로 표현해야 함. 위치는 그냥 빼면 끝.

$$
e_{\text{pos}} = p^\ast - p
$$

회전은 뺄 수 없음. $R^\ast - R$은 회전행렬이 아니고 크기에 물리적 의미도 없음. 대신 **상대 회전**을 만든 뒤 축·각으로 푸는 방식.

$$
R_{\text{err}} = R^\ast R^\top, \qquad e_{\text{rot}} = \log(R_{\text{err}}) = \theta\, \hat{a}
$$

$\hat a$는 회전축, $\theta$는 회전각. 이 3-벡터가 **rotation vector**이며, 크기가 곧 남은 회전각이라 오차 척도로 자연스러움. 무엇보다 기하학적 Jacobian의 $\omega$ 행과 **1차 근사에서 정확히 대응** — $\omega \Delta t$가 미소 rotation vector이기 때문.

**Rodrigues 공식의 역연산.** $R = I + \sin\theta\, K + (1-\cos\theta) K^2$ ($K$ = 축의 skew 행렬)를 $\theta, \hat a$에 대해 푸는 문제. $R$은 답을 세 군데에 나눠 갖고 있음.

| $R$의 부분                                    | 담긴 정보                      |
| --------------------------------------------- | ------------------------------ |
| 비대칭 성분 $(R - R^\top)/2 = \sin\theta\, K$ | 축 (일반적인 경우)             |
| trace $\text{tr}(R) = 1 + 2\cos\theta$        | 각도 크기                      |
| 대칭 성분 $(R + I)/2 = \hat a \hat a^\top$    | 축 ($\theta \approx \pi$ 폴백) |

```python
# 1. Read w = sin(t) * a from the antisymmetric part
w = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
s = np.linalg.norm(w)                                  # sin(t), >= 0

# 2. Read cos(t) from the trace
c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)

# 3. atan2(sin, cos) is stable over the whole range
angle = math.atan2(s, c)
```

`atan2`를 쓰는 이유가 있음. $\arccos$만으로 $\theta$를 구하면 $\theta \approx 0$과 $\theta \approx \pi$ 근처에서 도함수가 발산해 정밀도 손실. $\sin$과 $\cos$을 모두 주면 전 구간 안정적. `clip`은 부동소수 오차로 trace가 $\pm 1$을 살짝 벗어나 `NaN`이 되는 것을 방지.

$\sin\theta \approx 0$인 두 경우는 축을 비대칭 성분에서 뽑을 수 없어 분기.

```python
if s < 1e-10:
    if c > 0.0:
        # t ~ 0: axis is undefined but w = sin(t)*a ~ t*a is the answer
        return w
    # t ~ pi: recover axis from (R + I)/2 = aa^T (largest-diagonal column)
    A = (R + np.eye(3)) / 2.0
    i = int(np.argmax(np.diag(A)))
    axis = A[:, i] / math.sqrt(A[i, i])
```

- **$\theta \approx 0$** — 축은 정의되지 않지만 답인 $\theta\hat a$는 잘 정의됨. $\sin\theta \approx \theta$ 이므로 `w`를 그대로 반환
- **$\theta \approx \pi$** — 비대칭 성분이 사라지지만 대칭 성분에 $\hat a \hat a^\top$이 남음. 어느 열을 골라도 $\hat a$의 상수배지만 **대각 성분이 가장 큰 열**을 골라야 0으로 나누는 것을 회피

이렇게 만든 6-벡터가 `_pose_error()`.

```python
def _pose_error(target, T):
    """6D error twist: [dp; rotation_vector(R_t R^T)]."""
    e[:3] = target[:3, 3] - T[:3, 3]
    e[3:] = rotation_vector(target[:3, :3] @ T[:3, :3].T)
```

행 순서가 Jacobian의 `[v; ω]`와 일치 ([3.4절](#34-인덱스와-행-순서-규약)).

### 4.3 뉴턴에서 DLS까지

**1단계 — 선형화.** 현재 $q$에서 목표까지 오차가 $e$일 때 Jacobian 정의에 따라

$$
J\, \Delta q \approx e
$$

이 선형계를 풀어 $q \leftarrow q + \Delta q$로 갱신하고 반복하면 뉴턴 계열 반복법.

**2단계 — 역행렬이 안 되는 이유.** $J$가 정방(6×6)이니 $\Delta q = J^{-1} e$면 될 것 같고, 특이점에서 멀면 실제로 잘 동작. 문제는 특이점 근처. $\sigma_{\min} \to 0$이면 $\|J^{-1}\| \to \infty$라 $\Delta q$가 폭발하고, 관절이 미친 듯이 돌면서 선형 근사가 깨져 발산.

redundant 로봇($n > 6$)이면 정방도 아니라 애초에 역행렬이 없음. 일반적으로는 pseudo-inverse $J^{+} = J^\top (JJ^\top)^{-1}$을 쓰지만 $JJ^\top$이 특이점에서 특이행렬이 되므로 같은 문제가 잔존.

**3단계 — 정규화 항 추가.** 오차만 줄이지 말고 **관절 변화량도 함께 억제**하는 목적함수를 세움.

$$
\Delta q^\ast = \arg\min_{\Delta q} \Big( \| J \Delta q - e \|^2 + \lambda^2 \|\Delta q\|^2 \Big)
$$

$\lambda$가 "정확도 대 안정성"의 저울. 미분해서 0으로 놓으면

$$
2 J^\top (J\Delta q - e) + 2\lambda^2 \Delta q = 0
\quad\Longrightarrow\quad
(J^\top J + \lambda^2 I_n)\, \Delta q = J^\top e
$$

$$
\Delta q = (J^\top J + \lambda^2 I_n)^{-1} J^\top e
$$

**4단계 — 행렬 항등식으로 뒤집기.** 위 형태는 $n \times n$ 역행렬을 요구. 다음 항등식으로 $6 \times 6$ 문제로 전환 가능.

$$
(J^\top J + \lambda^2 I_n)^{-1} J^\top = J^\top (J J^\top + \lambda^2 I_6)^{-1}
$$

최종 형태가 **Damped Least Squares(DLS)**. Levenberg–Marquardt 감쇠와 같은 아이디어.

$$
\boxed{\ \Delta q = J^\top \big( J J^\top + \lambda^2 I \big)^{-1} e\ }
$$

$\lambda^2 I$ 덕분에 $JJ^\top$이 특이해도 **역행렬이 항상 존재**. 6축이라 두 형태의 크기가 같지만, redundant 로봇으로 확장할 때 이 형태가 그대로 유효.

### 4.4 DLS와 특이점

SVD $J = U \Sigma V^\top$을 대입하면 damping 효과가 정확히 보임.

$$
\Delta q = \sum_i \frac{\sigma_i}{\sigma_i^2 + \lambda^2}\, v_i \,(u_i^\top e)
$$

pseudo-inverse면 계수가 $1/\sigma_i$. 두 계수 비교.

| $\sigma_i$    | pseudo-inverse $1/\sigma$ | DLS $\sigma/(\sigma^2+\lambda^2)$ |
| ------------- | ------------------------- | --------------------------------- |
| $\gg \lambda$ | $1/\sigma$                | $\approx 1/\sigma$ (거의 동일)    |
| $= \lambda$   | $1/\lambda$               | $1/(2\lambda)$ — **최댓값**       |
| $\to 0$       | $\to \infty$              | $\to \sigma/\lambda^2 \to 0$      |

- 잘 움직이는 방향($\sigma$ 큼)에서는 pseudo-inverse와 거의 동일하게 동작
- 특이 방향($\sigma \to 0$)에서는 **폭발 대신 0으로 수렴** — 갈 수 없는 방향은 포기하는 셈
- 증폭 상한이 $1/(2\lambda)$로 **엄밀히 유계**

$\lambda$ 선택의 트레이드오프도 여기서 읽힘. 코드 기본값은 `damping=0.05`.

- **$\lambda$가 크면** 특이점에서 안정적이지만 모든 방향의 스텝이 줄어 수렴이 느려지고 정상 영역에서도 오차가 잔존
- **$\lambda$가 작으면** 빠르지만 특이점 근처에서 과도한 스텝으로 진동하거나 발산

### 4.5 코드: solve_ik

```python
for it in range(max_iters):
    # 1. Current 6D error to target (position + rotation vector)
    e = _pose_error(target, fk(q, dh))
    pos_err = float(np.linalg.norm(e[:3]))
    rot_err = float(np.linalg.norm(e[3:]))

    # 2. Converged? Done
    if pos_err < tol_pos and rot_err < tol_rot:
        return IKResult(True, q, pos_err, rot_err, it)

    # 3. DLS step (lambda^2 keeps it bounded at singularities)
    J = jacobian(q, dh)
    dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)

    # 4. Clamp step size (Jacobian is only a local approximation)
    step = np.linalg.norm(dq)
    if step > max_step:
        dq *= max_step / step

    # 5. Apply and iterate
    q += dq
```

**3단계** — `np.linalg.inv()`가 아니라 `np.linalg.solve()`. 역행렬을 명시적으로 만들지 않는 쪽이 빠르고 수치적으로 안정적. 목적은 선형계 풀이지 역행렬 자체가 아님.

**4단계** — 유도에는 없는 항목. Jacobian은 **국소 근사**일 뿐이라 목표가 멀면 DLS가 계산한 $\Delta q$가 근사 유효 범위를 초과하고, 새 자세의 오차가 오히려 커질 수 있음. `max_step=0.5` rad로 스텝 노름을 잘라 유효 범위 안에 유지. 방향은 그대로 두고 크기만 줄이므로 수렴 방향은 보존.

**위치와 자세의 수렴을 따로 판정**하는 점도 의도적. 단위가 다르므로(m vs rad) 하나의 노름으로 합치면 임계값의 의미가 흐려짐.

**미수렴 시 예외를 던지지 않는 구조.** 루프를 다 돌면 마지막 상태를 그대로 담아 `IKResult` 반환.

```python
@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    pos_error: float
    rot_error: float
    iterations: int
```

궤적 생성에서 waypoint 수십 개를 연속으로 풀 때 실패가 예외가 아니라 값이어야 "몇 번째에서 얼마나 못 미쳤는지"를 호출자가 판단 가능 ([robot_trajectory.md](robot_trajectory.md)의 `cartesian_to_joint`가 이 구조에 의존).

### 4.6 IK 사용 시 주의점

**seed가 해를 결정.** DLS는 국소적 방법이라 **seed에서 가장 가까운 해 분기**로 수렴. 같은 목표라도 seed가 다르면 다른 자세가 나옴. 버그가 아니라 성질이며, 오히려 이 성질을 이용해 연속 경로를 생성 ([robot_trajectory.md](robot_trajectory.md)).

**수렴 실패 ≠ 도달 불가.** `success=False`는 "이 seed에서 이 반복 수 안에 못 갔다"는 뜻이지 "작업공간 밖"이라는 뜻이 아님. 다른 seed로 재시도하면 풀리는 경우가 흔함.

**`q`에 범위 제한 없음.** 관절 한계를 강제하지 않으므로 $2\pi$를 넘는 값이 나올 수 있음. URDF `limit`은 $\pm 6.283$ rad로 넉넉하지만 실로봇 연동 단계에서는 별도 wrapping·clamping 필요.

**`q += dq`는 제자리 갱신.** `q0`를 `.copy()`로 복사하므로 호출자 배열은 안전.

### 4.7 검증: 왕복과 실패

`test/test_ik.py`는 세 갈래 구성.

**rotation vector 단독** (`TestRotationVector`) — 항등행렬 → 0, 미소 회전 $10^{-4}$ rad, $90°$ 회전. IK 전체를 돌리기 전에 오차 표현부터 격리해 확인.

**FK → IK 왕복** (`TestIKRoundtrip`) — 무작위 $q_{\text{true}}$로 목표를 만들고 IK가 그 목표를 재현하는지 확인. 허용치는 위치 1 mm, 자세 $0.1°$.

```python
q_true = rng.uniform(-np.pi, np.pi, 6)
target = fk(q_true)
q_seed = q_true + rng.uniform(-0.3, 0.3, 6)
result = solve_ik(target, q_seed)
```

`q_true`와 `result.q`를 직접 비교하지 않는 점이 중요. 해가 여러 개이므로 **다른 관절각이 같은 pose를 만드는 것이 정상**. 비교는 항상 pose 공간에서.

seed 조건을 둘로 나눈 것도 의도적.

| 테스트                               | seed              | 기대             |
| ------------------------------------ | ----------------- | ---------------- |
| `test_converges_from_perturbed_seed` | 정답에서 ±0.3 rad | 30/30 전부 성공  |
| `test_converges_from_home_seed`      | 고정 home 자세    | 10개 중 8개 이상 |

앞은 궤적 추종 시나리오(직전 waypoint가 seed), 뒤는 cold start. 뒤쪽에서 100%를 요구하지 않는 것은 DLS가 국소적 방법이라는 사실을 테스트가 인정하는 것. 전부 통과하도록 조이면 오히려 성질을 왜곡하는 테스트가 됨.

**정상적인 실패** (`TestIKFailure`) — 두 가지 확인.

- 3 m 떨어진 도달 불가 목표: `success=False`이면서 `q`가 전부 유한 (`NaN` 금지)
- $q=0$ 특이점에서 출발: 발산 없이 유한한 값 유지. [4.4절](#44-dls와-특이점)에서 유도한 damping 효과의 최종 확인

---

## 참고 문헌

- Denavit, J., Hartenberg, R. S. (1955). _A kinematic notation for lower-pair mechanisms based on matrices_. ASME Journal of Applied Mechanics.
- Craig, J. J. _Introduction to Robotics: Mechanics and Control_. — modified DH 규약의 출처
- Siciliano, B. et al. _Robotics: Modelling, Planning and Control_. — 기하학적 Jacobian, DLS IK
- Buss, S. R. (2004). _Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods_. — DLS의 특이값 해석
- Nakamura, Y., Hanafusa, H. (1986). _Inverse Kinematic Solutions with Singularity Robustness for Robot Manipulator Control_. — DLS 원논문
