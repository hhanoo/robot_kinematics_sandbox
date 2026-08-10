# robot_description — DH 파라미터를 URDF로 옮기기<!-- omit from toc -->

DH 표 한 장으로 로봇을 계산할 수는 있어도 ([robot_kinematics.md](robot_kinematics.md)), RViz에 그리거나 ROS 생태계에 태우려면 URDF가 필요. 그런데 두 표현은 **변환 적용 순서가 다름**. 이 문서는 그 불일치를 해소한 방법을 다룸.

- [1. 문제: 두 규약의 순서 불일치](#1-문제-두-규약의-순서-불일치)
  - [1.1 DH가 요구하는 순서](#11-dh가-요구하는-순서)
  - [1.2 URDF가 강제하는 순서](#12-urdf가-강제하는-순서)
  - [1.3 한 origin에 몰아넣으면 생기는 일](#13-한-origin에-몰아넣으면-생기는-일)
- [2. 해법: 조인트 하나를 둘로 분해](#2-해법-조인트-하나를-둘로-분해)
  - [2.1 교환법칙이라는 열쇠](#21-교환법칙이라는-열쇠)
  - [2.2 dh\_revolute 매크로](#22-dh_revolute-매크로)
  - [2.3 체인 전개 결과](#23-체인-전개-결과)
- [3. 기구학과 시각화의 분리](#3-기구학과-시각화의-분리)
- [4. 규약과 함정](#4-규약과-함정)
- [5. 검증](#5-검증)

---

## 1. 문제: 두 규약의 순서 불일치

### 1.1 DH가 요구하는 순서

standard DH의 링크 변환은 네 기본 변환의 곱 ([robot_kinematics.md 1.2절](robot_kinematics.md#12-링크-변환-행렬-유도)).

$$
{}^{i-1}T_i = R_z(\theta_i)\, T_z(d_i)\, T_x(a_i)\, R_x(\alpha_i)
$$

여기서 **$\theta$가 맨 앞**이라는 사실이 핵심. 관절 회전이 링크 기하($a, \alpha$)보다 **먼저** 적용됨.

### 1.2 URDF가 강제하는 순서

URDF `<joint>`는 두 조각으로 구성.

```xml
<joint name="..." type="revolute">
  <origin xyz="..." rpy="..."/>   <!-- 부모 → 조인트 프레임 (고정) -->
  <axis xyz="0 0 1"/>             <!-- 이 축 둘레로 관절값만큼 회전 -->
</joint>
```

부모 링크에서 자식 링크까지의 변환은 항상 이 순서.

$$
{}^{\text{parent}}T_{\text{child}} = \underbrace{\text{Trans}(xyz) \cdot R_{rpy}}_{\text{origin (고정)}} \cdot \underbrace{R_{\text{axis}}(\theta)}_{\text{관절 변수}}
$$

**origin이 먼저, 관절 회전이 나중.** 순서를 바꿀 방법은 없음 — URDF 명세가 그렇게 정의됨.

> `rpy`는 fixed-axis roll-pitch-yaw이며 행렬로는 $R_z(y) R_y(p) R_x(r)$. 테스트의 `_rpy_matrix()`가 이 순서를 그대로 구현.

### 1.3 한 origin에 몰아넣으면 생기는 일

DH 한 행의 $a, d, \alpha$를 조인트 하나의 origin에 다 넣는 것이 자연스러운 첫 시도. `xyz="a 0 d"`, `rpy="α 0 0"`으로 두면

$$
T = \text{Trans}(a, 0, d) \cdot R_x(\alpha) \cdot R_z(\theta)
$$

DH가 요구하는 것과 비교.

$$
\begin{aligned}
\text{필요:}\quad & T_z(d)\, R_z(\theta)\, T_x(a)\, R_x(\alpha) \\
\text{얻음:}\quad & \text{Trans}(a,0,d)\, R_x(\alpha)\, R_z(\theta)
\end{aligned}
$$

$R_x(\alpha)$가 $R_z(\theta)$보다 **먼저** 적용되는 것이 치명적. $\alpha$만큼 이미 기울어진 좌표계에서 z축 회전이 일어나므로 **관절이 엉뚱한 축을 중심으로 회전**. $\alpha = 90°$인 joint 1, 4, 5에서 특히 크게 어긋남.

$T_x(a)$의 위치도 틀림. DH에서는 $\theta$ 회전 **뒤에** x offset이 붙어 회전과 함께 돌아야 하는데, origin에 넣으면 회전 전에 고정됨.

$\alpha = 0$이고 $a = 0$인 관절에서는 우연히 맞아떨어지므로 **일부 자세에서만 맞고 나머지는 틀리는** 형태로 나타남. 눈으로 잡기 어려운 종류의 버그.

---

## 2. 해법: 조인트 하나를 둘로 분해

### 2.1 교환법칙이라는 열쇠

$R_z(\theta)$와 $T_z(d)$는 **둘 다 같은 z축에 대한 조작이라 교환 가능**.

$$
R_z(\theta)\, T_z(d) = T_z(d)\, R_z(\theta)
$$

따라서 DH 변환을 이렇게 다시 쓸 수 있음.

$$
{}^{i-1}T_i = \underbrace{T_z(d)\, R_z(\theta)}_{\text{revolute joint}} \cdot \underbrace{T_x(a)\, R_x(\alpha)}_{\text{fixed joint}}
$$

좌우 두 덩어리가 각각 **URDF 조인트 하나의 형태**와 정확히 일치.

- 왼쪽: origin = $T_z(d)$, axis = z → **revolute joint**
- 오른쪽: origin = $T_x(a) R_x(\alpha)$, 관절 변수 없음 → **fixed joint**

$T_x(a) R_x(\alpha)$는 URDF origin 문법 `xyz="a 0 0" rpy="α 0 0"`으로 그대로 표현됨 (origin이 `Trans` 후 `Rot` 순서이므로).

두 조인트 사이에는 링크가 하나 필요 — URDF의 조인트는 반드시 부모와 자식 링크를 가지며 조인트끼리 직접 이을 수 없음. 그래서 기하도 시각 요소도 없는 **virtual link**를 삽입.

### 2.2 dh_revolute 매크로

```xml
<xacro:macro name="dh_revolute" params="name parent child a d alpha">

  <!-- 1) Revolute joint : Tz(d) 이동 후 z축 θ 회전 -->
  <joint name="${name}" type="revolute">
    <parent link="${parent}"/>
    <child  link="${name}_virtual"/>
    <origin xyz="0 0 ${d}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.283" upper="6.283" effort="200" velocity="3.0"/>
  </joint>

  <!-- 2) Virtual link : 프레임 분리 전용, 시각 요소 없음 -->
  <link name="${name}_virtual"/>

  <!-- 3) Fixed joint : Tx(a) 이동 후 x축 α 회전 -->
  <joint name="${name}_fixed" type="fixed">
    <parent link="${name}_virtual"/>
    <child  link="${child}"/>
    <origin xyz="${a} 0 0" rpy="${alpha} 0 0"/>
  </joint>

</xacro:macro>
```

세 요소가 각각 수식 한 조각을 담당.

| 요소                                   | 수식 조각              | 비고                       |
| -------------------------------------- | ---------------------- | -------------------------- |
| revolute origin `xyz="0 0 d"`          | $T_z(d)$               | rpy는 항상 0               |
| revolute axis `xyz="0 0 1"`            | $R_z(\theta)$          | $\theta$는 런타임 관절값   |
| virtual link                           | —                      | 두 조인트를 잇기 위한 자리 |
| fixed origin `xyz="a 0 0" rpy="α 0 0"` | $T_x(a)\, R_x(\alpha)$ | 관절값 없이 고정           |

매크로 인자에 $\theta$가 없는 점에 주목. revolute의 $\theta$는 모델 상수가 아니라 `/joint_states`로 들어오는 **변수**이기 때문. `dh.py` 테이블에 $\theta$ 열이 없는 것과 같은 이유.

### 2.3 체인 전개 결과

매크로를 6번 호출하면 URDF 체인이 완성.

```
base_link
  └─[link_1_joint    revolute]→ link_1_joint_virtual
      └─[link_1_joint_fixed  fixed]→ link1
          └─[link_2_joint    revolute]→ link_2_joint_virtual
              └─ ...
                  └─ link6
                      └─[tool0_fixed  fixed]→ tool0
```

`base_link → tool0` 전체 곱을 펼치면

$$
\big[T_z(d_1) R_z(\theta_1)\big]\big[T_x(a_1) R_x(\alpha_1)\big] \cdots \big[T_z(d_6) R_z(\theta_6)\big]\big[T_x(a_6) R_x(\alpha_6)\big]
$$

가 되고, 인접한 대괄호를 다시 묶으면 DH 누적곱과 **항등적으로 동일**. 즉 `fk.py`가 계산하는 행렬과 `robot_state_publisher`가 `/tf`로 방송하는 변환이 같은 값.

`tool0`는 기하 없는 순수 참조 프레임. `link6`와 offset 0으로 붙어 있어 지금은 같은 위치지만, 툴(gripper 등)이 붙으면 이 조인트 origin만 고치면 되는 구조.

---

## 3. 기구학과 시각화의 분리

이 모델 설계에서 실무적으로 가장 중요한 원칙.

| 태그                          | 역할                                     | FK/IK 영향 |
| ----------------------------- | ---------------------------------------- | ---------- |
| `<joint>`의 `origin`, `axis`  | **기구학** — DH 값 그 자체               | **있음**   |
| `<link>`의 `<visual><origin>` | **메쉬 정렬** — DAE를 링크 프레임에 맞춤 | **없음**   |

CAD에서 뽑은 메쉬는 원점과 축 방향이 DH 프레임과 일치하지 않는 것이 보통. UR10e도 그래서 링크마다 보정값이 붙어 있음.

```xml
<link name="link2">
  <visual>
    <origin xyz="0.6127 0 0.1762" rpy="1.5708 0 -1.5708"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/link2.dae"/>
    </geometry>
  </visual>
</link>
```

`0.6127`이 $|a_2|$와 같은 값이라 기구학처럼 보이지만 **아님**. 메쉬 원점이 링크 반대쪽 끝에 있어 되돌리는 것뿐.

이 분리의 효과.

- 메쉬가 어긋나 보이면 `visual origin`만 수정. **계산 결과는 절대 변하지 않음**
- 반대로 FK가 틀리면 `visual origin`은 볼 필요 없음. DH 값과 조인트 구조만 의심
- `<visual>`을 통째로 지워도 기구학은 그대로 동작

디버깅 시 **화면이 이상한 것**과 **계산이 이상한 것**을 분리해서 볼 수 있다는 뜻.

---

## 4. 규약과 함정

**값의 이중화.** DH 상수가 두 곳에 존재.

| 위치                                      | 형태                                          |
| ----------------------------------------- | --------------------------------------------- |
| `robot_description/urdf/ur10e.urdf.xacro` | `<xacro:property name="a2" value="-0.6127"/>` |
| `robot_kinematics/robot_kinematics/dh.py` | `UR10E_DH` 배열                               |

**한쪽만 고치면 RViz 화면과 계산 결과가 어긋남.** ROS 패키지 의존성 방향 때문에 (순수 numpy 코어는 `robot_description`을 import하지 않음) 하나로 합치지 않았고, 대신 [5절](#5-검증)의 테스트로 불일치를 검출.

**virtual link가 `/tf`에 노출.** `link_N_joint_virtual` 프레임 6개가 TF 트리에 추가됨. RViz의 TF 표시에서 프레임 수가 예상보다 많아 보이는 이유이며 정상.

**관절 한계가 넉넉함.** `lower="-6.283" upper="6.283"` ($\pm 2\pi$)는 UR 실제 스펙 반영. IK 반환값에 별도 제한이 없다는 점과 합쳐지면 실로봇 연동 시 wrapping 처리가 필요해짐 ([robot_kinematics.md 4.6절](robot_kinematics.md#46-ik-사용-시-주의점)).

**동역학 정보 없음.** `<inertial>`, `<collision>` 태그를 두지 않음. 기구학 검증과 RViz 시각화가 목적이라 불필요. Gazebo 연동 단계에서는 추가 필수.

---

## 5. 검증

"URDF 체인 = DH 누적곱"이라는 주장은 `robot_kinematics/test/test_fk.py`의 `TestAgainstURDF`가 직접 확인.

```python
def _urdf_chain_fk(root, q):
    """Compose base_link→tool0 by walking joint parents back from tool0."""
    # tool0에서 부모를 거슬러 올라가 체인을 만든 뒤 역순으로 합성
    for j in chain:
        T = T @ step                      # origin: Trans(xyz) · R(rpy)
        if j.attrib["type"] == "revolute":
            T = T @ rot                   # axis 둘레 관절값 회전
```

핵심은 이 함수가 **DH를 전혀 모른다는 것**. URDF 명세대로 origin과 axis만 순서대로 합성하므로 `dh.py` 계산과 독립적인 정답 역할을 수행.

```python
q = rng.uniform(-2 * np.pi, 2 * np.pi, 6)
T_dh = fk(q)
T_urdf = _urdf_chain_fk(root, q)
np.testing.assert_allclose(T_dh[:3, 3], T_urdf[:3, 3], atol=1e-6)
np.testing.assert_allclose(T_dh[:3, :3], T_urdf[:3, :3], atol=1e-6)
```

무작위 관절각 100개, 위치·자세 모두 $10^{-6}$ 이내. 이 테스트가 잡아내는 것.

- 조인트 분해 순서 오류 ([1.3절](#13-한-origin에-몰아넣으면-생기는-일)의 실패 모드)
- xacro property와 `dh.py` 값 불일치
- `rpy` 회전 순서 착각

관절 범위를 $\pm 2\pi$로 잡은 것도 의도적. 좁은 범위에서만 돌리면 $\alpha = 0$ 구간의 오류가 숨을 수 있음.

> 이 테스트는 `xacro` 파이썬 모듈에 의존. 없으면 `pytest.importorskip`으로 조용히 건너뛰므로 **프로젝트 컨테이너 안에서 `test-kinematics`로 실행해야** 실제 검증이 이뤄짐. 호스트에서 통과했다고 안심하면 안 되는 부분.
