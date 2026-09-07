# 기구학 문서<!-- omit from toc -->

**코드에 구현된 알고리즘의 이론적 배경과 유도 과정을 코드와 나란히 놓은 학습 문서**

이 프로젝트의 FK / Jacobian / IK / 궤적은 라이브러리 없이 numpy로 직접 구현돼 있음.  
코드만 읽으면 "무엇을 하는지"는 보여도 "왜 이 식인지"는 보이지 않으므로, 각 수식의 유도부터 코드 한 줄까지를 연결해 정리한 것임.

- [문서 목록](#문서-목록)
- [읽는 순서](#읽는-순서)
- [문서와 코드 대응](#문서와-코드-대응)
- [문서 구성 방식](#문서-구성-방식)
- [표기 규약](#표기-규약)

---

## 문서 목록

| 문서                                         | 다루는 내용                                                               |
| -------------------------------------------- | ------------------------------------------------------------------------- |
| [robot_description.md](robot_description.md) | DH 파라미터를 URDF로 옮기는 방법. 변환 순서 불일치 문제와 조인트 2단 분해 |
| [robot_kinematics.md](robot_kinematics.md)   | DH 링크 변환, FK 누적곱, 기하학적 Jacobian, DLS IK                        |
| [robot_trajectory.md](robot_trajectory.md)   | 5차 다항식 궤적, SO(3) interpolation(SLERP), 직선/원호 pose 경로, seed IK |

`robot_bringup`(데모 시퀀스, JointState 스트리밍)은 알고리즘이 아니라 조립 계층이라 제외했으며, 해당 패키지의 설명은 코드와 [README](../README.md)에 있음.

## 읽는 순서

처음 보는 경우 **robot_kinematics, robot_trajectory, robot_description**의 순서를 권장함.

```
robot_kinematics.md     DH → FK → Jacobian → IK
        │                (뒤 절이 앞 절을 그대로 사용하는 구조)
        ▼
robot_trajectory.md     IK를 반복 호출해 경로를 만드는 계층
        │
        ▼
robot_description.md    같은 DH를 URDF/RViz 쪽에서 다시 보는 관점
```

`robot_description.md`를 뒤에 둔 이유는 DH 변환 순서를 이미 알고 있어야 URDF 분해의 필요성이 이해되기 때문임.  
URDF 모델링만 필요하다면 이 문서만 따로 읽어도 됨.

## 문서와 코드 대응

| 코드                                      | 문서 절                                                                                                        |
| ----------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `robot_description/urdf/ur10e.urdf.xacro` | [_robot description.md_ 2\_조인트 분해](robot_description.md#2-해법-조인트-하나를-둘로-분해)                   |
| `robot_kinematics/dh.py`                  | [_robot kinematics.md_ 1_DH 파라미터](robot_kinematics.md#1-dh-파라미터-dhpy)                                  |
| `robot_kinematics/fk.py`                  | [_robot kinematics.md_ 2_FK](robot_kinematics.md#2-fk-fkpy)                                                    |
| `robot_kinematics/jacobian.py`            | [_robot kinematics.md_ 3_Jacobian](robot_kinematics.md#3-jacobian-jacobianpy)                                  |
| `robot_kinematics/ik.py`                  | [_robot kinematics.md_ 4_IK](robot_kinematics.md#4-ik-ikpy)                                                    |
| `robot_trajectory/joint_traj.py`          | [_robot trajectory.md_ 1_5차 다항식 궤적](robot_trajectory.md#1-5차-다항식-궤적-joint_trajpy)                  |
| `robot_trajectory/cartesian_traj.py`      | [_robot trajectory.md_ 2~4_SO(3) interpolation 이후](robot_trajectory.md#2-so3-interpolation-cartesian_trajpy) |

## 문서 구성 방식

각 개념은 아래의 5단 구조를 따름.

1. **문제** : 무엇을 왜 계산하는가
2. **유도** : 결과 식만 던지지 않고 단계별로
3. **코드 대응** : 실제 발췌와 수식의 대조
4. **주의점** : 인덱스 규약, 행 순서 등 틀리기 쉬운 지점
5. **검증** : 해당 pytest가 무엇을 보장하는가

5번을 매 절에 둔 이유는, 이 프로젝트의 테스트가 단순 회귀 방지가 아니라 **유도가 맞았는지를 확인하는 장치**이기 때문임.  
예를 들어 Jacobian의 열 공식은 수치미분 대조로, 5차 다항식의 peak 계수는 limit 검사로 검증됨.

## 표기 규약

- **수식** : GitHub 마크다운 LaTeX 렌더링을 기준으로 하며, 인라인은 `$...$`, 블록은 `$$...$$`를 씀
- **용어** : 해당 분야에서 영어로 통용되는 것은 영어 그대로 사용하고 (FK, IK, pose, waypoint, seed, revolute, SLERP, interpolation, singularity 등), 한국어가 표준인 것만 한국어로 씀 (관절각, 동차변환, 특이점 등)
- **코드 참조** : 리팩터링에 흔들리지 않도록 줄 번호 대신 **함수명 기준**으로 적고(`fk.py`의 `fk_frames()`), 코드 인용은 전문이 아니라 핵심 몇 줄만 발췌함
- **기호** : [_robot kinematics.md_ 0\_전체 구성](robot_kinematics.md#0-전체-구성)에 공통 기호표가 있음
