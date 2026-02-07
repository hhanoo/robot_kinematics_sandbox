# robot_kinematics_sandbox

ROS 2 Humble 기반 **로봇 기구학 및 궤적 생성 검증용 샌드박스 프로젝트**

CAD(STL)와 DH 파라미터를 기반으로 로봇을 직접 모델링하고,  
MoveIt에 의존하지 않고 구현한 FK / IK / Trajectory를  
RViz 상에서 시각적으로 검증하는 것을 목표로 한다.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-blue)](https://isocpp.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-brightgreen)](docker/)

<!-- [![License](https://img.shields.io/badge/License-MIT-orange)](LICENSE) -->

## 📋 목차

- [데모](#데모)
- [개요](#개요)
- [주요 기능](#주요-기능)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [설치](#설치)
- [빌드](#빌드)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [ROS2 인터페이스](#ros2-인터페이스)
- [문제 해결](#문제-해결)
- [자주 묻는 질문](#자주-묻는-질문)
- [로드맵](#로드맵)
- [라이선스](#라이선스)

---

## 데모

<!-- 추후 RViz 시각화 스크린샷 또는 GIF 추가 -->

### 시스템 구조

```

robot_kinematics_sandbox/
├── src/
│   ├── robot_description/     # URDF / xacro / STL
│   ├── robot_kinematics/      # FK / IK / Jacobian
│   ├── robot_trajectory/      # Trajectory generation
│   └── robot_bringup/         # joint_state, RViz

```

### 스크린샷

<!-- RViz 화면 이미지 추가 예정 -->

---

## 개요

### 프로젝트 목적

이 프로젝트는 특정 로봇이나 특정 플래너에 종속되지 않는  
**범용 로봇 기구학 및 궤적 생성 실험용 기준 프로젝트**이다.

다음과 같은 상황을 전제로 한다.

- CAD(STL) 파일은 제공되지만 ROS description은 없는 경우
- DH 파라미터 또는 기구학 정의는 확보되어 있는 경우
- MoveIt 없이 직접 기구학/궤적 알고리즘을 구현해야 하는 경우

본 프로젝트는 아래 파이프라인을 검증하는 것을 목표로 한다.

```

CAD (STL) + DH parameters
↓
URDF / xacro (custom description)
↓
FK / IK / Trajectory (custom implementation)
↓
JointState streaming
↓
RViz visualization

```

### 주요 구성요소

- **robot_description**: URDF / xacro 기반 로봇 모델 정의
- **robot_kinematics**: DH 기반 FK / IK / Jacobian
- **robot_trajectory**: Cartesian / Joint trajectory 생성
- **robot_bringup**: RViz 시각화 및 테스트 실행 환경

### 적용 가능 영역

- 산업용 매니퓰레이터 기구학 검증
- CAD 기반 로봇 모델 초기 검증
- 커스텀 궤적 생성 알고리즘 테스트
- 연구 및 교육 목적의 로봇 시뮬레이션

---

## 주요 기능

<!-- 추후 기능 정리 -->

---

## 빠른 시작

<!-- 초기 bringup 완성 후 작성 -->

---

## 시스템 요구사항

<!-- 추후 정리 -->

---

## 설치

<!-- 추후 정리 -->

---

## 빌드

<!-- 추후 정리 -->

---

## 실행

<!-- 추후 정리 -->

---

## 사용법

<!-- 추후 정리 -->

---

## 설정

<!-- 추후 정리 -->

---

## ROS2 인터페이스

<!-- 추후 토픽 / 노드 / 파라미터 정리 -->

---

## 문제 해결

<!-- 추후 트러블슈팅 정리 -->

---

## 자주 묻는 질문

<!-- 추후 정리 -->

---

## 로드맵

- [ ] DH 기반 가상 6축 로봇 URDF 작성
- [ ] STL 기반 UR10e 모델 정렬
- [ ] 수치 IK (DLS) 구현
- [ ] Cartesian trajectory → joint trajectory 변환
- [ ] JointState 기반 RViz 재생
- [ ] 실로봇 연동 인터페이스 정리

---

## 라이선스

이 프로젝트는 - 라이선스로 배포됩니다 - 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

**관리자**: hhanoo (woo980711@gmail.com)
