# ROS2 System Internal Analysis 🤖

> **Chungbuk National University - Computer Systems Lab (CS LAB.)**

>  **Topic:** Code-level analysis of ROS2 from a Systems Software Perspective.

## 📌 Project Overview
이 프로젝트는 ROS2(Robot Operating System)의 내부 구조를 시스템 소프트웨어 관점에서 분석하는 repository입니다. 단순한 API 활용을 넘어, 운영체제와 하드웨어 레벨에서 ROS2가 어떻게 상호작용하는지 심층적으로 탐구합니다.

## 🎯 Research Goals
- **Top-Down Architecture Study:** ROS2의 계층적 구조와 추상화 레이어 이해.
- **Code-Level Deep Dive:** `rclcpp`, `rcl`, `rmw` 등 핵심 레이어의 소스 코드 분석.
- **System Interaction:** 통신 메커니즘(DDS)과 OS 스케줄링, 메모리 관리 간의 관계 분석.
- **Windows Environment Optimization:** 윈도우 환경(WSL2 포함)에서의 최적의 분석 환경 구축.

## 🛠 Analysis Environment
- **Host OS:** Windows 11
- **Subsystem:** WSL2 (Ubuntu 22.04 LTS)
- **Target:** ROS 2 Humble / Iron
- **Tools:** VS Code, GDB, Perf, Valgrind, Docker

## 📂 Directory Structure
- `/docs`: 아키텍처 및 레이어별 이론 정리.
- `/src_analysis`: 주요 소스 코드 분석 및 주석.
- `/experiments`: 성능 및 스케줄링 실험 코드.
- `/profiling`: 하드웨어 자원 점유율 분석 데이터.

---
*Maintained by [Kim, Minseong/felixmskim] @ CS LAB.*
