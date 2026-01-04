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
```Plaintext
.
├── README.md               # 프로젝트 전체 개요 및 로드맵
├── daily/                  # 날짜별 학습 일지 (TIL)
├── docs/                   # 계층별 이론 분석 문서 (Markdown)
│   ├── 01_Architecture.md
│   ├── 02_RMW_Layer.md
│   └── 03_Client_Libraries.md
├── src_analysis/           # 실제 ROS 2 소스 코드 라인별 분석 및 주석
│   ├── rclcpp/             # C++ 클라이언트 라이브러리 분석
│   ├── rcl/                # 공통 C 라이브러리 분석
│   └── rmw/                # 미들웨어 인터페이스 분석
├── experiments/            # 시스템 성능 및 동작 분석 실험 코드
│   ├── latency_test/       # 통신 지연 시간 측정
│   └── scheduling_test/    # OS 스케줄러 상호작용 테스트
├── references/             # 참고한 논문 PDF나 링크 정리
└── scripts/                # 빌드 및 분석 자동화 스크립트
```

---
*Maintained by [Kim, Minseong/felixmskim] @ CS LAB.*
