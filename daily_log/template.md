# 📅 2026-01-XX

## 🎯 목표
- 리눅스 커널 빌드 환경 구축 및 첫 번째 모듈 작성

## 📝 학습 내용
- [이론] 리눅스 커널과 사용자 공간의 차이점
- [실습] `hello.c` 커널 모듈 작성 및 `insmod` 테스트

## 🔍 소스코드 분석: `task_struct`
- **파일 위치:** `include/linux/sched.h` (Line 728~)
- **핵심 코드:**
  ```c
  struct task_struct {
      volatile long state;    // 프로세스 상태
      struct mm_struct *mm;   // 메모리 관리 정보
      // ...
  };
  ```

## 🔍 직면한 문제 & 해결 (Troubleshooting)
- **문제:** `make` 실행 시 `kernel headers`를 찾을 수 없다는 오류 발생
- **원인:** 현재 실행 중인 커널 버전과 헤더 버전이 불일치함
- **해결:** `sudo apt install linux-headers-$(uname -r)` 명령어로 해결

## 💡 인사이트 (오늘 깨달은 것)
- 커널 모듈은 `printf`가 아니라 `printk`를 써야 한다는 점이 신기했다.

## 🔗 참고 자료
- [Blog] mythos.log: Linux Tutorial #1 (부팅하기)
- [Book] 리눅스 커널 소스 해설: 기초입문 (정재준 저)