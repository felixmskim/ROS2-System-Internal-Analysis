# 1. WSLg 설치 및 방법 확인
WSLg는 ***WSL GUI***의 약자로, WSL2의 커널 및 가상화 스택을 활용해 리눅스 GUI 앱을 윈도우 창으로 띄워주는 기술이다.

다음 기술에 따라 WSLg를 설치하자.

1. Powershell이나 CMD를 '관리자 권한'으로 실행하자.
2. 다음 명령어 입력
```PoswerShell
# wsl 최신 버전으로 업데이트
wsl --shutdown
```
3. 업데이트가 완료되면 WSL 완전히 종료했다가 켜야한다.
```PowerShell
wsl --shutdown
```
4. WSLg 동작을 확인해주자. WSL(Ubuntu) 터미널을 열어서 확인하자.
```bash
sudo apt update
sudo apt install x11-aps -y
xeyes
```

WSLg를 다운로드 받기위해 Microsoft 공식 홈페이지에 가니 다음과 같은 문구가 있었다.


# 2. vGPU(가상 GPU) 드라이버 설치

"Linux GUI 앱을 실행하려면 먼저 아래 시스템과 일치하는 드라이버를 설치해야 합니다. 이렇게 하면 하드웨어 가속 OpenGL 렌더링을 활용할 수 있도록 vGPU(가상 GPU)를 사용할 수 있습니다."

**vGPU 드라이버란?**
기본적으로 WSL2는 가상화된 환경이라서, 리눅스 안에서 돌아가는 앱(예: Gazebo)은 윈도우에 꽂힌 그래픽 카드가 무엇인지 직접 알 수 없다. 그래서 드라이버가 없으면 모든 그래픽 계산을 CPU가 대신하게 된다 (Software Rendering).

**vGPU (GPU Paravirtualization)**는 윈도우 호스트의 GPU 드라이버가 가상화 계층을 통해 리눅스 커널에 GPU 자원을 노출해주는 기술이다.

D3D12 매핑은 리눅스의 그래픽 표준인 OpenGL이나 Vulkan 명령어를 윈도우의 DirectX 12 명령어로 변환해서 하드웨어 가속을 실현한다.

이는 CPU 성능이 여유롭지 않은 노트북에 사용하기에 매우 안성맞춤이라고 한다. 시뮬레이션의 그래픽 계산을 GPU로 넘겨주면, CPU는 ROS 노드의 로직 처리나 시스템 분석에 ㅈㄴ념할 수 있어서 전체적인 시스템 반응 속도가 훨씬 빨라지고,  배터리 및 발열에도 매우 좋다고 한다.

[!CAUTION] 주의할 점: 리눅스(Ubuntu) 안에서 `sudo apt install nvidia-driver-...` 같은 명령어로 리눅스용 드라이버를 직접 설치하면 절대로 안된다. 윈도우 호스트에 최신 드라이버를 깔면 WSL2가 알아서 인식하는 방식이다.

## 설치 방법
1. 드라이버를 업데이트해준다. 나는 Intel 그래픽 드라이버를 사용하므로 [여기](https://www.intel.co.kr/content/www/kr/ko/download-center/home.html)에서 설치해줬다. 이미 최신 버전이라면 넘어가도 좋다.




이어서 계속... 제미나이에서 "터미널(Ubuntu)에서 아래 명령어를 쳤을 때 제조사 이름이 나오는지 확인해봐." 검색하면 나올것




# ROS2 로봇 시뮬레이션 종류
1. Turtlesim-가장 가벼운 2D 시뮬레이터-Node 통신 분석의 정석. IPC(Inter-Process Communication) 구조를 분석하기에 가장 노이즈가 적음.
3. Webots-효율적인 3D 시뮬레이터-메모리 효율성. 상대적으로 적은 자원으로 복잡한 로봇 시뮬레이션이 가능해 자원 제약 환경 분석에 유리.

```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim -y

# 실행 확인 (WSLg 창이 떠야 함)
ros2 run turtlesim turtlesim_node
```

```bash

```