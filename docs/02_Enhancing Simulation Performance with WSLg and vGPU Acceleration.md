이 챕터에서는 ROS2 시뮬레이션 환경의 시각화와 성능 최적화를 위해 **WSLg(Windows Subsystem for Linux GUI)** 를 설정하고, **vGPU(가상 GPU) 하드웨어 가속** 을 호스트와 **Docker 컨테이너** 양쪽에서 활성화하는 과정을 다룬다.

# 1. WSLg 설치 및 동작 확인
**WSLg** 는 WSL2 커널 및 가상화 스택을 활용해 리눅스 GUI 앱을 윈도우 창으로 직접 띄워주는 기술이다. 이를 통해 Gazebo나 Rviz2 같은 도구를 로컬 앱처럼 사용할 수 있다.

## 1.1 WSL 업데이트 및 재시작
먼저 Windows 호스트의 PowerShell을 '관리자 권한'으로 실행하고 다음 명령어를 입력하자.

``` PowerShell
# WSL 최신 버전으로 업데이트
wsl --update

# 업데이트 적용을 위해 WSL 전체 종료
wsl --shutdown
```

## 1.2 WSLg 동작 테스트
Ubuntu 터미널을 열어 기본적인 GUI 앱이 정상적으로 실행되는지 확인하자.

```Bash
sudo apt update
sudo apt install x11-apps -y

# 눈 모양의 창이 뜨면 성공이다
xeyes
```

# 2. vGPU(가상 GPU) 하드웨어 가속 설정 (Host)

기본적으로 WSL2는 가상화 환경이므로, 별도의 설정이 없으면 모든 그래픽 계산을 CPU가 담당하는 Software Rendering 방식을 사용한다. **vGPU(GPU Paravirtualization)** 기술을 적용하면 윈도우 호스트의 GPU 자원을 리눅스 커널에 노출하여 하드웨어 가속을 실현할 수 있다.

이는 CPU 부하를 줄여 시스템 반응 속도를 높이고, 배터리 효율 및 발열 관리에도 매우 효과적이다.

[!CAUTION] 주의사항: Ubuntu 내부에서 `sudo apt install nvidia-driver-...`와 같은 리눅스용 그래픽 드라이버를 직접 설치해서는 안 된다. 윈도우 호스트에 최신 드라이버를 설치하면 WSL2가 이를 자동으로 인식하는 구조이다.

## 2.1 호스트 드라이버 확인
Windows의 장치 관리자나 dxdiag를 통해 그래픽 드라이버가 다음 조건을 만족하는지 확인하자.

`드라이버 날짜: 2023년 이후 버전 권장`

`드라이버 모델: WDDM 3.0 이상`

## 2.2 Ubuntu 내 Mesa 라이브러리 업데이트

리눅스에서 윈도우 GPU를 호출하려면 그래픽 라이브러리인 Mesa가 최신 버전이어야 한다. d3d12 드라이버가 포함된 최신 패키지를 설치하자.

```Bash
# Mesa 라이브러리 업데이트 (PPA 추가)
sudo add-apt-repository ppa:kisak/kisak-mesa -y
sudo apt update && sudo apt upgrade -y
```

## 2.3 vGPU 가속 활성화 및 고정
시스템이 하드웨어 가속 드라이버를 확실히 사용하도록 강제로 지정하자. 먼저 장치 파일이 존재하는지 확인한다.

```Bash
# /dev/dxg 장치 파일이 보여야 호스트 GPU가 노출된 것이다
ls -l /dev/dxg
```

장치가 인식되었다면, d3d12 드라이버를 기본으로 사용하도록 환경 변수를 설정하자.

```Bash
# 환경 변수 영구 등록
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
source ~/.bashrc

# GPU 장치 파일 존재 확인
ls -l /dev/dxg

# 그래픽 가속 확인 (D3D12 및 내 외장 그래픽 모델명이 떠야 함)
glxinfo -B | grep "OpenGL renderer"
```

출력 결과에 D3D12 (Intel/NVIDIA/AMD...)가 포함되어 있다면 하드웨어 가속이 정상 작동하는 상태이다. 만약 llvmpipe라고 뜬다면 여전히 CPU 렌더링을 사용 중인 것이다.

# 3. Docker 컨테이너에서 GUI 및 GPU 가속 활성화

호스트에서 가속 설정이 끝났다면, 이제 이를 Docker 컨테이너 내부로 전파해야 한다. 컨테이너는 호스트의 장치를 자동으로 공유하지 않으므로 실행 시 별도의 옵션이 필수적이다.

## 3.1 컨테이너 실행 옵션
컨테이너를 실행할 때 다음 세 가지 요소를 반드시 포함해야 한다.

1. 디스플레이 연결: -`e DISPLAY=$DISPLAY` 및 `-v /tmp/.X11-unix:/tmp/.X11-unix`

2. GPU 장치 할당: `--device /dev/dxg` (WSL2에서 GPU를 넘겨주는 핵심 경로)

3. 권한 부여: 호스트 터미널에서 `xhost +local:docker` 실행

## 3.2 최적화된 실행 명령어

`x11-xserver-utils` 패키지를 설치하자.

```bash
sudo apt update
sudo apt install x11-xserver-utils
```

그리고 아래 명령어를 사용하여 컨테이너를 실행하자.

```Bash
# 1. 호스트 터미널에서 외부 장치 접근 허용
xhost +local:docker

# 2. GPU 장치와 디스플레이 소켓을 마운트하여 컨테이너 실행
docker run -it --name ros2_jazzy_container \
  --net=host \
  --device /dev/dxg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -e MESA_D3D12_DEFAULT_ADAPTER_NAME=Arc \
  my_ros2_jazzy:latest
```

- `--device /dev/dxg`: 컨테이너가 호스트의 GPU 가속 장치에 접근하게 한다.

- `-v /usr/lib/wsl:/usr/lib/wsl`: WSL2 전용 GPU 라이브러리들을 컨테이너와 공유한다.

- `LD_LIBRARY_PATH`: 컨테이너 내부에서 공유된 GPU 라이브러리를 찾을 수 있도록 경로를 지정한다.

## 3.3 컨테이너 내부 가속 확인
컨테이너 안으로 접속한 뒤, 2.2에서 다룬 명령어를 실행하여 필요한 것들을 다운로드 받자.

그리고 호스트에서 했던 것과 마찬가지로 가속 여부를 확인하자.

``` Bash
# 컨테이너 내부 터미널
glxinfo -B | grep "OpenGL renderer"
```

여기서도 D3D12 (Intel(R) Arc(TM) A350M Graphics)와 같이 외장 그래픽 카드가 정확히 명시되어야 한다.

잘 안된다면 다음 내용을 참고하자.

Mesa의 D3D12 드라이버는 사용할 그래픽 카드의 이름을 필터링할 수 있는 환경 변수를 제공한다. 컨테이너 내부 터미널에서 아래 명령어를 입력해 보자.

```bash
# 'Arc'라는 단어가 포함된 어댑터를 우선적으로 사용하도록 지정
export MESA_D3D12_DEFAULT_ADAPTER_NAME=Arc
export GALLIUM_DRIVER=d3d12

# 확인
glxinfo -B | grep "OpenGL renderer"
```



## 3.4.1 Dockerfile에 영구 적용하기

매번 컨테이너를 만들 때마다 설치하기 귀찮으니까, chapter01에서 작성했던 `Dockerfile`의 `Run` 항목에 `mesa-utils`를 추가하자

```bash
# ... 생략 ...
RUN apt-get update && apt-get install -y \
    locales \
    sudo \
    vim \
    git \
    mesa-utils \  # 여기에 추가!
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
# ... 생략 ...

# ... (기존 설정들) ...

# 외장 그래픽(Arc) 강제 지정 환경 변수 추가
ENV MESA_D3D12_DEFAULT_ADAPTER_NAME=Arc
ENV GALLIUM_DRIVER=d3d12

# ...
```

# 4. GPU 성능 우선순위 설정 (노트북 사용자 권장)
노트북 환경에서는 전력 절약을 위해 내장 그래픽이 우선 선택될 수 있다. 성능이 더 좋은 외장 GPU(예: Intel Arc A350M 등)를 사용하도록 설정하자.

1. **윈도우 설정 → 시스템 → 디스플레이 → 그래픽** 으로 이동한다.

2. 목록에서 `C:\Windows\System32\wsl.exe`를 추가한다. (목록에 이미 있다면 클릭한다.)

3. 옵션을 클릭하고 [고성능] (외장 GPU)으로 지정한 뒤 저장한다.

4. 커널 로그를 통한 최종 무결성 검사
하드웨어 초기화 단계에서 에러가 없는지 커널 로그(dmesg)를 통해 최종 확인하자.

```Bash
dmesg | grep -i dxg
```

`Initialized dxgk`와 같은 메시지가 보인다면 하드웨어 가속을 위한 모든 준비가 끝난 것이다.

이제 고성능 그래픽 가속이 준비되었으므로, 다음 챕터에서 본격적인 시뮬레이션 도구들을 설치하고 구동해 보자.


![check for xeyes in container](./assets/image/ch02/check%20xeyes%20in%20container.png)

(마지막으로 container에서도 잘 동작한다는 것을 보여주고 마무리짓겠다)

## 🔗 참고 자료
- [wikidocs] 우분투와 WSL (2.3. WSL에서 GUI 사용)
- [Microsoft Learn] Linux용 Windows 하위 시스템의 Linux GUI 앱 실행