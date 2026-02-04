ROS2 Jazy 환경에 효율적인 개발을 위한 **Workspace** 를 구축하고, ROS2의 전용 빌드 시스템인 **colcon** 빌드 시스템을 통해 패키지를 컴파일하는 표준 절차를 정의한다. 컨테이너를 생성할 때마다 깔끔한 환경에서 워크스페이스를 초기화하는 과정을 포함한다.


# 1. Update Environment Configuration
다음은 업데이트된 Dockerfile을 명시하겠다. 이 환경은 GUI 개발 도구인 `QtCreator`와 이전에 다룬 vGPU 가속 설정을 포함한다.

### Dockerfile (Fixed Version)

```Dockerfile
# 1. 베이스 이미지: ROS2 Jazzy 데스크탑 버전 (Ubuntu 24.04 기반)
FROM osrf/ros:jazzy-desktop

# 2. 기본 쉘 및 환경 변수 설정
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 3. 필수 패키지 및 저장소 설정
# software-properties-common: universe 저장소 추가를 위한 도구
# qtcreator: IDE 기반의 시스템 분석 및 개발 환경
RUN apt-get update && apt-get install -y \
    locales sudo vim git python3-pip \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
    qtcreator build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# 4. 로캘(Locale) 설정 (UTF-8 환경 보장)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 5. GPU 및 GUI 가속 설정 (WSL/Windows 11 Intel Arc vGPU)
ENV MESA_D3D12_DEFAULT_ADAPTER_NAME=Arc
ENV GALLIUM_DRIVER=d3d12

# 6. DDS 및 QoS 환경 설정 (블로그 기반 최적화 반영)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0

# 7. 자동 환경 설정 (.bashrc)
# 컨테이너 접속 시 ROS2 및 빌드된 워크스페이스 환경 자동 로드
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
```

### Execution Command

호스트(Windows 11)와 컨테이너(Ubuntu 24.04) 간의 자원 공유 및 소스 코드 영속성을 위해 다음과 같은 명령어로 컨테이너를 실행한다.

```bash
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

# 2. Manual Workspace Initialization

컨테이너 접속한 후, 워크스페이스를 생성한다. 이는 각 프로젝트마다 독립된 환경을 파악하기 위함이다.

```Bash
# 워크스페이스 디렉토리 생성 (표준 구조: ~/ros2_ws/src)
mkdir -p ~/ros2_ws/src

# 워크스페이스 이동
cd ~/ros2_ws
```

- `src/`: 모든 소스 코드가 위치하는 곳이다. 이후 개발할 패키지들은 이 폴더 안에 생성된다.

# 3. Build Mechanism: Colcon Build

ROS2는 `colcon`을 통해 여러 패키지의 의존성을 파악하고 병렬 빌드를 수행한다.

```bash
# 워크스페이스 빌드 실행
colcon build --symlink-install
```

의존성 기반 병렬 스케줄링 colcon은 워크스페이스 내 각 패키지의 package.xml을 분석하여 패키지 간의 선후 관계를 정의하는 **방향성 비순환 그래프(DAG, Directed Acyclic Graph)** 를 생성한다. 이 그래프를 바탕으로 상호 의존성이 없는 패키지들을 식별하고, 시스템의 가용한 CPU 코어(Worker)에 적절히 배분하여 병렬 컴파일을 수행함으로써 전체 빌드 시간을 최적화한다. 이는 하드웨어 자원의 가동률을 극대화하여 대규모 시스템 소프트웨어를 효율적으로 빌드하기 위한 멀티 스레드 스케줄링 전략의 핵심이다.

**🔗 `--symlink-install` 옵션의 이해**

보통 colcon build를 실행하면 src에 있는 소스 파일이나 설정 파일들을 install 폴더로 **복사(Copy)** 하게 된다. 하지만 `--symlink-install` 옵션을 사용하면 복사 대신 원본 파일을 가리키는 **심볼릭 링크(Symbolic Link)** 를 생성한다.

- **왜 사용하나요? (효율성)**

    - **재빌드 생략:** 파이썬(Python) 코드나 런치(Launch) 파일, YAML 설정 파일 등을 수정했을 때, 다시 colcon build를 치지 않아도 변경 사항이 즉시 시스템에 반영된다.

    - **시간 단축:** 수정 후 바로 실행(Run)만 하면 되므로, 반복적인 테스트가 필요한 개발 단계에서 비약적인 속도 향상을 가져온다.

- 주의사항: * C++ 코드처럼 컴파일이 필요한 소스는 수정한 뒤 반드시 다시 빌드해야 한다 (바이너리 파일 자체가 바뀌어야 하기 때문). 하지만 C++ 패키지 내의 share 폴더(런타임 설정 등)는 여전히 링크 덕분에 재빌드 없이 수정이 가능하다.

빌드에 문제가 없으면 하위 폴더에 `/build`, `install`, `src/`, `log` 디렉토리가 생성된다.

### Core Concepts of Colcon Build:
```Plaintext
ros2_ws/
├── src/            # (User) 소스 코드, 패키지들이 위치
├── build/          # (Colcon) 중간 빌드 파일 (CMake/Python 캐시 등)
├── install/        # (Colcon) 최종 실행 파일, 라이브러리, 셋업 스크립트
└── log/            # (Colcon) 각 패키지 빌드 로그
```

1. `build/`: 컴파일 과정에서 생성되는 중간 오브젝트 파일들이 위치한다.
2. `install/`: 빌드 결과물(실행 파일, 라이브러리)이 설치된다. 이 안의 `setup.bash`를 통해 환경 변수를 로드한다.
3. `log/`: 빌드 이력을 기록하며, 에러 발생 시 디버깅의 핵심 소스가 된다.
4. - `src/`: 모든 소스 코드가 위치하는 곳이다. 이후 개발할 패키지들은 이 폴더 안에 생성된다.
5. `--symlink-install`: 원본 소스 파일과 설치된 파일 간에 심볼릭 링크를 생성하여, 소스 코드 수정 시 재빌드 없이도 변경 사항을 반영(스크립트 언어 등)할 수 있게 한다.

# 4. Environment Soucing Strategy

빌드가 성공적으로 끝나면 생성된 `install/` 디렉토리의 환경을 현재 쉘에 로드해야 한다.

```bash
# 현재 워크스페이스 환경 로드
source install/setup.bash
```

- **Note:** 본 Dockerfile는 `.bashrc` 설정에 의해, 이후 컨테이너를 재접속할 때 `~/ros2_ws/install/setup.bash` 파일이 존재한다면 자동으로 로드되도록 구성되어 있다.

---

# 5. 마무리

이번 포스팅에서는 ROS 2를 사용하기 위한 개발환경 구성을 다뤘다.

다음 포스팅은 ROS 2의 데이터 통신 미들웨어인 DDS에 관하여 다루겠다.

## 🔗 참고 자료
- [Blog] [ROS 2의 소개와 개발환경 구성](https://velog.io/@katinon/ROS2-ROS-2%EC%9D%98-%EC%86%8C%EA%B0%9C%EC%99%80-%EA%B0%9C%EB%B0%9C%ED%99%98%EA%B2%BD-%EA%B5%AC)
- [Blog] [심볼릭 링크 생성, 삭제, 수정](https://bravesuccess.tistory.com/306) 