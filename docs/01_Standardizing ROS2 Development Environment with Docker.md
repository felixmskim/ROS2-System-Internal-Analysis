# 0. ROS2 개발에 Docker가 필수일까?
**Docker**란 한마디로 소프트웨어를 담는 표준화된 화물 컨테이너이다.

과거에는 배에 물건을 실을 때 크기도 모양도 제각각이라 싣기가 힘들었다. 하지만 **container**라는 규격이 생기면서 어떤 배든, 어떤 항구든 똑같이 물건을 옮길 수 있게 되었다.

컴퓨터에서도 마찬가지이다. 내 컴퓨터(Ubuntu 24.04) 환경, 라이브러리 버전, 설정값들을 통째로 하나의 **image**로 만들고, 이를 **container**라는 격리된 공간에서 실행하는 기술이다.

- **image:** 실행에 필요한 모든 파일과 설정이 담긴 "설계도" 혹은 "설치 CD"

- **container:** 이미지를 실행시킨 "실제 상태". 운영체제 커널은 공유하면서 파일 시스템과 프로세스만 격리된 가벼운 공간

---

로컬에서 ROS2를 직접 설치하는 것보다 Docker를 사용하는 것이 권장된다. 왜 그럴까?

**1. Dependency Hell 방지:** ROS2는 수백 개의 라이브러리에 의존한다. 만약 다른 프로젝트를 위해 특정 라이브러리를 업데이트했다가 ROS2 Jazzy와 충돌한다면 시스템 전체가 꼬여버릴 수 있다. Docker를 쓰면 각 프로젝트가 독립된 컨테이너에서 돌아가므로 호스트 시스템을 깨끗하게 유지할 수 있다.

**2."내 컴퓨터에선 되는데?" 해결:** 연구실 동료나 교수님과 코드를 공유할 때, 상대방의 Ubuntu 설정이 조금만 달라도 에러가 나기 일쑤이다. Docker 이미지를 공유하면 전 세계 어디서든 똑같은 환경에서 실행되는 걸 보장할 수 있다.

**3. 다양한 버전 동시 사용:** 지금은 Jazzy를 쓰지만, 나중에 Humble이나 다른 하위 버전 기반의 오픈소스 로봇 코드를 돌려야 할 때가 온다. 로컬에 두 버전을 다 깔면 충돌 위험이 크지만, Docker라면 컨테이너만 바꿔서 띄우면 끝이다.

**4. 깔끔한 삭제와 복구:** 로컬에 ROS2를 깔았다가 지우면 찌꺼기 파일들이 남아 시스템을 느리게 만들거나 다른 패키지 설치를 방해한다. Docker는 컨테이너만 삭제하면 시스템에 아무런 흔적도 남지 않는다.

추가로 성능 차이는 로컬에 비해 95~99%로 거의 없다고 한다. 결론은 무조건 Docker를 사용하자.

다만 Docker를 쓸 때 주의할 점은 **GUI 및 하드웨어 연결**이다. Gazebo 같은 시뮬레이터를 띄우거나, 실제 로봇 센서(Lidar, Camera)를 연결할 때 로컬 환경과 통신 설정을 해줘야 한다. 처음엔 조금 까다로울 수 있지만, 한 번 설정하면 매우 편리하다.

# 1. 시스템 업데이트 및 필수 패키지 설치
먼저 시스템을 최신 상태로 만들고, Docker 설치에 필요한 도구들을 먼저 깔아주자.

```Bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ca-certificates curl gnupg lsb-release
```

# 2. Docker 공식 GPG 키 및 저장소 등록
Ubuntu 기본 저장소에도 Docker가 있지만, 공식 최신 버전을 사용할 것이다. 이게 더 버그가 적고 최적화되어 있다.

- **GPG키:** GNU Privacy Guard(GnuPG)에서 사용하는 암호화 키로, 데이터를 암호화하거나 디지털 서명을 통해 신원을 증명하는 데 사용한다.

```Bash
# GPG 키 등록
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg


# 저장소 추가
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
```

# 3. Docker 엔진 설치
```Bash
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

- 설치가 끝나면 `sudo systemctl status docker`를 입력해 Docker가 `active (running)` 상태인지 꼭 확인하자.

![docker active running](./assets/image/ch01/docker%20active%20running.png)

# 4. 권한 설정 (sudo 없이 사용하기)
매번 sudo docker라고 치는 건 번거롭고 효율적이지 않다. 현재 사용자를 Docker 그룹에 추가하자.

```Bash
sudo usermod -aG docker $USER
# 변경사항 적용을 위해 터미널을 껐다 켜거나 아래 명령 실행
newgrp docker
```

- docker 그룹에 사용자를 추가하는 것은 시스템 root 권한을 주는 것과 비슷하다. 보안이 아주 중요한 서버 환경이라면 조심해야 하지만, 개인 개발용 로컬 PC에서는 사용해도 괜찮다.


# 5. Dockerfile을 통한 로캘(Locale) 및 환경 설정

Docker 컨테이너는 정말 "최소한"의 것만 들어있는 깡통 상태이다. 그래서 sudo도 없고, 한글/영어 인코딩 설정도 안 되어 있어 문자가 깨지기도 한다. 이걸 매번 컨테이너 안에서 설정하는 건 너무 비효율적이다. Dockerfile에 필요한 모든 설정을 넣어보자.

먼저 `~/ros2_docker` 폴더를 만들고 그 안에 Dockerfile을 작성하자.

```Bash
mkdir -p ~/ros2_docker && cd ~/ros2_docker
nano Dockerfile
```

**[Dockerfile 내용]**
```bash
# 1. ROS2 Jazzy 공식 데스크탑 이미지를 베이스로 사용
FROM osrf/ros:jazzy-desktop

# 2. 기본 쉘을 bash로 설정
SHELL ["/bin/bash", "-c"]

# 3. 환경 변수 설정 (Non-interactive 모드)
ENV DEBIAN_FRONTEND=noninteractive

# 4. 필수 패키지 및 로캘(Locale) 설치
# sudo, vim, git, wget 등 개발에 꼭 필요한 도구들을 미리 깔아두자
RUN apt-get update && apt-get install -y \
    locales \
    sudo \
    vim \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 5. 로캘 생성 및 적용 (미국 영어 UTF-8)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# 6. (선택) 컨테이너 접속 시 자동으로 ROS2 환경 설정 로드
# 매번 source /opt/ros/jazzy/setup.bash 입력하기 귀찮으니까!
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# 7. 작업 디렉토리 설정
WORKDIR /root
```

작성이 끝났다면 아래 명령어로 이미지를 빌드하자.

```bash
# 이미지 이름은 'my_ros2_jazzy'로 지정한다
docker build -t my_ros2_jazzy:latest .
```

# 6. ROS2 Jazzy 컨테이너 실행하기
빌드한 이미지를 실행해보자. GUI 툴(Rviz2 등)을 쓸 수 있도록 디스플레이 설정을 포함한다.

```Bash
docker run -it --name my_ros2_jazzy_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  my_ros2_jazzy:latest
```

**[옵션 설명]**
- `-it`: 대화형 터미널을 사용하겠다는 뜻이다. 이게 없으면 컨테이너가 실행되자마자 종료될 수 있다.

- `--net=host`: 컨테이너가 호스트 PC의 네트워크를 직접 쓰게 한다. **ROS2의 노드 통신(DDS)** 을 위해 아주 중요하다.

- `-e DISPLAY=$DISPLAY` & `-v ...`: 컨테이너 내부의 GUI 창(Rviz2 등)을 내 우분투 화면에 띄우기 위한 핵심 설정이다.

# 7. 실행 확인 및 GUI 테스트
컨테이너 안으로 들어오면 별도의 `source` 명령 없이도 ROS2 명령어가 바로 먹힐 것이다. (`Dockerfile`에서 `.bashrc`에 등록했기 때문!)

```Bash
# 컨테이너 내부에서 바로 확인
ros2 run demo_nodes_cpp talker
```

새 터미널을 열고 실행 중인 컨테이너에 추가로 접속해보자.

```Bash
docker exec -it my_ros2_jazzy_container bash
ros2 run demo_nodes_py listener
```

![Hello World](./assets/image/ch01/Hello%20world%20demo.png)

위는 가장 기본적인 형태의 demo node들이 소통하는 예제이다. 자세한 내용은 다음에 다뤄보자.

그리고 만약 Rviz2 같은 창을 띄울 때 에러가 난다면, **호스트(Ubuntu 로컬)** 터미널에서 아래 명령어를 입력해 컨테이너의 디스플레이 접근 권한을 허용해주자.

```Bash
xhost +local:docker
```

# 8. 컨테이너 관리 및 유지보수
- **중지된 컨테이너 다시 시작:** `docker start my_ros2_jazzy_container` 후 `docker attach my_ros2_jazzy_container`를 입력한다.

- **컨테이너 삭제:** `docker rm -f my_ros2_jazzy_container`

- **상태 확인:** `docker ps -a` 명령어로 현재 어떤 컨테이너가 떠 있는지 수시로 확인하자.

- **일회용 컨테이너:** 잠깐 테스트만 할 거라면 `run` 명령어에 `--rm` 옵션을 붙이자. 종료 시 흔적도 없이 사라진다.


# + VS Code 'Dev Containers' 확장 도구 활용

Docker 컨테이너는 기본적으로 터미널 기반이라 코드 수정이 불편할 수 있다. 이때 **VS Code Dev Container** 를 쓰면 신세계가 펼쳐진다.

1. VS Code에서 `Dev Containers` 확장을 설치한다.

2. 컨테이너를 실행한다.

3. 좌측 하단 **초록색 아이콘(`><`)** 클릭 -> **"Attach to Running Container..."** 선택.

4. 이제 컨테이너 내부의 파일을 VS Code의 강력한 기능들(자동 완성, 디버깅 등)과 함께 편집할 수 있다.