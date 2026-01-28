# 0. ROS2 개발에 Docker가 필수일까?

**Docker**란 한마디로 소프트웨어를 담는 표준화된 화물 컨테이너이다.

과거에는 배에 물건을 실을 때 크기도 모양도 제각각이라 싣기가 힘들었다. 하지만 **container**라는 규격이 생기면서 어떤 배든, 어떤 항구든 똑같이 물건을 옮길 수 있게 되었다.

컴퓨터에서도 마찬가지이다. 내 컴퓨터(Ubuntu 24.04) 환경, 라이브러리 버전, 설정값들을 통째로 하나의 **image**로 만들고, 이를 **container**라는 격리된 공간에서 실행하는 기술이다.

- **image:** 실행에 필요한 모든 파일과 설정이 담긴 "설계도" 혹은 "설치 CD"
- **container:** 이미지를 실행시킨 "실제 상태". 운영체제 커널은 공유하면서 파일 시스템과 프로세스만 격리된 가벼운 공간

---

로컬에서 ROS2를 직접 설치하면 안좋다고 한다. 왜 그럴까?

1. **Dependency Hell:**
ROS2는 수백 개의 라이브러리에 의존한다. 만약 다른 프로젝트를 하려고 특정 라이브러리를 업데이트했는데, 그게 ROS2 Jazzy랑 충돌한다면 시스템 전체가 꼬여버릴 수 있다. Docker를 쓰면 각 프로젝트가 독립된 컨테이너에서 돌아가므로 시스템이 깨끗하게 유지된다.

2. **"내 컴퓨터에선 되는데?" 해결:**
연구실 동료나 교수님과 코드를 공유할 때, 상대방의 Ubuntu 설정이 조금만 달라도 에러가 나기 일쑤이다. Docker 이미지를 공유하면 **전 세계 어디서든 똑같은 환경**에서 실행되는 걸 보장할 수 있다.

3. **다양한 버전 동시 사용:**
지금은 Jazzy를 쓰지만, 나중에 Humble이나 다른 하위 버전 기반의 오픈소스 로봇 코드를 돌려야 할 때가 올것이다. 로컬에 두 버전을 다 깔면 충돌 위험이 크지만, Docker라면 컨테이너만 바꿔서 띄우면 끝!

4. **깔끔한 삭제와 복구**
로컬에 ROS2를 깔았다가 지우면 찌꺼기 파일들이 남아서 시스템을 느리게 만들거나 나중에 다른 패키지 설치를 방해한다. Docker는 컨테이너만 삭제하면 시스템에 아무런 흔적도 남지 않는다.

추가로 성능차이는 로컬에 비해 95~99%로 거의 없다고 한다(제미나이 피셜)

결론은 **무조건 Docker를 사용하자**

다만 Docker를 쓸 때 주의할 점은 **GUI 및 하드웨어 연결**이다. Gazebo 같은 시뮬레이터를 띄우거나, 실제 로봇 센서(Lidar, Camera)를 연결할 때 로컬 환경과 통신 설정을 해줘야 한다. 처음엔 조금 까다로울 수 있다.

---
자, 그럼 Docker에서 ROS2 Jazzy를 올리는 방법을 알아보자. 참고로 나중에 로봇 시뮬레이션을 GUI로 실행시키는 방식까지 고려하였다.

# 1. 시스템 업데이트 및 필수 패키지 설치
먼저 시스템을 최신 상태로 만들고, Docker 설치에 필요한 도구들을 먼저 깔아주자.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ca-certificates curl gnupg lsb-release
```

# 2. Docker 공식 GPG 키 및 저장소 등록
Ubuntu 기본 저장소에도 Docker가 있지만, 우리는 공식 최신 버전을 사용할 것이다. 이게 더 버그가 적고 최적화되어 있다.

- GPG키: GNU Privacy Guard(GnuPG)에서 사용하는 암호화 키로, 데이터를 암호화하거나 디지털 서명을 통해 신원을 증명하는 데 사용

```bash
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
```bash
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
- 설치가 끝나면 `sudo systemctl status docker`를 입력해 Docker가 `active (running)` 상태인지 꼭 확인해보자.

![active running](./assets/image/sudo%20systemctl%20status%20docker.png)

# 4. 권한 설정 (sudo 없이 사용하기)
매번 `sudo docker`라고 치는 건 너무 귀찮고 효율적이지 않다. 현재 사용자를 Docker 그룹에 추가하자.

```bash
sudo usermod -aG docker $USER
# 변경사항 적용을 위해 터미널을 껐다 켜거나 아래 명령 실행
newgrp docker
```

- `docker` 그룹에 사용자를 추가하는 것은 시스템 root 권한을 주는 것과 비슷하다. 보안이 아주 중요한 서버 환경이라면 조심해야 하지만, 개인 개발용 로컬 PC에서는 사용해도 ㄱㅊ

# 5. ROS Jazzy 컨테이너 실행하기
ROS2 공식 이미지를 불러와서 실행해보자. 우리는 GUI 툴(Rviz2 등)을 쓸 수도 있으니 `desktop` 버전을 선택하자.

```bash
# --name 옵션으로 이름을 지정하면, 그 이름은 고유해야 한다.
docker run -it --name ros2_jazzy_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:jazzy-desktop
```

[옵션 설명]
- `-it`: 대화형 터미널을 사용하겠다는 뜻
- `--net-host`: 컨테이너가 호스트 PC의 네트워크를 직접 쓰게 한다. **ROS2의 노드 통신(DDS)**을 위해 아주 중요하다.
- `-e DISPLAY-$DISPLAY` & `-v ...`: 컨테이너 안에서 뜬 창을 내 화면(GUI)에 보여주기 위한 설정이다.

# 6. 실행하기

컨테이너 안으로 들어왔다면, ROS2가 잘 작동하는지 확인해보자.

```bash
# 컨테이너 내부 터미널에서 입력
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

새 터미널을 추가로 열고 `docker exec -it ros2_jazzy_container bash`로 접속해서 아래를 입력해보자.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

추가로 Rviz2 같은 창(GUI)을 띄우려는데 에러가 난다면, **HOST (Ubuntu 로컬)** 터미널에서 다음 명령어를 입력해주자. 외부 기기(컨테이너)가 내 화면을 쓸 수 있게 허용해주는 것이다.

# 7. 기존 컨테이너 다시 시작하기
이미 만들어진 컨테이너가 있다면, 새로 만들지 않고 기존 것을 다시 깨워오자. 컨테이너 내부에서 작업했던 내용이 남아있을 때 유용하다.

```bash
# 1. 중지된 컨테이너 다시 시작
docker start ros2_jazzy_container

# 2. 실행 중인 컨테이너 안으로 들어가기 (attach)
docker attach ros2_jazzy_container
```

# 8. 기존 컨테이너 삭제 후 새로 만들기
```bash
# 1. 기존 컨테이너 강제 삭제
docker rm -f ros2_jazzy_container

# 2. 다시 실행
docker run -it --name ros2_jazzy_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:jazzy-desktop
```

# 8. 컨테이너 상태 확인하기
지금 내 컴퓨터에 어떤 컨테이너들이 있는지 궁금하다면 다음 명령어를 사용하자.

```bash
docker ps -a
```
`STATUS`열을 보면 `Exited`인지 `'Up`(실행중)인지 바로 알 수 있다.

# 9. `-rm` 옵션 활용하기
실습용으로 잠깐 쓰고 버릴 컨테이너라면, 명령어에 `--rm`을 추가해 보자. **컨테이너를 종료하는 순간 자동으로 삭제**. 이름 충돌 에러를 원천 봉쇄할 수 있다.

```bash
# 실행 종료 시 자동 삭제되는 버전
docker run -it --rm --name ros2_jazzy_test \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:jazzy-desktop
```

# 10. ROS2 Jazzy 및 관련 설정 삭제
1. ROS2 Jazzy 패키지 삭제
먼저 설치된 모든 ROS2 관련 패키지를 삭제하자.

```bash
# ROS2 Jazzy 관련 모든 패키지 삭제
sudo apt remove ~nros-jazzy-*
# 더 이상 필요 없는 의존성 패키지들 삭제
sudo apt autoremove -y
# 설정 파일까지 완전히 삭제 (purge)
sudo apt purge ~nros-jazzy-*
sudo apt autoremove -y
```

2. Repository 및 key 제거
APT 리스트에 등록된 ROS2 저장소와 보안 key도 지워주자. 시스템 업데이트 시 불필요한 체크를 줄여준다.

```bash
# ROS2 저장소 리스트 삭제
sudo rm /etc/apt/sources.list.d/ros2.list
# 등록된 GPG 키 삭제 (설치 시 경로를 따름)
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
# 패키지 목록 업데이트
sudo apt update
```

3. 워크스페이스 및 설정 폴더 삭제

워크스페이스는 보통 직접 만든 폴더이기 때문에 수동으로 지워줘야 한다.

```bash
# 워크스페이스 삭제
rm -rf ~/ros2_ws

# ROS 관련 임시/로그 폴더 삭제
rm -rf ~/.ros
```

4. `.bashrc` 환경 변수 정리
터미널을 켤 때마다 `source /opt/ros/jazzy/setup.bash`가 실행되도록 설정해뒀다. 이걸 안 지우면 터미널을 열 때마다 "파일을 찾을 수 없다"는 에러 메시지가 뜰 수 있다.

터미널에서 `vim ~/.bashrc` 입력하고

파일의 맨 아래쪽으로 내려가서 아래와 가틍ㄴ 줄을 찾아 삭제하거나 주석처리하자.

```bash
source /opt/ros/jazzy/setup.bash
# 또는
source ~/ros2_ws/install/setup.bash
```

`source ~/.bashrc`를 입력해 변경 사항을 바로 적용하자.

# + VS Code 'Dev Containers' 확장 도구
Docker 컨테이너는 기본적으로 호스트와 격리된 환경이다. 그래서 ubuntu와 달리 `code`라는 명령어가 설치되어 있지 않거나, 있더라도 호스트의 GUI와 연결되지 않는다.

그래서 **VS Code 'Dev Container'** 라는 확장도구를 사용할 것이다.

VS Code에서 컨테이너 내부의 파일을 마치 로컬 파일처럼 편집할 수 있게 해주는 강력한 확장 기능이다.

1. VS Code를 열고 왼쪽 사이드바의 Extensions(`Ctrl` + `Shift` + `X`)에서 **Dev Container**를 검색해서 설치하자. (보통 Remote Development 팩에 포함되어 있다)

2. **실행 중인 컨테이너에 접속:** 

Docker conatiner를 먼저 실행해준다. 

![running container](./assets/image/ch03/docker%20running.png)

VS Code 왼쪽 하단의 초록색 아이콘(`><`)을 클릭하여 **"Attach to Running Container..."** 를 선택하자.

![docker 1](./assets/image/ch03/docker%20vscode%201.png)

![docker 2](./assets/image/ch03/docker%20vscode%202.png)

목록에서 현재 실행 중인 ROS 컨테이너를 선택하면 끝이다.

![docker 2](./assets/image/ch03/docker%20vscode%203.png)