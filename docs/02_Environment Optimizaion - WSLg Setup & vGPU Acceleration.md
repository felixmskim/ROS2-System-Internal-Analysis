# 1. WSLg 설치 및 방법 확인
WSLg는 ***WSL GUI***의 약자로, WSL2의 커널 및 가상화 스택을 활용해 리눅스 GUI 앱을 윈도우 창으로 띄워주는 기술이다.

다음 기술에 따라 WSLg를 설치하자.

1. Powershell이나 CMD를 '관리자 권한'으로 실행하자.
2. 다음 명령어 입력
```PowerShell
# wsl 최신 버전으로 업데이트
wsl --update
```
3. 업데이트가 완료되면 WSL 완전히 종료했다가 켜야한다.
```PowerShell
wsl --shutdown
```
4. WSLg 동작을 확인해주자. WSL(Ubuntu) 터미널을 열어서 확인하자.
```bash
sudo apt update
sudo apt install x11-apps -y
xeyes
```

WSLg를 다운로드 받기위해 Microsoft 공식 홈페이지에 가니 다음과 같은 문구가 있었다.

"Linux GUI 앱을 실행하려면 먼저 아래 시스템과 일치하는 드라이버를 설치해야 합니다. 이렇게 하면 하드웨어 가속 OpenGL 렌더링을 활용할 수 있도록 vGPU(가상 GPU)를 사용할 수 있습니다."

# 2. vGPU(가상 GPU) 드라이버 설치

**vGPU 드라이버란?**
기본적으로 WSL2는 가상화된 환경이라서, 리눅스 안에서 돌아가는 앱(예: Gazebo)은 윈도우에 꽂힌 그래픽 카드가 무엇인지 직접 알 수 없다. 그래서 드라이버가 없으면 모든 그래픽 계산을 CPU가 대신하게 된다 (Software Rendering).

**vGPU(GPU Paravirtualization)** 는 윈도우 호스트의 GPU 드라이버가 가상화 계층을 통해 리눅스 커널에 GPU 자원을 노출해주는 기술이다.

D3D12 매핑은 리눅스의 그래픽 표준인 OpenGL이나 Vulkan 명령어를 윈도우의 DirectX 12 명령어로 변환해서 하드웨어 가속을 실현한다.

이는 CPU 성능이 여유롭지 않은 노트북에 사용하기에 매우 안성맞춤이라고 한다. 시뮬레이션의 그래픽 계산을 GPU로 넘겨주면, CPU는 ROS 노드의 로직 처리나 시스템 분석에 전념할 수 있어서 전체적인 시스템 반응 속도가 훨씬 빨라지고,  배터리 및 발열에도 매우 좋다고 한다.

[!CAUTION] 주의할 점: 리눅스(Ubuntu) 안에서 `sudo apt install nvidia-driver-...` 같은 명령어로 리눅스용 드라이버를 직접 설치하면 절대로 안된다. 윈도우 호스트에 최신 드라이버를 깔면 WSL2가 알아서 인식하는 방식이다.

## 설치 방법
1. 드라이버를 업데이트해준다. 나는 Intel 그래픽 드라이버를 사용하므로 [여기](https://www.intel.co.kr/content/www/kr/ko/download-center/home.html)에서 설치해줬다. 이미 최신 버전이라면 넘어가도 좋다.

2. Ubuntu에서 아래 명령어를 쳤을 때 제조사 이름이 나오는지 확인해보자.

```bash
sudo apt update && sudo apt install mesa-utils -y
glxinfo -B | grep "OpenGL renderer"
```

- 출력 결과에 `D3D12 (Intel/NVIDIA/AMD...)`라고 뜨면 성공

- 만약 `llvmpipe`라고 뜨면 아직 CPU가 고생 중인 상태이다. 즉, 현재 리눅스가 그래픽 카드 하드웨어를 인식하지 못하고 CPU의 계산 능력으로만 그래픽을 억지로 그려내게 된다.

# 3. 그래픽 카드 하드웨어 인식하게 하는 방법..
1. [Window 호스트] 그래픽 드라이버의 버전 확인
WSLg(D3D12 매핑)를 지원하는 드라이버가 깔려 있는지 확인하자.

`Win + X` 키 -> `장치 관리자` -> `디스플레이 어댑터`에서 내 그래픽 카드를 우클릭해서 **드라이버 날짜**를 확인하자. 2023년 이후 버전이여야 함.

2022년 이후 버전이면 대부분 하드웨어 가속을 지원한다.

또 다른 방법으로는 `Win + R` 키 -> `dxdiag` 입력 -> [디스플레이] -> [드라이버] 항목에서 [드라이버 모델]을 확인한다.

- WDDM 3.0 이상: 하드웨어 가속 가능 (업데이트만 진행)
- WDDM 2.x 이하: 하드웨어 가속이 불가능하거나 제한적 (최신 드라이버로 교체 필수)

최신 드라이버로 교체할 때, 그래픽 칩셋 제조사에서 직접 설치해야 한다. 드라이버를 아예 싹 밀고 설치하는 **clean install** 옵션을 체크해주자. 기존의 꼬인 드라이버 설정을 밀어버리는게 가장 깔끔하다. 그리고 윈도우를 **재시작**하고, PowerShell에서 `wsl --shutdown`을 한번 더 해준 뒤 Ubuntu를 다시 켜야 한다.


만약 윈도우 기본 드라이버(Microsoft Basic Display Adapter)가 잡혀 있다면, 반드시 제조사(Intel/NVDIA/AMD) 홈페이지에서 직접 받은 드라이버로 재설치해야 함.

2. [WSL 커널] `wsl --update` 강제 실행
WSL 커널 버전이 낮으면 윈도우 드라이버와 리눅스 사이의 브리지(vGPU)가 작동하지 않는다.
- 명령어(PowerShell 관리자 권한):
```PowerShell
wsl --update
wsl --shutdown
```

3. [Ubuntu 게스트] Mesa 라이브러리 업데이트
리눅스에서 윈도우의 GPU를 호출하려면 `Mesa`라는 그래픽 라이브러리가 최신이어야 한다. 특히 `d3d12` 드라이버가 포함되어 있는지 확인하다.

```bash
# Ubuntu 터미널에서 실행
sudo add-apt-repository ppa:kisak/kisak-mesa -y
sudo apt update && sudo apt upgrade -y
```

4. [환경 변수] `d3d12` 드라이버 강제 지정
시스템이 어떤 드라이버를 쓸지 헤매고 있을 때, 강제로 D3D12(윈도우 GPU 하드웨어 가속)를 사용하게 명령한다.

이 방법을 사용하기전에 다음 두가지 방법으로 상태를 확인하자.
- **1. `/dev/dxg` 존재 확인:** Ubuntu 터미널에서 `ls -l /dev/dxg`라고 입력했을 때, 장치 파일이 보여야 한다. 이게 보여야 윈도우 GPU가 리눅스에 노출된 것이다.

- **2. 그래픽 카드 상태 확인:**
```PowerShell
Get-PnpDevice -FriendlyName "*Intel(R) Arc(TM)*" | Select-Object FriendlyName, Status, DEVICESTATUS
```
- **Status**가 `OK`여야 한다.
- **Error Code 43**이 남아있다면 Status가 `Error`나 `Warning`으로 뜰거다. `OK`라고 뜬다면 하드웨어 초기화는 성공한것이다.

- **3. 리눅스 커널 로그(dmesg) 확인:**
WSL2 리눅스 터미널에서 커널이 장치를 인식할 때 에러가 없었는지 확인하자.
```bash
dmesg | grep -i dxg
```
- 만약 에러가 있다면 `[drm:dxg_...] *ERROR*` 같은 문구가 보일 것이다.
- 아무런 에러 메시지가 없거나, `Initialized dxgk` 같은 긍정적인 메시지가 보인다면 OK

자, 이제 다음 명령어를 입력해주자.

```bash
# 드라이버 강제 지정 (현재 터미널 세션에만 적용)
export GALLIUM_DRIVER=d3d12
# 그래픽 가속 확인
glxinfo -B | grep "OpenGL renderer"
```

만약에 여기서 `D3D12`라고 뜬다면 성공! 이 설정을 영구적으로 저장하려면 아래를 수행
```bash
# WSL2 GPU Acceleration
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
source ~/.bashrc
```

# 4. 외장 GPU(Arc A350M) 우선순위 설정
노트북은 전력 절약을 위해 내장 그래픽을 우선 쓰려고 할 때가 있어. 성능 분석을 위해 성능이 더 좋은 Arc A350M을 WSL2가 쓰도록 강제하자.

`윈도우 설정` -> `시스템` -> `디스플레이` -> `그래픽`으로 이동.

'vmmem' 또는 'WSL' 관련 앱이 목록에 있다면 클릭해서 [고성능] (Intel Arc A350M)으로 지정해.

목록에 없다면 `C:\Windows\System32\wsl.exe`를 추가해서 고성능으로 설정해줘.


# 5. ROS2 로봇 시뮬레이션 종류

다음 2가지 ROS2 로봇 시뮬레이션을 돌려보려고 한다.

**1. Turtlesim:** 가장 가벼운 2D 시뮬레이터이다. Node 통신 분석의 정석이며, IPC(Inter-Process Commi=unication) 구조를 분석하기에 가장 노이즈가 적다.

**2. Webots:** 효율적인 3D 시뮬레이터이다. 메모리 효율성이 좋다. 상대적으로 적은 자원으로 복잡한 로봇 시뮬레이션이 가능해 자원 제약 환경을 분석하기 유리하다.

## 5-1. Turtlesim 설치 및 구동

다음 명령어를 사용하여 Turtlesim을 다운로드하자.
```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim -y
```

그리고 귀여운 프로젝트를 하자 진행해보자. 거북이를 조종하는 예제이다. 터미널 2개 띄워야 함.

**1) 시뮬레이터 실행(터미널1)**
```bash
# 실행 확인 (WSLg 창이 떠야 함)
ros2 run turtlesim turtlesim_node
```

**2) 거북이 조종 노드 실행(터미널2)**
```bash
ros2 run turtlesim turtle_teleop_key
```

![터틀심 작동 예시](./assets/clips/ros2_tutlesim_example.gif)

다음은 코드로 터틀봇을 구동시키는 예제이다.
터틀심은 단순히 키보드 입력만 받는게 아니다. `/turtle1/cmd_vel`이라는 이름의 **Topic**을 Subscribe한다. 우리가 코드로 이 토픽에 "어느 방향으로, 얼마나 빨리 가라"는 메시지만 보내기만 하면 된다.

### 5-1-1. 코딩으로 움직이는 원리 (통신구조)
터틀심 로봇을 움직이려면 `geometry_msg/msg/Twist`라는 타입의 메시지를 Publish하는 노드를 작성해야 한다. 이 메시지는 크게 두 가지 데이터를 담고 있다.
- **Linear(선속도):** 앞뒤로 움직이는 속도 (***x***, ***y***, ***z*** 축)
- **Angular(각속도):** 제자리에서 회전하는 속도(***z***축 중심 회전)

### 5-1-2. 패키지 생성 및 코드 실행
터미널을 열고 `src` 폴더로 이동하여 Python 기반의 ROS2 패키지를 만든다.
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 turtlesim_drawer
```

이제 터틀심이 사각형을 그리도록 제어하는 코드를 작성해보자. `~/ros2_ws/src/turtlesim_drawer/turtlesim_drawer/drawer_node.py` 파일을 만들고 아래 코드를 작성하자.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('터틀심 모형 그리기를 시작합니다!')
        self.draw_square()

    def move_straight(self, speed, duration):
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop_turtle()

    def turn(self, angular_speed, duration):
        msg = Twist()
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop_turtle()

    def stop_turtle(self):
        msg = Twist()
        self.publisher_.publish(msg)
        time.sleep(1) # 동작 간의 간격

    def draw_square(self):
        for i in range(4):
            self.get_logger().info(f'{i+1}번째 변 그리는 중...')
            self.move_straight(2.0, 2.0)  # 2.0 속도로 2초간 전진
            self.turn(1.5708, 1.0)        # 약 90도(PI/2) 회전
        self.get_logger().info('사각형 완성!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDrawer()
    # 사각형 그리기가 끝나면 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5-1-3. 설정 파일 수정(`setup.py`)
ROS2가 코드를 실행 파일로 인식할 수 있게 등록해야 함.
`~/ros2_ws/src/turtlesim_drawer/setup.py` 파일을 열어 `entry_points` 부분을 수정한다.

```python
entry_points={
        'console_scripts': [
            'draw_node = turtlesim_drawer.drawer_node:main'
        ],
    },
```

### 5-1-4. 빌드 및 소스 로드
워크스페이스 루트로 돌아와 빌드하고 환경을 적용한다.
```bash
cd ~/ros2_ws
colcon build --packages-select turtlesim_drawer
source install/setup.bash
```

![turtlesim_example2](./assets/clips/turtlesim_example2.gif)


## 5-2. Webots 설치 및 ROS2 패키지 설정

Webots 시뮬레이터 본체와 ROS2를 연결해 주는 인터페이스 패키지를 설치해야 한다.

```bash
# 1. 공식 Webots .deb 패키지 다운로드 및 설치
wget [https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb](https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb)
sudo apt install ./webots_2025a_amd64.deb

# 2. ROS2 Jazzy용 Webots 인터페이스 설치
sudo apt update
sudo apt install ros-jazzy-webots-ros2 ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-diff-drive-controller
```

# 6. WSL2(root 계정) 환경에서의 트러블슈팅

WSL2에서 root 계정으로 시뮬레이션을 돌리려니 예기치 못한 경로 및 환경 변수 문제가 많이 발생했다. 이를 해결한 과정을 정리한다.

## 6-1. 환경 변수 및 wslpath 문제 해결
webots_ros2 드라이버는 내부적으로 리눅스-윈도우 경로 변환을 위해 wslpath를 사용한다. 하지만 root 계정은 윈도우 시스템 경로가 PATH에 잡히지 않아 에러가 발생한다.

- 해결 방법: 가짜 `wslpath` 스크립트 생성 리눅스용 Webots를 쓸 때는 경로 변환이 필요 없으므로, 경로를 그대로 반환하는 스크립트를 만들어 준다.

```bash
# WEBOTS_HOME 설정
export WEBOTS_HOME=/usr/local/webots
echo 'export WEBOTS_HOME=/usr/local/webots' >> ~/.bashrc

# wslpath 에러 방지용 래퍼 스크립트 생성
cat << 'EOF' > /usr/bin/wslpath
#!/bin/bash
echo "$2"
EOF
chmod +x /usr/bin/wslpath
```

## 6-2. 실행 파일 경로 인식 오류 (webots.exe 찾기 error)
드라이버가 리눅스 환경임에도 불구하고 `webots.exe`라는 윈도우용 파일을 찾는 경우가 있다.

- 해결 방법: 심볼릭 링크 트릭

```bash
mkdir -p /usr/local/webots/msys64/mingw64/bin/
ln -sf /usr/local/webots/webots /usr/local/webots/msys64/mingw64/bin/webots.exe
```

## 6-3. 그래픽 가속 및 소리 에러

WSLg 환경에서 그래픽 창이 뜨자마자 꺼지거나, 소리 장치(OpenAL)를 찾지 못해 시스템이 멈추는 현상이 발생한다.

- 해결 방법: 소프트웨어 렌더링 및 환경 변수 강제 지정

```bash
export LIBGL_ALWAYS_SOFTWARE=1  # 그래픽 드라이버 충돌 시 사용
export DISPLAY=:0               # 디스플레이 포트 지정
export USER=root                # 사용자 이름 명시
```

# 7. Webots 터틀봇3 시뮬레이션 구동
모든 설정이 완료되었다면 아래 명령어로 터틀봇3 버거 모델을 띄울 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

## 7-1. 키보드로 로봇 제어하기 (Teleop)
Webots 창이 뜨고 시뮬레이션 시간이 흐르고 있다면 (하단 Time 확인), 새 터미널에서 제어 노드를 실행한다.

```bash
# 토픽 이름 리매핑과 함께 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffdrive_controller/cmd_vel
```

- **왜 리매핑(`/cmd_vel:=/diffdrive_controller/cmd_vel`)이 필요한가?**
Webots 내부의 `ros2_control` 노드가 실제 바퀴를 굴리기 위해 Subscribe하는 토픽 이름이 `/diffdrive_controller/cmd_vel`이기 때문이다.

# 8. 시스템 구조 분석
- **App Layer:** `teleop_twist_keyboard`가 사용자의 키 입력을 `Twist` 메시지로 변환
- **Middleware Layer:** ROS2 DDS를 통해 `/diffdrive_controller/cmd_vel` 토픽 전송
- **Hardware Abstraction:** `ros2_control`의 `DiffDriveController`가 메시지를 받아 각 바퀴의 조인트 속도로 변환
- **Simulation Layer:** `webots_ros2_driver`가 이 신호를 Webots API로 전달하여 가상 로봇 구동

현재 `BallJoint(Caster Joint)` 관련 URDF 내보내기 경고가 뜨고 있는데, 이는 터틀봇 모델의 보조 바퀴 모델링 방식과 ROS2 URDF 파서 간의 호환성 문제로 보인다. 실제 주행에는 지장이 없으나, 정밀한 물리 분석 시에는 조인트 타입 수정을 고려해야 한다.

향후 문제를 해결하여 수정하겠다.

## 🔗 참고 자료
- [Official] Docker Documentation: Install on Ubuntu

- [blog]] 로봇운영체제 실습 - Simulation: Webots