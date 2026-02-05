이 문서는 ROS2의 통신 인프라인 **DDS(Data Distrubuted Service)** 의 내부 구조를 분석하고, 다양한 네트워크 환경에서 통신 품질을 보장하기 위한 **QoS(Quality of Service)** 설정 전략을 수립한다. 미들웨어가 어떻게 데이터를 추상화하고 전송하는지에 초점을 맞춘다.

# 1. Understanding DDS: The Data-Centric Middleware

ROS1이 중앙 집중식 마스터(`roscore`)에 의존했다면, ROS2는 분산 환경에 최적화된 **DDS**를 채택하여 **Sercerless** 통신 아키텍처를 구축한다.

## 1.1. Data-Centric 중심 철학

DDS는 **Data-Centric** 모델을 따른다.이는 미들웨어가 단순히 메시지를 전달하는 통로 역할만 하는 것이 아니라, 데이터 자체의 내용과 속성을 이해하고 관리한다는 것을 의미한다.

- **분산 공유 데이터 공간 (Global Data Space):** 모든 노드는 거대한 가상 메모리 공간에 데이터를 쓰고 읽는 것처럼 통신하다.
- **상태 관리:** 노드가 언제 참여하든 상관없이 시스템의 최신 상태를 동기화할 수 있는 매커니즘을 제공한다.

## 1.2. DDS Entity Hierachy (DCPS 모델)

DDS의 통신은 **DCPS(Data-Centric Publish-Subscribe)** 께층에서 이루어지며, 다음과 같은 엔티티 구조를 가진다.

| Entity | 역할 및 특징 |
| :--- | :--- |
| **Domain Participant** | DDS 네트워크 참여의 기본 단위. 하나의 도메인 내에서 다른 참여자들을 식별함 |
| **Topic** | 데이터의 종류를 식별하는 이름. 데이터 형식(IDL)과 연결됨 |
| **Publisher/Subscriber** | 데이터를 배포하거나 수신하기 위한 관리자 객체 |
| **DataWriter/DataReader** | 실제 데이터를 쓰거나 읽는 엔드포인트. 여기서 QoS가 적용됨. |

# 2. RMW(ROS Middleware) Layer

### RMW 추상화 계층
- **추상화의 목적:** 다양한 DDS 벤더(e.g. eProsima FastDDS, Eclipse CycloneDDS)의 API 차이를 숨기고 ROS 2 상위 계층에 동일한 인터페이스를 제공한다.
- **현재 설정:** 이전 Chapter에서 구축한 Docker 환경에서는 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 사용하여 **FastDDS**를 기본 미들웨어로 고정하였다. 이는 성능과 안정성 측면에서 JAZZY 버전과 가장 잘 부합하기 때문이다.

# 3. QoS(Quality of Service) Polices
QoS는 "데이터를 어떻게 보낼 것인가?"에 대한 규칙이다. 네트워크의 상태나 애플리케이션의 중요도에 따라 최적의 통신 정책을 수립해야 한다.

## 3.1. 주요 QoS 파라미터 상세 분석

**1) History (이력관리)**
- **Keep Last:** 정해진 개수(Depth) 만큼의 최신 데이터만 보관한다. (가장 흔히 사용)
- **Keep All:** 모든 데이터를 보관한다. 메모리 자원 소모가 크지만 데이터 유실을 방지해야 할 때 사용한다.

**2) Reliability(신뢰성)**
- **Reliable:** TCP와 유사하게 데이터 수신을 확인(ACK/NACK)하고 유실 시 재전송한다. 제어 명령 등에 필수적이다.
- **Best Effort:** UDP와 유사하게 확인 절차 없이 전송한다. 센서 데이터처럼 주기적으로 갱신되어 유실보다 지연 시간이 더 중요한 경우에 적합하다.

**3) Durability(지속성)**
- **Volatile:** 노드가 접속한 시점 이후의 데이터만 수신한다.
- **Transient Local:** 노드가 늦게 접속하더라도, 기존에 발행되었던 데이터를 Publisher가 보관하고 있다가 전달해준다. (노드의 재시작 시 상태 복구에 유리)

**4) Liveliness(생존 확인)**
- 노드가 여전히 살아있는지 확인하는 주기를 설정한다. 시스템 소프트웨어의 Fault Detection과 직결되는 부분이다.

# 4. QoS Compatibility: RX/TX Matching
QoS 설정 시 가장 주의해야 할 점은 **Publisher와 Subscriber 간의 호환성**이다.호환되지 않는 설정을 가질 경우, 노드 간 연결은 되지만 데이터가 전달되지 않는 문제가 발생한다.
- **Compatibility Principle:** 제공하는 쪽(Tx)이 요구하는 쪽(Rx)보다 같거나 더 높은 수준의 서비스를 제공해야 한다.
  - e.g. `TX: Reliable / Rx: Best Effort` => **연결 성공** (더 좋은 서비스를 준다는데 거절할 이유가 없음)
  - e.g. `TX: Best Effort / RX: Reliable` => **연결 실패** (요구 수준은 높은데 낮은 서비스를 제공하므로)
  
# 5. Discovery Mechanism

DDS는 중앙 마스터 없이 노드를 어떻게 찾을까? 이 과정을 이해하는 것이 시스템 최적화의 핵심이다.

1. **PDP (Participant Discovery Protocol):** 멀티캐스트를 통해 새로운 참여자의 존재를 알린다.
2. **EDP(Endjpoint Discovery Protocol):** 참여자 간 어떤 Topic과 QoS를 사용하는지 유니캐스트로 정보를 교환한다.
3. **Matching:** 최종적으로 QoS 호환성이 확인되면 데이터 통신 경로 (Locator)를 생성한다.

# 6. Summary for Practice
ROS 2 통신 성능은 단순히 코드가 아니라 **DDS 설정**에 달려 있음을 확인하였다.
- 제어 명령이나 상태 보고는 **Reliable + Transient Local**을 권장한다.
- 영상이나 LiDAR 데이터는 **Best Effort + Volatile**을 통해 Latency를 최소화한다.

다음 Chapter에서는 이 DDS 지식을 바탕으로 직접 QoS 프로파일을 생성하고, `Turtlesim` 노드 간의 통신 신뢰성을 강제로 조정하며 네트워크 동작 변화를 관찰하겠다.

## 🔗 참고 자료
- [Wikidocs] [[ROS 2 Jazzy 문서] 2.02. ROS 2의 다양한 미들웨어 벤더](https://wikidocs.net/254953)
- [Wikidocs] [[ROS 2 Jazzy 문서] 2.01. ROS_DOMAIN_ID](https://wikidocs.net/254951)
- [Blog] [[ROS2] ROS 2의 DDS와 QoS](https://velog.io/@katinon/ROS2-ROS-2%EC%9D%98-DDS%EC%99%80-QoS)