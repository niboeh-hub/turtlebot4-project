# 🏥 병원용 복합 AMR 자율주행 순찰 로봇 및 원격 통합 관제 시스템 플랫폼

> **ROS 2 Humble 기반 다중 로봇 제어, AprilTag 정밀 로컬라이제이션, YOLOv8 비전 인지 및 Flask 관제 풀스택 통합 인프라**

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)
![Flask](https://img.shields.io/badge/Framework-Flask-000000?style=flat-square&logo=flask)
![MySQL](https://img.shields.io/badge/Database-MySQL-4479A1?style=flat-square&logo=mysql)
![YOLOv8](https://img.shields.io/badge/Vision-YOLOv8-FF1493?style=flat-square)
![Ubuntu](https://img.shields.io/badge/OS-Ubuntu_22.04-E95420?style=flat-square&logo=ubuntu)

본 플랫폼은 스마트 병동 환경에서 자율주행 순찰을 수행하는 로봇 에이전트 레이어(ROS 2)와, 이를 실시간으로 원격 모니터링하고 순찰 동선을 관리하는 중앙 관제 서버 레이어(Flask + MySQL)를 유기적으로 동기화한 **로봇-웹 풀스택 통합 플랫폼**입니다. 하드웨어의 저수준 제어부터 웹 상위 인터페이스 설계 및 데이터 트랜잭션 예외 처리까지 견고하게 설계되었습니다.

---

## 📋 목차
1. [전체 시스템 아키텍처](#1-전체-시스템-아키텍처)
2. [통합 프로젝트 디렉토리 구조](#2-통합-프로젝트-디렉토리-구조)
3. [핵심 컴포넌트별 기능 분석](#3-핵심-컴포넌트별-기능-분석)
4. [데이터베이스 스키마 및 ROS 2 인터페이스 명세](#4-데이터베이스-스키마-및-ros-2-인터페이스-명세)
5. [🚀 설치 및 구동 방법](#5--설치-및-구동-방법)
6. [❓ 문제 해결 (Troubleshooting)](#6-문제-해결-troubleshooting)

---

## 1. 전체 시스템 아키텍처

```text
[ 상위 웹 관제 및 인프라 레이어 ]
  - Flask Web Server (Port 5002) & PM2 프로세스 매니저 기반 무중단 배포
  - MySQL DB (`hospital_robot_db`): 환자 상태 관리 및 로봇별 순찰 동선(Waypoint) 저장
  - YOLOv8 + ByteTrack: 병실 입구 유동 인구 분석 및 실시간 실내 체류 혼잡도 연산
        │
        ├── [ROS 2 통신 브릿지 및 원격 제어 인터페이스]
        │   - Publishes : /robot1/cmd_vel, /patrol_cmd, /send_msg
        │   - Subscribes: /robot1/amcl_pose, /robot1/battery_state, /robot1/emergency_alert
        │   - Actions   : /robot6/navigate_to_pose (협업 로봇 긴급 출동 제어)
        │
[ 하위 로봇 에이전트 및 제어 레이어 ]
  - patrol_node_april: 자율 언도킹 및 초기화 주행 스케줄링 총괄 마스터 노드
  - enji_auto_localization: AprilTag 기반 동시 P-Control 정렬 주행 및 AMCL 영점 재조정
  - amr_detect: 360도 회전 스캔 및 YOLOv8-pose 기반 Score Voting 구조의 누움(Lying) 자세 분류
```

---

## 2. 통합 프로젝트 디렉토리 구조

```text
📦 hospital_robot_project
 ┣ 📂 hospital_robot_server/          # 상위 중앙 관제 서버 패키지
 ┃ ┣ 📜 app.py                        # Flask 메인 백엔드 & ROS 2 노드, YOLO 스레드 통합 제어 루프
 ┃ ┣ 📜 mysql_database.py             # PyMySQL 기반 DB 커넥션 래퍼 (DictCursor 구현)
 ┃ ┣ 📜 mysql_crud.py                 # DB 트랜잭션 및 공통 CRUD 쿼리 처리 서브 클래스
 ┃ ┣ 📜 yolov8s.pt                    # 실시간 혼잡도 분석용 객체 검출 모델 가중치
 ┃ ┣ 📜 virtual_line.json             # In/Out 방향 판정용 가상 펜스 2D 좌표 데이터
 ┃ ┣ 📜 requirements.txt              # 파이썬 가상환경 의존성 명세서
 ┃ ┗ 📜 ecosystem.config.cjs          # PM2 백그라운드 서비스 배포 가동 파일
 ┃
 ┗ 📂 rokey_patrol/                   # 하위 로봇 에이전트 구동 패키지
   ┣ 📜 patrol_node_april.py          # 자동 언도킹 및 웹 명령 동적 수신/실행 최상위 스케줄러
   ┣ 📜 enji_auto_localization.py     # AprilTag 정면 정렬(동시 P-Control) 및 /initialpose 발행 노드
   ┣ 📜 amr_detect.py                 # 360도 탐색 회전, Nudge 제어 및 위급 상황 트리거 노드
   ┣ 📜 pose_classifier.py            # 17개 관절 키포인트 기반 주성분분석(PCA) 및 자세 판별 모듈
   ┣ 📜 audio_manager.py              # 상황별 안내/비상 음성(mp3) 외부 플레이어 연동 비동기 재생 모듈
   ┣ 📜 visualizer.py                 # 스켈레톤, 가이드라인 및 침대 영역 반투명 오버레이 디버깅 UI
   ┗ 📜 patrol_planner.py             # 복도-병실 단순 순회용 경량 백업 플래너
```

---

## 3. 핵심 컴포넌트별 기능 분석

### 📡 상위 웹 관제 서버 (`hospital_robot_server`)

- **동적 순찰 계획 제어 (Dynamic Task Planning)**: 하드코딩된 위치 데이터 구조를 탈피하고 `plan_table` DB와 동적 연동합니다. 특히 `rooms` 테이블을 실시간 추적하여 병동이 원격으로 폐쇄 상태(`existence=FALSE`)로 전환되면 해당 병동 구역을 관제 화면에서 비활성화하고, 순찰 리스트 생성 시 목록에서 자동으로 예외 처리하는 데이터 필터링 알고리즘을 수행합니다.
- **코드 블루(Code Blue) 다중 로봇 자율 협업**: 순찰 중인 1번 로봇이 현장에서 위급 상황(`emergency_alert`)을 감지하여 웹 대시보드 전역에 긴급 경보 팝업을 발생시키면, 관제자 승인 즉시 1번 로봇의 현재 위치 AMCL 좌표 스냅샷을 추출합니다. 추출된 위치를 대기 상태인 6번 협업 로봇에게 액션 메시지로 송신하여 Nav2 `NavigateToPose` 제어로 특정 응급 구역에 강제 백업 급파시키는 다중 에이전트 오케스트레이션을 구현했습니다.
- **가상 라인 벡터 외적 기반 혼잡도 계산**: 병실 입구 카메라 데이터 스트림에 YOLOv8 및 ByteTrack 추적 객체를 매핑합니다. 설정된 가상 선 펜스 좌표(`virtual_line.json`)와 이동 객체의 실시간 좌표 벡터 간 외적(Cross Product) 연산의 부호 변화를 추적하여 들어오고 나가는 인원을 판별하고, 병동 내부 실시간 잔류 인원수 및 체류 상태를 계산합니다.
- **안정적인 계층형 폴백 설계 (Auto-Mocking)**: 런타임 시작 시 호스트 환경의 의존성을 스스로 검증합니다. ROS 2 환경 소싱의 부재, MySQL 인스턴스 연결 실패, GPU 가속 및 YOLO 라이브러리 미설치 환경이 감지될 경우 시스템이 즉시 인메모리(In-Memory) 가상 Stub 데이터 모드로 안전하게 전환되어, 로봇 하드웨어가 없는 일반 PC 환경에서도 전체 백엔드와 UI 동작성을 완벽히 검증할 수 있도록 설계되었습니다.

### 🤖 하위 로봇 제어 시스템 (`rokey_patrol`)

- **상태 기반 자율 언도킹 시퀀스 (`patrol_node_april`)**: 로봇의 전원 관리 시스템으로부터 배터리 상태 플래그를 상시 구독합니다. 현재 도킹 스테이션에서 충전 결합 중(`is_docked`)으로 확인될 경우, 자율 주행 태스크 시작 직전 역방향 속도 제어 벡터를 인가하여 강제 후진 언도킹을 기동하고 주행 안전 구역을 확보한 뒤 임무를 할당하는 마스터 스케줄링 시퀀스를 자동 수행합니다.
- **동시 다자유도 P-Control 영점 보정 (`enji_auto_localization`)**: 카메라 영상 스트림에서 AprilTag를 검출한 뒤, 로봇 하드웨어 중심 기준 오차 각도(yaw), 좌우 편차 오프셋(tx), 실측 보정 거리(distance)를 산출합니다. 세 가지 기하학적 오차 변수를 독립 분리하지 않고 하나의 연산 루프 안에서 동시에 제어 출력 속도(Twist)로 피드백하는 동시 P-Control 주행을 정밀 설계하여 태그 정면 1.0m 목표 지점에 안착시키고, 5개 샘플의 동차 변환 행렬 평균값을 연산하여 AMCL 영점을 보정합니다.
- **점수 누적 기반 포즈 분류 알고리즘 (`pose_classifier`)**: YOLOv8-pose 모델을 활용해 사람의 17개 관절 골격 포인트 데이터를 실시간 추출합니다. 단순 한 가지 조건에 의존하는 룰 기반 분류를 배제하고 어깨-골반축의 기울기, 바운딩 박스 종횡비, 머리-골반 Y축 높이 정규화 비율, 주성분 분석(PCA) 기반 키포인트 선형성 비율 데이터를 결합한 Score Voting 방식을 구현하여 연산 오탐을 차단하고 바닥 낙상 환자 상태(Lying)를 강건하게 분류합니다.
- **시한성 비상 대응 루프 (`amr_detect`)**: 순찰 주행 중 바닥에 쓰러진 환자 상태가 인지되면 로봇의 모션을 즉시 정지시키고 전신 정렬 기동을 수행한 후, `audio_manager`를 통해 비동기로 "일어나세요" 안내 음성을 쏩니다. 내부 타이머 스레드 기반의 응답 대기 시간(10초) 동안 환자의 움직임이나 자세 회복이 관측되지 않을 경우 지체 없이 `emergency_alert` 토픽을 발행하여 중앙 웹 관제 서버에 전역 비상 알림을 송신합니다.

---

## 4. 데이터베이스 스키마 및 ROS 2 인터페이스 명세

### 📊 MySQL 데이터베이스 테이블 관계 구조

| 테이블 | 설명 | 주요 컬럼 |
|---|---|---|
| `users` | 관제 시스템 접속 엔지니어 및 관리자 인증 정보 | `user_id`, `username`, `password` |
| `rooms` | 실시간 병동 폐쇄/가동 마스터 스위치 정보 | `room_id`, `room_name`, `existence` |
| `patients` | 입원 환자 인적 정보 및 등급 관리 | `patient_id`, `room_name`, `patient_position`, `patient_state` (0: 정상, 1: 주의, 2: 응급) |
| `emergency` | 실시간 발생 비상 이벤트 수집 로그 기록 | `emergency_id`, `patient_position`, `patient_state` |
| `plan_table` | 로봇 고유 번호별 동적 할당 순찰 웨이포인트 물리 좌표 데이터 | `plan_ID`, `robot_num`, `room_name`, `x_pose`, `y_pose`, `yaw`, `fulfill` (0: 이동만, 1: 도착 후 비전 감지) |

### 📡 ROS 2 통신 토픽 및 액션 명세

**Subscriptions (하위 데이터 수집):**

| 토픽 | 메시지 타입 | 설명 |
|---|---|---|
| `/robot1/battery_state` | `sensor_msgs/BatteryState` | 도킹 충전 확인 및 전압 데이터 수집 |
| `/robot1/odom` | `nav_msgs/Odometry` | 로봇 인코더 기반 데드 레코닝 좌표 확인 |
| `/robot1/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 맵 기준 글로벌 추정 좌표 스냅샷 추출 |
| `/robot1/oakd/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | 관제 모니터링용 웹 스트리밍 원격 전송 |
| `/robot1/emergency_alert` | `std_msgs/String` | 비전 노드가 생성한 현장 위급상황("위급상황") 문자열 수신 |
| `/robot1/search_done` | `std_msgs/String` | 스캔 완료 및 현장 정상 상태("문제없음") 문자열 수신 |

**Publications / Actions (상위 명령 하달 및 제어):**

| 토픽/액션 | 메시지 타입 | 설명 |
|---|---|---|
| `/robot1/cmd_vel` | `geometry_msgs/Twist` | 웹 제어 탭 인터페이스를 통한 수동 원격 조이스틱 제어 출력 |
| `/patrol_cmd` | `std_msgs/String` | 웹 관제단에서 가동 병동 필터링 알고리즘을 거쳐 조립된 순찰 태스크 JSON 문자열 전달 |
| `/robot6/navigate_to_pose` | `nav2_msgs/NavigateToPose` (Action) | 코드 블루 연동 시 1번 로봇의 좌표를 추종해 6번 협업 로봇을 강제 급파하기 위한 Nav2 액션 클라이언트 인터페이스 |

---

## 5. 🚀 설치 및 구동 방법

이 프로젝트는 웹 관제 서버와 ROS 2 로봇 시스템이 서로 데이터를 상시 공유하므로 두 시스템을 동시 가동해야 완벽하게 연동됩니다.

### 5-1. 환경 설정 및 의존성 설치

```bash
# Python 라이브러리 설치
cd hospital_robot_server
pip install -r requirements.txt

# PM2 설치 (웹 서버 백그라운드 무중단 가동용)
npm install -g pm2
```

### 5-2. 관제 데이터베이스 초기화 (MySQL)

```sql
CREATE DATABASE hospital_robot_db CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
USE hospital_robot_db;

-- 초기 계정 및 필수 샘플 병동 데이터 삽입
INSERT INTO users (username, password) VALUES ('admin', 'admin1234');
INSERT INTO rooms (room_name, existence) VALUES ('병동1', TRUE), ('병동2', TRUE);
```

### 5-3. 터미널별 구동 시퀀스 가이드 (Terminal 1 ~ 8)

시스템을 안정적으로 구동하기 위해 아래 가이드라인의 순서에 따라 별도의 터미널 탭에서 명령어를 가동하십시오. 모든 로봇 주행 패키지는 `robot1` 네임스페이스 격리 환경을 공유합니다.

#### 🌐 제1계층: 상위 중앙 원격 관제 웹 서버 가동

**터미널 WEB** (PM2 프로세스 가디언 기반 백그라운드 실행 및 환경 변수 고정 가동)

```bash
cd hospital_robot_server
pm2 start ecosystem.config.cjs

# 실시간 백엔드 로그 확인이 필요할 경우
pm2 logs hospital-robot
```

> 인프라 검증을 위해 로컬 개발 모드로 직접 실행할 경우: `SECRET_KEY="hospital-robot-secret" python3 app.py` 수행 후 `http://localhost:5002` 접속

#### 🤖 제2계층: 로봇 에이전트 및 내비게이션 구동 (Robot1 네임스페이스 격리 가동)

**터미널 1** (전역 로컬라이제이션 맵 서버 가동):
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=/home/rokey/rokey_ws/maps/hospital_map.yaml
```

**터미널 2** (로봇 내부 모델 및 상태 RViz 렌더링 시각화 인프라 구동):
```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot1
```

**터미널 3** (내비게이션 메인 스택 가동 - Nav2 스택):
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
```

**터미널 4** (비전 인지 및 360도 스캔 상태 제어 노드 실행 - `amr_detect`):
```bash
ros2 run rokey_patrol amr_detect --ros-args -r __ns:=/robot1
```

**터미널 5** (AprilTag 정렬 및 자동 영점 보정 노드 실행 - `enji_auto_localization`):
```bash
ros2 run rokey_patrol enji_auto_localization --ros-args -r __ns:=/robot1
```

**터미널 6** (웹 명령 수신 및 자동 언도킹 마스터 스케줄러 노드 실행 - `patrol_node_april`):
```bash
ros2 run rokey_patrol patrol_node_april --ros-args -r __ns:=/robot1
```

**터미널 7** (실시간 순찰 상태 모니터링 토픽 에코 - 선택사항):
```bash
ros2 topic echo /patrol_status
```

**터미널 8** (응급 상황 오버라이드용 키보드 수동 조작 제어 결합 - 선택사항):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

---

## 6. ❓ 문제 해결 (Troubleshooting)

| 증상 / 에러 메시지 | 원인 파악 | 해결 방법 |
|---|---|---|
| [DB] MySQL 연결 실패 | MySQL 프로세스 미실행 또는 `app.py` DB 정보 오기입 | `sudo systemctl start mysql` 실행 또는 `app.py` 접속 계정 정보 수정 |
| [ROS2] 미설치 → Mock 모드 | 해당 터미널 인스턴스에 ROS 2 환경 변수가 소싱되지 않음 | 서버 실행 전 `source /opt/ros/humble/setup.bash` 환경 활성화 |
| 카메라 index out of range | 시스템에 호환되는 USB 카메라 또는 OAK-D 장치가 물리적 미연결 상태 | 장치 포트 하드웨어 연결 확인 또는 소스 코드 내 카메라 오프셋 인덱스 수정 |
| `ModuleNotFoundError: ultralytics` | 인원 계측 스레드 구동을 위한 YOLO 라이브러리 가상환경 누락 | `pip install ultralytics` 명령어로 의존 패키지 설치 |
| 관제 페이지 로그인 튕김 현상 | Flask 암호화 쿠키 Key가 가동 시마다 매번 초기화 | `ecosystem.config.cjs` 내부 `SECRET_KEY` 상수를 하드코딩 형태로 완전 고정 |
