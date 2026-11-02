# 🏎️ 2025 SEA:ME HACKATHON — Autonomous Driving Project

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%2F%20Foxy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Donkey Car](https://img.shields.io/badge/Framework-Donkey%20Car-orange?style=for-the-badge)

**2025 SEA:ME 자율주행 해커톤**을 위한 실시간 비전 인식, 정지선 검출 및 지능형 조향 제어 시스템입니다.  
Donkey Car 프레임워크와 ROS 2를 연동하여 트랙 주행, 체커보드/정지선 정지·재출발, 자동-수동 전환 안전 제어를 구현했습니다.

</div>

---

## 📌 Key Highlights

- **실시간 비전 인식 (Vision Processing)**
  - 카메라 영상 스트리밍 노드(`webcam_pub.py`) 구현
  - BEV(Bird's Eye View) 변환 및 슬라이딩 윈도우(Sliding Window) 기반 차선 추적
  - 노란색 차선 우선 인식 필터링 및 체커보드/정지선 인식 (`detect_checkerboard`, `stopline_detect.cpp`)
- **지능형 주행 제어 (Adaptive Control)**
  - 배터리 전압 변화에 대응하는 적응형 PID 조향 제어 (`auto_adaptive_pid.cpp`)
  - 정지선 인식 시 정지 후 딜레이 재출발 및 3번째 정지선 구간 강제 조향 알고리즘
  - 조이스틱 기반 A/B 버튼 수동/자동 전환 안전 제어 로직 (`main.py`)
- **임베디드 하드웨어 텔레메트리 (Hardware Telemetry)**
  - INA219 전압/전류 센서와 SSD1306 OLED 디스플레이를 I2C로 제어하여 실시간 전압 및 배터리 잔량(%) 대시보드 표출

---

## 🏗️ Repository Architecture

```text
2025-seame-hackathon/
├── README.md                          # 프로젝트 개요 및 매뉴얼
├── ros2_ws/                           # ROS 2 워크스페이스
│   └── src/
│       ├── auto_control/              # [C++] 주행 제어 및 정지선 인식 패키지
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── scripts/
│       │   │   └── main.py            # 메인 컨트롤러 노드 (조이스틱 / 모드 전환)
│       │   ├── src/
│       │   │   ├── auto.cpp           # 대회 메인 자율주행 제어 노드
│       │   │   └── stopline_detect.cpp# 정지선 및 체커보드 인식 노드
│       │   └── variants/              # 실험 및 튜닝 이력
│       │       ├── auto_adaptive_pid.cpp     # 배터리 보정 PID 제어
│       │       ├── auto_shortcut.cpp         # 지름길 강제 조향 제어
│       │       └── auto_yellow_priority.cpp  # 노란색 차선 가중치 제어
│       └── lane_detection/            # [Python] 비전 차선 검출 패키지
│           ├── package.xml
│           ├── setup.py
│           └── lane_detection/
│               ├── webcam_pub.py      # 카메라 프레임 퍼블리셔
│               ├── webcam_sub.py      # 슬라이딩 윈도우 차선 인식기
│               └── auto_manual_switch.py
├── donkeycar/                         # Donkey Car RC 플랫폼 설정
│   └── mycar/
│       ├── calibrate.py               # 스티어링/스로틀 캘리브레이션
│       ├── config.py                  # 기본 파라미터 구성
│       ├── manage.py                  # Donkey Car 구동 진입점
│       └── myconfig.py                # 튜닝된 커스텀 파라미터
└── hardware/                          # 온보드 텔레메트리
    ├── battery_oled_monitor.py        # INA219 + SSD1306 배터리 모니터링
    └── display_off.py                 # 디스플레이 종료 스크립트
```

---

## 🚀 Getting Started

### 1. ROS 2 Workspace Build
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Run Autonomous Driving Pipeline
```bash
# Terminal 1: 비전 카메라 노드 실행
ros2 run lane_detection webcam_pub

# Terminal 2: 차선 인식 노드 실행
ros2 run lane_detection webcam_sub

# Terminal 3: 자율주행 제어기 실행
ros2 run auto_control auto
```

### 3. Hardware Battery Monitor
```bash
python3 hardware/battery_oled_monitor.py
```

---

## 🛠️ Hardware Requirements
- **Vehicle**: Donkey Car RC Platform
- **SBC**: Raspberry Pi 4 / Single Board Computer (Ubuntu Linux, ROS 2)
- **Sensors**: USB / CSI Camera, INA219 Current/Voltage Sensor
- **Display**: SSD1306 0.96 inch I2C OLED Display
