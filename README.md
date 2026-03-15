# SimpleVehicleArduino

---

## 한국어

### 프로젝트 개요

**SimpleVehicleArduino**는 Arduino Uno에 조이스틱 쉴드(YwRobot JoyStick Shield V1)와 CAN 버스 쉴드(DiyMore CAN-BUS Shield V1.2, MCP2515)를 연결하여 차량 동역학 모델을 실시간으로 시뮬레이션하고, 계산된 신호를 CAN 버스로 전송하는 프로젝트입니다.

조이스틱을 통해 스로틀/브레이크/조향 입력을 받아, 엔진·구동계·타이어·횡방향 거동(바이사이클 모델)을 포함한 차량 역학 모델을 매 루프 업데이트하며 CAN 프레임을 100 Hz 주기로 송출합니다.

---

### 하드웨어 구성

| 부품 | 사양 |
|------|------|
| MCU | Arduino Uno |
| 조이스틱 쉴드 | YwRobot JoyStick Shield V1 |
| CAN 버스 쉴드 | DiyMore CAN-BUS Shield V1.2 (MCP2515) |

#### 핀 매핑

| 핀 | 기능 |
|----|------|
| A0 | 조이스틱 X → 조향 입력 |
| A1 | 조이스틱 Y → 스로틀(전진) / 브레이크(후진) |
| D6 | 조이스틱 SW (버튼) |
| D2 | SW1 (CAN INT — 미사용) |
| D3 | SW2 |
| D4 | SW3 |
| D5 | SW4 |
| D7 | SW5 |
| D10 | CAN CS (SPI 칩 선택) |
| D11 | MOSI |
| D12 | MISO |
| D13 | SCK |

---

### 소프트웨어 구성

#### 사용 라이브러리

- **MCP_CAN** by Cory Fowler ([GitHub](https://github.com/coryjfowler/MCP_CAN_lib))  
  Arduino 라이브러리 매니저에서 `MCP_CAN` 검색 후 설치

#### 주요 모듈 및 동작

| 모듈 | 설명 |
|------|------|
| `readJoystick()` | A0/A1 ADC 값 읽어 스로틀·브레이크·조향 각도 계산 (데드존 처리 포함) |
| `engineTorque()` | RPM 기반 포물선형 엔진 토크 커브 (스로틀 입력으로 스케일) |
| `updateAckermann()` | 애커먼 기하학으로 4륜 개별 조향각 계산 |
| `normalForces()` | 종방향 무게 이동을 반영한 4륜 법선력 계산 |
| `integrateWheel()` | 반암시적 오일러법(semi-implicit Euler)으로 휠 각속도 적분 (저속 수치 불안정 방지) |
| `updateDynamics()` | 매 루프 호출되는 메인 동역학 업데이트 (종/횡방향 통합) |
| `sendCANFrames()` | 계산 결과를 CAN 프레임으로 인코딩 후 100 Hz로 전송 |

#### 차량 모델

- **구동 방식**: RWD (후륜 구동), 오픈 디퍼렌셜
- **종방향 모델**: 타이어 종방향 슬립 → 타이어 힘 → 차체 가속도 → 차속 적분
- **횡방향 모델**: 2-DOF 바이사이클 모델 (횡속도·요레이트), 타이어 슬립각 기반 횡력 계산
- **타이어 모델**: 선형 슬립 강성 + 마찰 한계 클램프 (Fiala 단순화)
- **에어로**: 항력(Cd) 적용
- **롤링 저항**: 차체 레벨에서 직접 적용

---

### CAN 프레임 맵 (500 kbps, Standard 11-bit ID)

| CAN ID | 이름 | Byte 0-1 | Byte 2-3 | Byte 4-5 | Byte 6-7 |
|--------|------|----------|----------|----------|----------|
| 0x100 | Engine | RPM (uint16) | Torque × 10 (int16, Nm) | Byte4: Throttle (uint8, 0–255) · Byte5: Brake (uint8, 0–255) | 0x0000 |
| 0x101 | Wheel FL | Speed × 100 (int16, km/h) | Slip × 1000 (int16) | Steer × 10 (int16, deg) | 0x0000 |
| 0x102 | Wheel FR | ↑ 동일 | ↑ 동일 | ↑ 동일 | 0x0000 |
| 0x103 | Wheel RL | ↑ 동일 | ↑ 동일 | ↑ 동일 | 0x0000 |
| 0x104 | Wheel RR | ↑ 동일 | ↑ 동일 | ↑ 동일 | 0x0000 |
| 0x200 | Steering | SW × 10 (int16, deg) | RW × 100 (int16, deg) | 0x0000 | 0x0000 |
| 0x201 | Lateral | ay × 100 (int16, m/s²) | Yaw rate × 1000 (int16, rad/s) | Side-slip × 100 (int16, deg) | 0x0000 |
| 0x300 | Vehicle | Speed × 100 (int16, km/h) | Accel × 100 (int16, m/s²) | 0x0000 | 0x0000 |

---

### 차량 파라미터 (기본값)

| 항목 | 기본값 | 단위 |
|------|--------|------|
| 최대 토크 | 200 | Nm |
| 최대 RPM | 6000 | rpm |
| 공회전 RPM | 800 | rpm |
| 피크 토크 RPM | 3500 | rpm |
| 변속비 | 3.50 | — |
| 최종 감속비 | 3.73 | — |
| 구동계 효율 | 0.92 | — |
| 차량 중량 | 1500 | kg |
| 휠베이스 | 2.70 | m |
| 트레드 | 1.50 | m |
| 항력 계수(Cd) | 0.30 | — |
| 타이어 반경 | 0.315 | m |
| 최대 조향각(SW) | ±180 | deg |
| 조향비 | 16 | — |
| 최대 제동력 | 8000 | N |

> 파라미터는 `VehicleDynamicsCAN.ino` 상단의 `// Vehicle parameters` 섹션에서 자유롭게 수정할 수 있습니다.

---

### 시리얼 대시보드

115200 baud로 시리얼 연결 시, ANSI 이스케이프 코드를 활용한 실시간 대시보드가 2 Hz(0.5초 주기)로 출력됩니다.  
ANSI 표시가 제대로 동작하려면 `screen`, `minicom`, `picocom` 등의 터미널 에뮬레이터를 사용하세요.

출력 항목:
- 운전자 입력 (스로틀, 브레이크, 조향각)
- 애커먼 조향각 (FL/FR/RL/RR)
- 엔진 RPM, 토크, 구동 토크
- 브레이크 토크 (앞/뒤)
- 법선력 (4륜)
- 휠별 각속도, 슬립, 종방향 타이어 힘, 속도
- 종방향 힘 균형 (FxSum, 항력, 롤링 저항, 합력)
- 차량 속도 및 가속도
- 횡방향 모델 입출력 (슬립각, 횡력, 요레이트, 횡가속도, 사이드슬립각)

---

### 빌드 및 업로드 방법

1. **Arduino IDE** 또는 **PlatformIO** 사용
2. 라이브러리 매니저에서 `MCP_CAN` (by Cory Fowler) 설치
3. `CAN_OSC_FREQ`를 사용하는 CAN 쉴드의 수정 발진기 주파수에 맞게 설정  
   - DiyMore CAN-BUS Shield V1.2는 일반적으로 **8 MHz** 크리스탈이 탑재되어 출하됩니다. 이 경우 `MCP_8MHZ`로 변경하세요.  
   - 16 MHz 오실레이터가 탑재된 제품이라면 `MCP_16MHZ` (코드 기본값)를 그대로 사용합니다.
4. 보드: **Arduino Uno** 선택 후 업로드

---

### 라이선스

이 프로젝트는 별도의 라이선스 파일이 없습니다. 사용 전 저작권자에게 확인하세요.

---
---

## English

### Project Overview

**SimpleVehicleArduino** is a real-time vehicle dynamics simulator running on an Arduino Uno, with a YwRobot JoyStick Shield V1 for driver input and a DiyMore CAN-BUS Shield V1.2 (MCP2515) for CAN bus output.

The joystick provides throttle, brake, and steering commands. On every loop iteration, a vehicle dynamics model — covering the engine, driveline, tyre mechanics, and lateral dynamics (bicycle model) — is updated and its outputs are transmitted as CAN frames at 100 Hz.

---

### Hardware

| Component | Specification |
|-----------|---------------|
| MCU | Arduino Uno |
| Joystick shield | YwRobot JoyStick Shield V1 |
| CAN shield | DiyMore CAN-BUS Shield V1.2 (MCP2515) |

#### Pin Mapping

| Pin | Function |
|-----|----------|
| A0 | Joystick X → Steering input |
| A1 | Joystick Y → Throttle (push forward) / Brake (pull back) |
| D6 | Joystick SW (push button) |
| D2 | SW1 (CAN INT — unused) |
| D3 | SW2 |
| D4 | SW3 |
| D5 | SW4 |
| D7 | SW5 |
| D10 | CAN CS (SPI chip select) |
| D11 | MOSI |
| D12 | MISO |
| D13 | SCK |

---

### Software

#### Required Library

- **MCP_CAN** by Cory Fowler ([GitHub](https://github.com/coryjfowler/MCP_CAN_lib))  
  Install via the Arduino Library Manager: search for `MCP_CAN`.

#### Key Functions

| Function | Description |
|----------|-------------|
| `readJoystick()` | Reads A0/A1, applies dead-zone, outputs throttle, brake, and steering-wheel angle |
| `engineTorque()` | Parabolic torque curve scaled by throttle input |
| `updateAckermann()` | Computes individual wheel steer angles using Ackermann geometry |
| `normalForces()` | Calculates per-wheel normal loads including longitudinal weight transfer |
| `integrateWheel()` | Semi-implicit Euler integration of wheel speed (numerically stable at low speed) |
| `updateDynamics()` | Main dynamics update called every loop: longitudinal + lateral integration |
| `sendCANFrames()` | Encodes all state variables into CAN frames and transmits at 100 Hz |

#### Vehicle Model

- **Drive layout**: Rear-Wheel Drive (RWD) with open differential
- **Longitudinal model**: Tyre longitudinal slip → tyre force → body acceleration → speed integration
- **Lateral model**: 2-DOF bicycle model (lateral velocity + yaw rate), slip-angle-based lateral tyre forces
- **Tyre model**: Linear slip stiffness with friction-limit clamping (simplified Fiala)
- **Aerodynamics**: Quadratic drag force (Cd model)
- **Rolling resistance**: Applied at the vehicle body level

---

### CAN Frame Map (500 kbps, Standard 11-bit ID)

| CAN ID | Name | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Bytes 6–7 |
|--------|------|-----------|-----------|-----------|-----------|
| 0x100 | Engine | RPM (uint16) | Torque × 10 (int16, Nm) | Byte4: Throttle (uint8, 0–255) · Byte5: Brake (uint8, 0–255) | 0x0000 |
| 0x101 | Wheel FL | Speed × 100 (int16, km/h) | Slip × 1000 (int16) | Steer angle × 10 (int16, deg) | 0x0000 |
| 0x102 | Wheel FR | same layout | same layout | same layout | 0x0000 |
| 0x103 | Wheel RL | same layout | same layout | same layout | 0x0000 |
| 0x104 | Wheel RR | same layout | same layout | same layout | 0x0000 |
| 0x200 | Steering | Steering-wheel × 10 (int16, deg) | Road-wheel × 100 (int16, deg) | 0x0000 | 0x0000 |
| 0x201 | Lateral | Lat. accel × 100 (int16, m/s²) | Yaw rate × 1000 (int16, rad/s) | Side-slip × 100 (int16, deg) | 0x0000 |
| 0x300 | Vehicle | Speed × 100 (int16, km/h) | Longitudinal accel × 100 (int16, m/s²) | 0x0000 | 0x0000 |

---

### Vehicle Parameters (defaults)

| Parameter | Default | Unit |
|-----------|---------|------|
| Peak engine torque | 200 | Nm |
| Engine redline | 6000 | rpm |
| Idle RPM | 800 | rpm |
| Peak-torque RPM | 3500 | rpm |
| Gear ratio | 3.50 | — |
| Final drive ratio | 3.73 | — |
| Drivetrain efficiency | 0.92 | — |
| Vehicle mass | 1500 | kg |
| Wheelbase | 2.70 | m |
| Track width | 1.50 | m |
| Drag coefficient (Cd) | 0.30 | — |
| Loaded tyre radius | 0.315 | m |
| Max steering-wheel angle | ±180 | deg |
| Steering ratio | 16 | — |
| Max brake force | 8000 | N |

> All parameters are defined at the top of `VehicleDynamicsCAN.ino` under the `// Vehicle parameters` section and can be adjusted freely.

---

### Serial Dashboard

At 115200 baud, an ANSI in-place dashboard is printed at 2 Hz (every 500 ms). Use a terminal emulator that supports ANSI escape codes (`screen`, `minicom`, `picocom`) for best results.

Dashboard output includes:
- Driver inputs (throttle, brake, steering angle)
- Ackermann wheel steer angles (FL/FR/RL/RR)
- Engine RPM, torque, per-wheel drive torque
- Brake torque (front / rear)
- Per-wheel normal forces
- Per-wheel angular velocity, slip, longitudinal tyre force, peripheral speed
- Longitudinal force balance (tyre forces, aero drag, rolling resistance, net force)
- Vehicle speed and longitudinal acceleration
- Lateral model state (slip angles, lateral forces, yaw rate, lateral acceleration, side-slip angle)

---

### Build & Upload

1. Use **Arduino IDE** or **PlatformIO**.
2. Install `MCP_CAN` (by Cory Fowler) via the Library Manager.
3. Set `CAN_OSC_FREQ` to match your CAN shield's crystal:
   - DiyMore CAN-BUS Shield V1.2 typically ships with an **8 MHz** crystal; change the define to `MCP_8MHZ` if that is the case.
   - If your shield has a **16 MHz** oscillator, keep the code default `MCP_16MHZ`.
4. Select board **Arduino Uno** and upload.

---

### License

No explicit license file is included in this repository. Please contact the author before use.
