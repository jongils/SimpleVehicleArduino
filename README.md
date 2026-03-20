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
| 조이스틱 쉴드 | [YwRobot JoyStick Shield V1](http://wiki.ywrobot.net/index.php?title=(SKU:ARD040401)JoyStick_Shield%E6%B8%B8%E6%88%8F%E6%91%87%E6%9D%86%E6%8C%89%E9%94%AE%E6%89%A9%E5%B1%95%E6%9D%BF_%E9%81%A5%E6%8E%A7%E5%99%A8) |
| CAN 버스 쉴드 | [DiyMore CAN-BUS Shield V1.2 (MCP2515)](https://m.blog.naver.com/icbanq/220948945676) |

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
| `integrateWheel()` | 명시적 오일러(explicit Euler)로 휠 각속도 적분 — `tyreFx()` 결과를 직접 사용하는 단일 경로 |
| `updateDynamics()` | 매 루프 호출되는 메인 동역학 업데이트 (종/횡방향 통합) — `USE_RK4` 플래그로 적분 방식 선택 |
| `sendCANFrames()` | 계산 결과를 CAN 프레임으로 인코딩 후 100 Hz로 전송 |

#### 수치 적분 방법

차량 동역학 상태 변수(차속, 횡속도, 요레이트)의 시간 적분에는 두 가지 방법을 지원합니다.
`VehicleDynamicsCAN.ino` 상단의 `#define USE_RK4` 값(0 또는 1)으로 선택합니다.

##### Variable-step 전향 오일러 (USE_RK4 = 0)

```
x(t+dt) = x(t) + f(x, t) · dt
```

- **단계당 미분 계산 횟수**: 1회
- **국소 절단 오차**: O(dt²), **전역 오차**: O(dt)
- `dt`는 `micros()`로 측정한 실제 루프 경과 시간 → 루프 속도에 자동 적응 (variable-step)
- 구현이 단순하고 AVR에서 연산 부담이 가장 적음

##### 룬게-쿠타 4차 방법 (RK4, USE_RK4 = 1, 기본값)

```
k1 = f(x,           t)
k2 = f(x + dt/2·k1, t + dt/2)
k3 = f(x + dt/2·k2, t + dt/2)
k4 = f(x + dt·k3,   t + dt)
x(t+dt) = x(t) + (dt/6)·(k1 + 2k2 + 2k3 + k4)
```

- **단계당 미분 계산 횟수**: 4회
- **국소 절단 오차**: O(dt⁵), **전역 오차**: O(dt⁴)
- 동일한 `dt`에서 오일러 대비 훨씬 정확
- AVR에서 오일러 대비 약 3–4× 연산 비용 증가

##### 적분 방법 비교

| 항목 | Variable-step Euler | RK4 (4차) |
|------|---------------------|-----------|
| 적용 대상 | 차속·횡속도·요레이트·휠 각속도 | 차속·횡속도·요레이트·휠 각속도 |
| 전역 오차 | O(dt) | O(dt⁴) |
| 미분 계산 횟수/스텝 | 1 | 4 |
| AVR 연산 부담 | 낮음 | 중간 (~3–4×) |
| 선택 이유 | 단순·고속 | 정확도 우선 |

---

#### 차량 모델

- **구동 방식**: RWD (후륜 구동), 오픈 디퍼렌셜
- **종방향 모델**: 타이어 종방향 슬립 → 타이어 힘 → 차체 가속도 → 차속 적분
- **횡방향 모델**: 2-DOF 바이사이클 모델 (횡속도·요레이트), 타이어 슬립각 기반 횡력 계산
- **타이어 모델**: 선형 슬립 강성 + 단순 μFz 캡 (종/횡방향 독립 적용, 마찰원 미사용)
- **에어로**: 항력(Cd) 적용
- **롤링 저항**: 휠 레벨에서 적용

#### 저속 정지 처리 (Hard Stop)

- 임계 속도(`HARD_STOP_MS`, 기본값 6 km/h) 이하이며 스로틀 없음 또는 제동 중인 경우, 1차 지수 감쇠로 부드럽게 감속
- 시상수는 `HARD_STOP_TAU_S`(기본값 0.5 s)로 조정 가능 — 값이 클수록 부드럽게 정지
- 속도가 0.05 m/s 미만이 되면 완전히 0으로 스냅

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
| 변속비 | 0.95 | — |
| 최종 감속비 | 3.73 | — |
| 구동계 효율 | 0.92 | — |
| 차량 중량 | 1500 | kg |
| 휠베이스 | 2.70 | m |
| 트레드 | 1.50 | m |
| 항력 계수(Cd) | 0.20 | — |
| 타이어 반경 | 0.315 | m |
| 타이어 종방향 슬립 강성 | 80000 | N |
| 타이어 마찰 계수 (μ) | 0.90 | — |
| 전/후 코너링 강성 | 120000 | N/rad |
| 최대 조향각(SW) | ±180 | deg |
| 조향비 | 16 | — |
| 최대 제동력 | 8000 | N |
| 하드스탑 임계 속도 | 6 | km/h |
| 하드스탑 시상수 | 0.5 | s |

> 파라미터는 `VehicleDynamicsCAN.ino` 상단의 `// Vehicle parameters` 섹션에서 자유롭게 수정할 수 있습니다.

---

### 시리얼 대시보드

115200 baud로 시리얼 연결 시, ANSI 이스케이프 코드를 활용한 실시간 대시보드가 2 Hz(0.5초 주기)로 출력됩니다.
ANSI 표시가 제대로 동작하려면 `screen`, `minicom`, `picocom` 등의 터미널 에뮬레이터를 사용하세요.

| 태그 | 출력 항목 |
|------|-----------|
| `[IN ]` | 스로틀, 브레이크, 조향휠 각도 |
| `[STR]` | 로드휠 각도, FL/FR/RL/RR 개별 조향각 (애커먼) |
| `[ENG]` | RPM, 엔진 토크, 휠당 구동 토크 |
| `[Nz ]` | 4륜 법선력 |
| `[WHL]` | 휠별 각속도, 슬립, 종방향 타이어 힘, 속도 |
| `[LON]` | FxSum, 항력, 롤링 저항, 합력 |
| `[VEH]` | 종방향 가속도, 차속 (km/h, m/s = Vx) |
| `[STR]` | 조향 입력 delta (rad/deg) |
| `[TYR]` | 앞/뒤 타이어 슬립각 (aF, aR), 횡력 (Fyf, Fyr) |
| `[LAT]` | **횡속도 Vy**, 요레이트 yr, 사이드슬립각 beta |
| `[DBG]` | vy_dot (차체 기준 횡가속도), r_dot, ay (타이어 횡력/질량) |

> `vy_dot`과 `ay`의 차이: `vy_dot = ay - r·Vx` (원심가속도 항 포함 여부)

---

### 빌드 및 업로드 방법

1. **Arduino IDE** 또는 **PlatformIO** 사용
2. 라이브러리 매니저에서 `MCP_CAN` (by Cory Fowler) 설치
3. `CAN_OSC_FREQ`를 사용하는 CAN 쉴드의 수정 발진기 주파수에 맞게 설정
   - DiyMore CAN-BUS Shield V1.2는 일반적으로 **8 MHz** 크리스탈이 탑재됩니다. 이 경우 `MCP_8MHZ`로 변경하세요.
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
| Joystick shield | [YwRobot JoyStick Shield V1](http://wiki.ywrobot.net/index.php?title=(SKU:ARD040401)JoyStick_Shield%E6%B8%B8%E6%88%8F%E6%91%87%E6%9D%86%E6%8C%89%E9%94%AE%E6%89%A9%E5%B1%95%E6%9D%BF_%E9%81%A5%E6%8E%A7%E5%99%A8) |
| CAN shield | [DiyMore CAN-BUS Shield V1.2 (MCP2515)](https://m.blog.naver.com/icbanq/220948945676) |

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
| `integrateWheel()` | Single-path explicit Euler integration of wheel angular velocity using `tyreFx()` directly |
| `updateDynamics()` | Main dynamics update called every loop: longitudinal + lateral integration — method selected via `USE_RK4` |
| `sendCANFrames()` | Encodes all state variables into CAN frames and transmits at 100 Hz |

#### Numerical Integration Methods

The integration method for vehicle body and lateral states is selected at compile time via `#define USE_RK4` at the top of `VehicleDynamicsCAN.ino`.

##### Variable-step Forward Euler (USE_RK4 = 0)

```
x(t+dt) = x(t) + f(x, t) · dt
```

- **Derivative evaluations per step**: 1
- **Local truncation error**: O(dt²); **global error**: O(dt)
- `dt` is taken from `micros()`, adapting to the real loop period
- Simplest implementation; lowest computational cost on AVR

##### Runge-Kutta 4th Order — RK4 (USE_RK4 = 1, default)

```
k1 = f(x,           t)
k2 = f(x + dt/2·k1, t + dt/2)
k3 = f(x + dt/2·k2, t + dt/2)
k4 = f(x + dt·k3,   t + dt)
x(t+dt) = x(t) + (dt/6)·(k1 + 2k2 + 2k3 + k4)
```

- **Derivative evaluations per step**: 4
- **Local truncation error**: O(dt⁵); **global error**: O(dt⁴)
- Significantly more accurate than Euler at the same step size
- ~3–4× higher floating-point cost on AVR

##### Method Comparison

| Aspect | Variable-step Euler | RK4 (4th order) |
|--------|---------------------|-----------------|
| Applied to | body speed, Vy, yaw rate, wheel ω | body speed, Vy, yaw rate, wheel ω |
| Global error | O(dt) | O(dt⁴) |
| Derivative evals/step | 1 | 4 |
| AVR compute cost | low | medium (~3–4×) |
| Reason for use | simplicity / speed | accuracy priority |

---

#### Vehicle Model

- **Drive layout**: Rear-Wheel Drive (RWD) with open differential
- **Longitudinal model**: Tyre longitudinal slip → tyre force → body acceleration → speed integration
- **Lateral model**: 2-DOF bicycle model (lateral velocity + yaw rate), slip-angle-based lateral tyre forces
- **Tyre model**: Linear slip stiffness with independent μFz cap per axle (longitudinal and lateral decoupled — no friction circle)
- **Aerodynamics**: Quadratic drag force (Cd model)
- **Rolling resistance**: Applied at the wheel level

#### Low-speed Hard Stop

- When speed falls below `HARD_STOP_MS` (default 6 km/h) with no throttle or under braking, a 1st-order exponential decay smoothly reduces speed to zero
- Time constant is `HARD_STOP_TAU_S` (default 0.5 s) — larger value = smoother stop
- Final snap to exactly zero when speed drops below 0.05 m/s

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
| Gear ratio | 0.95 | — |
| Final drive ratio | 3.73 | — |
| Drivetrain efficiency | 0.92 | — |
| Vehicle mass | 1500 | kg |
| Wheelbase | 2.70 | m |
| Track width | 1.50 | m |
| Drag coefficient (Cd) | 0.20 | — |
| Loaded tyre radius | 0.315 | m |
| Longitudinal slip stiffness | 80000 | N |
| Peak friction coefficient (μ) | 0.90 | — |
| Front/Rear cornering stiffness | 120000 | N/rad |
| Max steering-wheel angle | ±180 | deg |
| Steering ratio | 16 | — |
| Max brake force | 8000 | N |
| Hard-stop threshold | 6 | km/h |
| Hard-stop time constant | 0.5 | s |

> All parameters are defined at the top of `VehicleDynamicsCAN.ino` under the `// Vehicle parameters` section.

---

### Serial Dashboard

At 115200 baud, an ANSI in-place dashboard is printed at 2 Hz (every 500 ms). Use a terminal emulator that supports ANSI escape codes (`screen`, `minicom`, `picocom`) for best results.

| Tag | Contents |
|-----|----------|
| `[IN ]` | Throttle, brake, steering-wheel angle |
| `[STR]` | Road-wheel angle, per-wheel steer angles (FL/FR/RL/RR, Ackermann) |
| `[ENG]` | RPM, engine torque, per-wheel drive torque |
| `[Nz ]` | Per-wheel normal forces |
| `[WHL]` | Per-wheel angular velocity, slip, longitudinal tyre force, peripheral speed |
| `[LON]` | FxSum, aero drag, rolling resistance, net force |
| `[VEH]` | Longitudinal acceleration, vehicle speed (km/h, m/s = Vx) |
| `[STR]` | Front steer input delta (rad / deg) |
| `[TYR]` | Front/rear tyre slip angles (aF, aR), lateral forces (Fyf, Fyr) |
| `[LAT]` | **Lateral velocity Vy**, yaw rate yr, side-slip angle beta |
| `[DBG]` | vy_dot (body-frame lat. accel incl. centripetal), r_dot, ay (tyre force / mass) |

> Difference between `vy_dot` and `ay`: `vy_dot = ay − r·Vx` (includes centripetal term)

---

### Build & Upload

1. Use **Arduino IDE** or **PlatformIO**.
2. Install `MCP_CAN` (by Cory Fowler) via the Library Manager.
3. Set `CAN_OSC_FREQ` to match your CAN shield's crystal:
   - DiyMore CAN-BUS Shield V1.2 typically ships with an **8 MHz** crystal; change to `MCP_8MHZ` if needed.
   - If your shield has a **16 MHz** oscillator, keep the default `MCP_16MHZ`.
4. Select board **Arduino Uno** and upload.

---

### License

No explicit license file is included in this repository. Please contact the author before use.
