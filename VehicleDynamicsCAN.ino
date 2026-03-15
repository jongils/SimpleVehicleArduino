/*
 * VehicleDynamicsCAN.ino
 *
 * Hardware:
 *   Arduino Uno
 *   + JoyStick Shield V1 (YwRobot)
 *   + CAN-BUS Shield V1.2 (DiyMore, MCP2515)
 *
 * Pin Mapping:
 *   A0  – Joystick X  → Steering (right +, left -)
 *   A1  – Joystick Y  → Throttle (forward) / Brake (backward)
 *   D6  – Joystick SW
 *   D2  – SW1  (CAN INT — unused; no conflict)
 *   D3  – SW2
 *   D4  – SW3
 *   D5  – SW4
 *   D7  – SW5
 *   D10 – CAN CS
 *   D11 – MOSI
 *   D12 – MISO
 *   D13 – SCK
 *
 * CAN Frame Map (500 kbps, Standard 11-bit ID):
 *   0x100  Engine  : [RPM_H][RPM_L][TRQ_H][TRQ_L][THR][BRK][0][0]
 *   0x101  Wheel FL: [SPD_H][SPD_L][SLP_H][SLP_L][STR_H][STR_L][0][0]
 *   0x102  Wheel FR: (same layout)
 *   0x103  Wheel RL: (same layout)
 *   0x104  Wheel RR: (same layout)
 *   0x200  Steering: [SW_H][SW_L][RW_H][RW_L][0][0][0][0]
 *   0x201  Lateral : [AY_H][AY_L][YR_H][YR_L][BETA_H][BETA_L][0][0]
 *   0x300  Vehicle : [SPD_H][SPD_L][ACC_H][ACC_L][0][0][0][0]
 *
 * Signal resolution:
 *   RPM        – uint16  [rpm]
 *   Torque     – int16   [Nm × 10]
 *   THR / BRK  – uint8   [0–255]
 *   Wheel SPD  – int16   [km/h × 100]
 *   Wheel SLP  – int16   [slip × 1000]
 *   Steer STR  – int16   [road-wheel deg × 10]
 *   Steer SW   – int16   [steering-wheel deg × 10]
 *   Steer RW   – int16   [road-wheel deg × 100]
 *   Veh SPD    – int16   [km/h × 100]
 *   Veh ACC    – int16   [m/s² × 100]
 *
 * Library: MCP_CAN by coryjfowler (mcp_can.h)
 *   Install via Library Manager: "MCP_CAN" by Cory Fowler
 */

#include <SPI.h>
#include <mcp_can.h>

// ============================================================
//  Hardware pins
// ============================================================
#define PIN_JOY_X    A0
#define PIN_JOY_Y    A1
#define PIN_JOY_SW    6
#define PIN_SW1       2   // also CAN INT, unused
#define PIN_SW2       3
#define PIN_SW3       4
#define PIN_SW4       5
#define PIN_SW5       7
#define CAN_CS_PIN   10

// ============================================================
//  CAN configuration
// ============================================================
#define CAN_BAUD        CAN_500KBPS
// DiyMore CAN-BUS Shield V1.2 typically ships with an 8 MHz crystal.
// Change to MCP_8MHZ if that is the case; keep MCP_16MHZ only if your
// shield has a 16 MHz oscillator (the less common variant).
#define CAN_OSC_FREQ    MCP_16MHZ

#define CAN_ID_ENGINE   0x100u
#define CAN_ID_WHL_FL   0x101u
#define CAN_ID_WHL_FR   0x102u
#define CAN_ID_WHL_RL   0x103u
#define CAN_ID_WHL_RR   0x104u
#define CAN_ID_STEER    0x200u
#define CAN_ID_LATERAL  0x201u   // Lateral: ay, yaw rate, side-slip angle
#define CAN_ID_VEHICLE  0x300u

// CAN frame transmission period [ms]
#define CAN_TX_PERIOD_MS   10u   // 100 Hz

// ============================================================
//  Vehicle parameters — edit freely
// ============================================================

// --- Engine ---
const float P_ENG_MAX_TORQUE_NM   = 200.0f;  // Peak torque [Nm]
const float P_ENG_MAX_RPM         = 6000.0f; // Redline [rpm]
const float P_ENG_IDLE_RPM        =  800.0f; // Idle [rpm]
const float P_ENG_PEAK_TQ_RPM     = 3500.0f; // RPM at peak torque

// --- Drivetrain (Rear-Wheel-Drive, open differential) ---
const float P_GEAR_RATIO          =   3.50f; // Current gear ratio
const float P_FINAL_DRIVE_RATIO   =   3.73f; // Final drive ratio
const float P_DRIVETRAIN_EFF      =   0.92f; // Mechanical efficiency

// --- Vehicle body ---
const float P_MASS_KG             = 1500.0f; // Total mass [kg]
const float P_WHEELBASE_M         =   2.70f; // Front–rear axle distance [m]
const float P_TRACK_M             =   1.50f; // Left–right wheel distance [m]
const float P_CG_HEIGHT_M         =   0.50f; // CG height [m]
const float P_CG_TO_FRONT_M       =   1.20f; // CG to front axle [m]

// --- Aerodynamics ---
const float P_AERO_CD             =   0.30f; // Drag coefficient
const float P_FRONTAL_AREA_M2     =   2.20f; // Frontal area [m²]
const float P_AIR_DENSITY         =   1.225f;// Air density [kg/m³]

// --- Wheel & tyre ---
const float P_WHEEL_R_M           =  0.315f; // Loaded wheel radius [m]
const float P_WHEEL_J_KGM2        =   1.50f; // Wheel+tyre inertia [kg·m²]
const float P_TYRE_CS             = 80000.0f;// Longitudinal slip stiffness [N/1]
const float P_TYRE_MU             =   0.90f; // Peak friction coefficient
const float P_TYRE_RR             =  0.015f; // Rolling-resistance coefficient
const float P_TYRE_CF             = 120000.0f;// Front axle cornering stiffness [N/rad] (both wheels)
const float P_TYRE_CR             = 120000.0f;// Rear  axle cornering stiffness [N/rad] (both wheels)

// --- Yaw inertia ---
const float P_VEHICLE_IZ_KGM2    = 2500.0f; // Yaw moment of inertia [kg·m²]

// --- Steering ---
const float P_STEER_MAX_DEG       = 180.0f;  // Max steering-wheel angle each side [°]
const float P_STEER_RATIO         =  16.0f;  // Steering ratio (SW turns / road turns)

// --- Brakes ---
const float P_BRAKE_MAX_N         = 8000.0f; // Max total brake force [N]
const float P_BRAKE_FRONT_BIAS    =   0.60f; // Front proportion

// --- Joystick ADC ---
const int P_JOY_CENTER            =  512;    // Nominal ADC centre value
const int P_JOY_DEADZONE          =   30;    // Dead-zone half-width [counts]

// ============================================================
//  Wheel state
// ============================================================
struct WheelState {
    float omega;      // Angular velocity [rad/s]
    float speed_kmh;  // Peripheral speed  [km/h]
    float slip;       // Longitudinal slip ratio  [-1 … +1]
    float steerDeg;   // Individual steer angle   [°]
};

WheelState wFL, wFR, wRL, wRR;

// ============================================================
//  Vehicle state
// ============================================================
float gSpeed_ms      = 0.0f;  // Longitudinal vehicle speed [m/s]
float gAccel_ms2     = 0.0f;  // Longitudinal acceleration  [m/s²]
float gEngRPM        = P_ENG_IDLE_RPM;
float gEngTq_Nm      = 0.0f;
float gSteerSW       = 0.0f;  // Steering-wheel angle [°]  + = right
float gThrottle      = 0.0f;  // [0 … 1]
float gBrake         = 0.0f;  // [0 … 1]

// Lateral dynamics (bicycle model, right-positive convention)
float gVy_ms         = 0.0f;  // Lateral velocity of CG [m/s]  + = right
float gYawRate_rads  = 0.0f;  // Yaw rate [rad/s]              + = right turn
float gLatAccel_ms2  = 0.0f;  // Lateral acceleration [m/s²]   + = right
float gSideSlip_deg  = 0.0f;  // Side-slip angle β [°]         + = rightward drift

// ---- Diagnostic intermediates (updated in updateDynamics, read by serial) ----
float gDrvTqRear = 0.0f;                              // Drive torque per rear wheel [Nm]
float gBrkTqF    = 0.0f;                              // Brake torque per front wheel [Nm]
float gBrkTqR    = 0.0f;                              // Brake torque per rear wheel  [Nm]
float gNfl=0.0f, gNfr=0.0f, gNrl=0.0f, gNrr=0.0f;   // Per-wheel normal forces [N]
float gFxFL=0.0f, gFxFR=0.0f, gFxRL=0.0f, gFxRR=0.0f; // Per-wheel long. tyre force [N]
float gFxSum_d = 0.0f;  // Sum of all tyre Fx [N]
float gFdrag   = 0.0f;  // Aerodynamic drag [N]
float gFrollB  = 0.0f;  // Rolling resistance (body level) [N]
float gFnet    = 0.0f;  // Net longitudinal force [N]
float gVxLat   = 0.0f;  // Longitudinal speed used in lateral model [m/s]
float gDeltaRad= 0.0f;  // Average front steer angle [rad]
float gAlphaF  = 0.0f;  // Front tyre slip angle [rad]
float gAlphaR  = 0.0f;  // Rear tyre slip angle  [rad]
float gFyf     = 0.0f;  // Front lateral tyre force [N]
float gFyr     = 0.0f;  // Rear lateral tyre force  [N]
float gVyDot   = 0.0f;  // Lateral velocity derivative [m/s²]
float gRDot    = 0.0f;  // Yaw rate derivative [rad/s²]

// ============================================================
//  Timing & CAN
// ============================================================
MCP_CAN CAN0(CAN_CS_PIN);

unsigned long gLastDynUs    = 0;
unsigned long gLastCanMs    = 0;
unsigned long gLastSerialMs = 0;

const float DT_MAX = 0.050f; // Safety clamp on integration step [s]

// Serial print period [ms] — lower = faster scroll, higher = easier to read
const unsigned long SERIAL_PERIOD_MS = 500u;  // 2 Hz

// ============================================================
//  Utility
// ============================================================
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float mapf(float x, float x0, float x1, float y0, float y1)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

// Print a float with explicit +/- sign prefix
static void pSgn(float v, int d = 2)
{
    if (v >= 0.0f) Serial.print('+');
    Serial.print(v, d);
}

// ============================================================
//  Tyre model — capped linear (Fiala-simplified)
//  Returns longitudinal force [N]:  + = forward (traction)
// ============================================================
static float tyreFx(float kappa, float Fz)
{
    float Fxmax = P_TYRE_MU * Fz;
    float Fx    = P_TYRE_CS * kappa;
    return clampf(Fx, -Fxmax, Fxmax);
}

// ============================================================
//  Engine torque curve — parabolic, throttle-scaled
// ============================================================
static float engineTorque(float thr, float rpm)
{
    float t;
    if (rpm < P_ENG_IDLE_RPM) {
        t = 0.0f;
    } else if (rpm <= P_ENG_PEAK_TQ_RPM) {
        t = 0.5f + 0.5f * (rpm - P_ENG_IDLE_RPM)
                        / (P_ENG_PEAK_TQ_RPM - P_ENG_IDLE_RPM);
    } else {
        t = 1.0f - 0.5f * (rpm - P_ENG_PEAK_TQ_RPM)
                        / (P_ENG_MAX_RPM - P_ENG_PEAK_TQ_RPM);
    }
    return thr * P_ENG_MAX_TORQUE_NM * clampf(t, 0.0f, 1.0f);
}

// ============================================================
//  Slip ratio — SAE convention, clamped to [-1, +1]
//  kappa = (v_wheel - v_vehicle) / max(|v_wheel|, |v_vehicle|, eps)
// ============================================================
static inline float calcSlip(float omega, float vx)
{
    float vw = omega * P_WHEEL_R_M;
    // Standstill: both wheel and vehicle stopped → no slip
    if (vx < 0.01f && vw < 0.01f) return 0.0f;
    // Spinning wheel from rest: pure wheel-spin
    if (vx < 0.01f) return 1.0f;
    // Normal running: SAE longitudinal slip (clamped)
    return clampf((vw - vx) / vx, -1.0f, 1.0f);
}

// ============================================================
//  Read joystick → throttle / brake / steering-wheel angle
// ============================================================
static void readJoystick()
{
    int rawAccel = analogRead(PIN_JOY_Y);   // Throttle/brake axis (physical Y → A1)
    int rawSteer = analogRead(PIN_JOY_X);   // Steering axis       (physical X → A0)

    // --- Throttle / Brake (A1) ---
    // Joystick pushed forward  (rawAccel > centre) → throttle
    // Joystick pulled backward (rawAccel < centre) → brake
    int dx = rawAccel - P_JOY_CENTER;
    if (abs(dx) <= P_JOY_DEADZONE) {
        gThrottle = gBrake = 0.0f;
    } else if (dx > 0) {
        gThrottle = clampf(
            mapf((float)(dx - P_JOY_DEADZONE), 0,
                 (float)(P_JOY_CENTER - P_JOY_DEADZONE), 0.0f, 1.0f),
            0.0f, 1.0f);
        gBrake = 0.0f;
    } else {
        gBrake = clampf(
            mapf((float)(-dx - P_JOY_DEADZONE), 0,
                 (float)(P_JOY_CENTER - P_JOY_DEADZONE), 0.0f, 1.0f),
            0.0f, 1.0f);
        gThrottle = 0.0f;
    }

    // --- Steering (A0) ---
    // Joystick right (rawSteer > centre) → positive (right) steering
    // Joystick left  (rawSteer < centre) → negative (left)  steering
    int dy = rawSteer - P_JOY_CENTER;
    if (abs(dy) <= P_JOY_DEADZONE) {
        gSteerSW = 0.0f;
    } else {
        float sgn = (dy > 0) ? 1.0f : -1.0f;
        float mag = (float)(abs(dy) - P_JOY_DEADZONE);
        gSteerSW  = sgn * clampf(
            mapf(mag, 0, (float)(P_JOY_CENTER - P_JOY_DEADZONE),
                 0.0f, P_STEER_MAX_DEG),
            0.0f, P_STEER_MAX_DEG);
    }
}

// ============================================================
//  Ackermann geometry — compute per-wheel steer angles
// ============================================================
static void updateAckermann()
{
    float roadDeg = gSteerSW / P_STEER_RATIO;          // Road-wheel angle [°]
    float roadRad = roadDeg * (float)M_PI / 180.0f;

    wRL.steerDeg = wRR.steerDeg = 0.0f;                // Rear non-steered

    if (fabsf(roadRad) < 1e-4f) {
        wFL.steerDeg = wFR.steerDeg = 0.0f;
        return;
    }

    float sgn    = (roadDeg > 0.0f) ? 1.0f : -1.0f;
    float absRad = fabsf(roadRad);
    float R      = P_WHEELBASE_M / tanf(absRad);       // Turn radius (rear axle)
    float th2    = P_TRACK_M * 0.5f;

    float inner = atanf(P_WHEELBASE_M / (R - th2)) * 180.0f / (float)M_PI;
    float outer = atanf(P_WHEELBASE_M / (R + th2)) * 180.0f / (float)M_PI;

    if (sgn > 0.0f) {   // Right turn — FR is inner wheel
        wFR.steerDeg =  inner;
        wFL.steerDeg =  outer;
    } else {            // Left turn  — FL is inner wheel
        wFL.steerDeg = -inner;
        wFR.steerDeg = -outer;
    }
}

// ============================================================
//  Longitudinal weight transfer → per-axle normal forces
// ============================================================
static void normalForces(float ax,
                         float &Nfl, float &Nfr,
                         float &Nrl, float &Nrr)
{
    const float g  = 9.81f;
    const float W  = P_MASS_KG * g;
    const float lr = P_WHEELBASE_M - P_CG_TO_FRONT_M;  // CG to rear axle
    const float dN = P_MASS_KG * ax * P_CG_HEIGHT_M / P_WHEELBASE_M;

    float Nf = max(0.0f,  W * lr / P_WHEELBASE_M - dN);
    float Nr = max(0.0f,  W * P_CG_TO_FRONT_M / P_WHEELBASE_M + dN);

    Nfl = Nfr = Nf * 0.5f;
    Nrl = Nrr = Nr * 0.5f;
}

// ============================================================
//  Wheel dynamics helper — semi-implicit Euler for one wheel
//
//  Explicit Euler for wheel omega is unstable at low speed:
//    stability requires dt < 2·J·vx / (Cs·R²)
//    e.g. vx=2 m/s, J=1.5, Cs=80000, R=0.315 → dt < 0.38 ms
//  Arduino loop dt ≈ 1–2 ms → explicit Euler causes oscillation.
//
//  Semi-implicit fix: treat the slip-stiffness term implicitly.
//  Linear region:  J·dω/dt = A – B·ω
//    A = driveTq + Cs·R – brakeTq – Frr·R   (constant part)
//    B = Cs·R² / vx                          (stiff coupling)
//  Implicit update (unconditionally stable):
//    ω_new = (J·ω_old + dt·A) / (J + dt·B)
//
//  Saturated region (|Fx_linear| ≥ μ·Fz):
//    force is constant → explicit Euler is exact.
// ============================================================
static void integrateWheel(WheelState &w,
                           float driveTq, float brakeTq,
                           float Fz, float vx, float dt)
{
    w.slip      = calcSlip(w.omega, vx);
    float Frr   = P_TYRE_RR  * Fz;                // Rolling-resistance force [N]
    float FxMax = P_TYRE_MU  * Fz;                // Friction limit [N]
    float FxLin = P_TYRE_CS  * w.slip;            // Unclamped linear tyre force [N]

    if (vx > 0.01f && fabsf(FxLin) < FxMax) {
        // ── Linear tyre: semi-implicit Euler (unconditionally stable) ──
        float A    = driveTq + P_TYRE_CS * P_WHEEL_R_M
                              - brakeTq  - Frr * P_WHEEL_R_M;
        float B    = P_TYRE_CS * P_WHEEL_R_M * P_WHEEL_R_M / vx;
        w.omega    = (P_WHEEL_J_KGM2 * w.omega + dt * A)
                   / (P_WHEEL_J_KGM2 + dt * B);
    } else {
        // ── Saturated tyre or standstill: explicit Euler (exact for const force) ──
        float Fx   = clampf(FxLin, -FxMax, FxMax);
        float tq   = driveTq - Fx * P_WHEEL_R_M - brakeTq - Frr * P_WHEEL_R_M;
        w.omega   += (tq / P_WHEEL_J_KGM2) * dt;
    }

    w.omega     = max(0.0f, w.omega);          // no reverse spinning
    w.slip      = calcSlip(w.omega, vx);       // refresh after update
    w.speed_kmh = w.omega * P_WHEEL_R_M * 3.6f;
}

// ============================================================
//  Main dynamics update (variable-step Euler)
// ============================================================
static void updateDynamics(float dt)
{
    if (dt < 1e-6f) return;
    dt = clampf(dt, 0.0f, DT_MAX);

    // ---- Ackermann steering (FIRST: all subsequent calcs use current steer) ----
    updateAckermann();

    float vx = max(0.0f, gSpeed_ms);

    // ---- Engine RPM (from driven rear wheels) ----
    float rearOmega = (wRL.omega + wRR.omega) * 0.5f;
    gEngRPM = clampf(
        rearOmega * P_GEAR_RATIO * P_FINAL_DRIVE_RATIO
                  * 60.0f / (2.0f * (float)M_PI),
        P_ENG_IDLE_RPM, P_ENG_MAX_RPM);

    // ---- Engine torque ----
    gEngTq_Nm = engineTorque(gThrottle, gEngRPM);

    // Drive torque distributed equally to both rear wheels (open diff)
    float driveTqRear = gEngTq_Nm
                      * P_GEAR_RATIO * P_FINAL_DRIVE_RATIO
                      * P_DRIVETRAIN_EFF / 2.0f;
    gDrvTqRear = driveTqRear;

    // Brake torque per wheel
    float brkFront = gBrake * P_BRAKE_MAX_N * P_BRAKE_FRONT_BIAS
                            * P_WHEEL_R_M / 2.0f;
    float brkRear  = gBrake * P_BRAKE_MAX_N * (1.0f - P_BRAKE_FRONT_BIAS)
                            * P_WHEEL_R_M / 2.0f;
    gBrkTqF = brkFront;  gBrkTqR = brkRear;

    // ---- Normal forces ----
    float Nfl, Nfr, Nrl, Nrr;
    normalForces(gAccel_ms2, Nfl, Nfr, Nrl, Nrr);
    gNfl=Nfl;  gNfr=Nfr;  gNrl=Nrl;  gNrr=Nrr;

    // ---- Wheel dynamics ----
    integrateWheel(wFL, 0.0f,       brkFront, Nfl, vx, dt); // front, non-driven
    integrateWheel(wFR, 0.0f,       brkFront, Nfr, vx, dt);
    integrateWheel(wRL, driveTqRear, brkRear,  Nrl, vx, dt); // rear, driven (RWD)
    integrateWheel(wRR, driveTqRear, brkRear,  Nrr, vx, dt);

    // ---- Vehicle longitudinal acceleration ----
    // Front tyre forces projected along vehicle axis via steer angle
    float cosFL = cosf(wFL.steerDeg * (float)M_PI / 180.0f);
    float cosFR = cosf(wFR.steerDeg * (float)M_PI / 180.0f);

    gFxFL = tyreFx(wFL.slip, Nfl);
    gFxFR = tyreFx(wFR.slip, Nfr);
    gFxRL = tyreFx(wRL.slip, Nrl);
    gFxRR = tyreFx(wRR.slip, Nrr);
    float FxSum = gFxFL * cosFL + gFxFR * cosFR + gFxRL + gFxRR;
    gFxSum_d = FxSum;

    float Fdrag = 0.5f * P_AIR_DENSITY * P_AERO_CD * P_FRONTAL_AREA_M2
                       * vx * vx;
    gFdrag = Fdrag;

    // Rolling resistance applied directly to vehicle body
    float Froll = (vx > 0.01f) ? (P_TYRE_RR * P_MASS_KG * 9.81f) : 0.0f;
    gFrollB = Froll;

    float Fnet = FxSum - Fdrag - Froll;
    if (Fnet < 0.0f && vx < 0.01f) Fnet = 0.0f;  // no backward creep
    gFnet = Fnet;

    gAccel_ms2 = Fnet / P_MASS_KG;
    gSpeed_ms  = max(0.0f, gSpeed_ms + gAccel_ms2 * dt);

    // Hard stop: snap to zero when nearly stationary and no throttle
    if (gSpeed_ms < 0.5f && gThrottle < 0.05f) {
        gSpeed_ms    = 0.0f;
        gVy_ms       = 0.0f;
        gYawRate_rads= 0.0f;
        gLatAccel_ms2= 0.0f;
        gSideSlip_deg= 0.0f;
        wFL.omega = wFR.omega = wRL.omega = wRR.omega = 0.0f;
        gAccel_ms2 = 0.0f;
    }

    // ---- Lateral dynamics: bicycle model (2-DOF) ----
    // vxLat is computed AFTER hard-stop so it reflects the true current speed.
    // Threshold 0.2 m/s (0.72 km/h) — lower than hard-stop to avoid edge cases.
    float vxLat = max(0.0f, gSpeed_ms);
    gVxLat = vxLat;

    if (vxLat > 0.2f) {
        const float lr_m = P_WHEELBASE_M - P_CG_TO_FRONT_M;

        // Average front road-wheel steer angle [rad] — sign follows gSteerSW
        float delta_rad = (wFL.steerDeg + wFR.steerDeg) * 0.5f
                          * (float)M_PI / 180.0f;
        gDeltaRad = delta_rad;

        // Tyre slip angles [rad]
        float alphaF = delta_rad
                       - (gVy_ms + P_CG_TO_FRONT_M * gYawRate_rads) / vxLat;
        float alphaR = -(gVy_ms - lr_m * gYawRate_rads) / vxLat;
        gAlphaF = alphaF;  gAlphaR = alphaR;

        // Lateral forces, clamped by friction limit
        float FyfMax = P_TYRE_MU * (Nfl + Nfr);
        float FyrMax = P_TYRE_MU * (Nrl + Nrr);
        float Fyf = clampf(P_TYRE_CF * alphaF, -FyfMax, FyfMax);
        float Fyr = clampf(P_TYRE_CR * alphaR, -FyrMax, FyrMax);
        gFyf = Fyf;  gFyr = Fyr;

        // Equations of motion (vehicle body frame)
        float vy_dot = (Fyf + Fyr) / P_MASS_KG - gYawRate_rads * vxLat;
        float r_dot  = (P_CG_TO_FRONT_M * Fyf - lr_m * Fyr)
                       / P_VEHICLE_IZ_KGM2;
        gVyDot = vy_dot;  gRDot = r_dot;

        // Euler integration
        gVy_ms        += vy_dot * dt;
        gYawRate_rads += r_dot  * dt;

        // Lateral acceleration (felt by occupants) [m/s²]
        gLatAccel_ms2  = (Fyf + Fyr) / P_MASS_KG;

        // Side-slip angle β = atan(vy / vx) [°]
        gSideSlip_deg  = atan2f(gVy_ms, vxLat) * 180.0f / (float)M_PI;
    } else {
        // Below threshold: decay lateral states to zero
        gVy_ms        *= 0.90f;
        gYawRate_rads *= 0.90f;
        gLatAccel_ms2  = 0.0f;
        gSideSlip_deg  = 0.0f;
        gDeltaRad = 0.0f;
        gAlphaF = gAlphaR = gFyf = gFyr = gVyDot = gRDot = 0.0f;
    }

}

// ============================================================
//  CAN helpers
// ============================================================

// Send one wheel frame: [SPD×100][SLP×1000][STEER×10][0][0]
static void sendWheelFrame(uint32_t id, const WheelState &w)
{
    byte buf[8];
    int16_t spd   = (int16_t)(w.speed_kmh  * 100.0f);
    int16_t slip  = (int16_t)(w.slip       * 1000.0f);
    int16_t steer = (int16_t)(w.steerDeg   *  10.0f);
    buf[0] = (byte)(spd   >> 8);  buf[1] = (byte)(spd   & 0xFF);
    buf[2] = (byte)(slip  >> 8);  buf[3] = (byte)(slip  & 0xFF);
    buf[4] = (byte)(steer >> 8);  buf[5] = (byte)(steer & 0xFF);
    buf[6] = 0;                   buf[7] = 0;
    CAN0.sendMsgBuf(id, 0, 8, buf);
}

static void sendCANFrames()
{
    byte buf[8];

    // 0x100 — Engine: [RPM uint16][TRQ int16 ×10][THR uint8][BRK uint8][0][0]
    {
        uint16_t rpm_u = (uint16_t)clampf(gEngRPM, 0, 65535.0f);
        int16_t  trq_i = (int16_t)(gEngTq_Nm * 10.0f);
        uint8_t  thr_u = (uint8_t)(gThrottle  * 255.0f);
        uint8_t  brk_u = (uint8_t)(gBrake     * 255.0f);
        buf[0] = (byte)(rpm_u >> 8);  buf[1] = (byte)(rpm_u & 0xFF);
        buf[2] = (byte)(trq_i >> 8);  buf[3] = (byte)(trq_i & 0xFF);
        buf[4] = thr_u;               buf[5] = brk_u;
        buf[6] = 0;                   buf[7] = 0;
        CAN0.sendMsgBuf(CAN_ID_ENGINE, 0, 8, buf);
    }

    // 0x101–0x104 — Wheel speeds, slip, steer angles
    sendWheelFrame(CAN_ID_WHL_FL, wFL);
    sendWheelFrame(CAN_ID_WHL_FR, wFR);
    sendWheelFrame(CAN_ID_WHL_RL, wRL);
    sendWheelFrame(CAN_ID_WHL_RR, wRR);

    // 0x200 — Steering: [SW ×10][RW ×100][0×4]
    {
        int16_t sw = (int16_t)(gSteerSW * 10.0f);
        int16_t rw = (int16_t)((gSteerSW / P_STEER_RATIO) * 100.0f);
        buf[0] = (byte)(sw >> 8);  buf[1] = (byte)(sw & 0xFF);
        buf[2] = (byte)(rw >> 8);  buf[3] = (byte)(rw & 0xFF);
        buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0;
        CAN0.sendMsgBuf(CAN_ID_STEER, 0, 8, buf);
    }

    // 0x201 — Lateral: [ay×100 int16][yaw_rate×1000 int16][beta×100 int16][0×2]
    {
        int16_t ay   = (int16_t)(gLatAccel_ms2  * 100.0f);
        int16_t yr   = (int16_t)(gYawRate_rads  * 1000.0f);
        int16_t beta = (int16_t)(gSideSlip_deg  * 100.0f);
        buf[0] = (byte)(ay   >> 8);  buf[1] = (byte)(ay   & 0xFF);
        buf[2] = (byte)(yr   >> 8);  buf[3] = (byte)(yr   & 0xFF);
        buf[4] = (byte)(beta >> 8);  buf[5] = (byte)(beta & 0xFF);
        buf[6] = 0; buf[7] = 0;
        CAN0.sendMsgBuf(CAN_ID_LATERAL, 0, 8, buf);
    }

    // 0x300 — Vehicle: [km/h ×100][m/s² ×100][0×4]
    {
        int16_t spd = (int16_t)(gSpeed_ms * 3.6f * 100.0f);
        int16_t acc = (int16_t)(gAccel_ms2 * 100.0f);
        buf[0] = (byte)(spd >> 8);  buf[1] = (byte)(spd & 0xFF);
        buf[2] = (byte)(acc >> 8);  buf[3] = (byte)(acc & 0xFF);
        buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0;
        CAN0.sendMsgBuf(CAN_ID_VEHICLE, 0, 8, buf);
    }
}

// ============================================================
//  Setup
// ============================================================
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_JOY_SW, INPUT_PULLUP);
    pinMode(PIN_SW1,    INPUT_PULLUP);
    pinMode(PIN_SW2,    INPUT_PULLUP);
    pinMode(PIN_SW3,    INPUT_PULLUP);
    pinMode(PIN_SW4,    INPUT_PULLUP);
    pinMode(PIN_SW5,    INPUT_PULLUP);

    // Initialise MCP2515
    while (CAN0.begin(MCP_ANY, CAN_BAUD, CAN_OSC_FREQ) != CAN_OK) {
        Serial.println(F("CAN init failed — retrying in 500 ms"));
        delay(500);
    }
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("CAN ready @ 500 kbps"));
    Serial.println(F("Dashboard: use screen/minicom/picocom for ANSI in-place display"));

    // Zero-initialise all wheel states
    WheelState zs = {0.0f, 0.0f, 0.0f, 0.0f};
    wFL = wFR = wRL = wRR = zs;

    gLastDynUs = micros();
    gLastCanMs = millis();
}

// ============================================================
//  Loop
// ============================================================
void loop()
{
    // --- Dynamics at free-running rate (dt from micros) ---
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - gLastDynUs) * 1e-6f;
    gLastDynUs = nowUs;

    readJoystick();
    updateDynamics(dt);

    // --- CAN TX at fixed period (100 Hz) ---
    unsigned long nowMs = millis();
    if (nowMs - gLastCanMs >= CAN_TX_PERIOD_MS) {
        gLastCanMs = nowMs;
        sendCANFrames();
    }

    // --- Serial dashboard at 2 Hz (ANSI fixed-position, use screen/minicom) ---
    if (nowMs - gLastSerialMs >= SERIAL_PERIOD_MS) {
        gLastSerialMs = nowMs;
        static bool firstPrint = true;
        if (firstPrint) { Serial.print(F("\033[2J")); firstPrint = false; }
        Serial.print(F("\033[H"));   // cursor to top-left

        float speedKmh = gSpeed_ms * 3.6f;
        float roadDeg  = gSteerSW / P_STEER_RATIO;

        // ── line 1: header ───────────────────────────────────────────
        Serial.print(F("=== VehicleDynamicsCAN ===  T:"));
        Serial.print(millis() / 1000.0f, 1);
        Serial.println(F("s\033[K"));

        // ── line 2: driver inputs ────────────────────────────────────
        Serial.print(F("[IN ] Thr:")); pSgn(gThrottle, 2);
        Serial.print(F("  Brk:"));    pSgn(gBrake,    2);
        Serial.print(F("  SW:"));     pSgn(gSteerSW,  1);
        Serial.println(F("deg\033[K"));

        // ── line 3: Ackermann steering geometry ──────────────────────
        Serial.print(F("[STR] Road:")); pSgn(roadDeg,      2);
        Serial.print(F("deg  FL:"));   pSgn(wFL.steerDeg, 2);
        Serial.print(F("  FR:"));      pSgn(wFR.steerDeg, 2);
        Serial.print(F("  RL:"));      pSgn(wRL.steerDeg, 2);
        Serial.print(F("  RR:"));      pSgn(wRR.steerDeg, 2);
        Serial.println(F(" [deg]\033[K"));

        // ── line 4: engine / drivetrain ──────────────────────────────
        Serial.print(F("[ENG] RPM:")); Serial.print((int)gEngRPM);
        Serial.print(F("  Tq:"));      pSgn(gEngTq_Nm,  1);
        Serial.print(F("Nm  DrvTq/whl:")); pSgn(gDrvTqRear, 1);
        Serial.println(F("Nm\033[K"));

        // ── line 5: brake torques ────────────────────────────────────
        Serial.print(F("      BrkTq  F:")); pSgn(gBrkTqF, 1);
        Serial.print(F("Nm  R:"));          pSgn(gBrkTqR, 1);
        Serial.println(F("Nm /whl\033[K"));

        // ── line 6: normal forces ────────────────────────────────────
        Serial.print(F("[Nz ] FL:")); Serial.print(gNfl, 0);
        Serial.print(F("  FR:"));    Serial.print(gNfr, 0);
        Serial.print(F("  RL:"));    Serial.print(gNrl, 0);
        Serial.print(F("  RR:"));    Serial.print(gNrr, 0);
        Serial.println(F(" [N]\033[K"));

        // ── line 7: wheel table header ───────────────────────────────
        Serial.println(F("[WHL]   omega[r/s]  slip[-]    Fx[N]  spd[km/h]\033[K"));

        // ── lines 8-11: per-wheel state ──────────────────────────────
        Serial.print(F("  FL: ")); pSgn(wFL.omega,3); Serial.print(F("  ")); pSgn(wFL.slip,4); Serial.print(F("  ")); pSgn(gFxFL,1); Serial.print(F("  ")); pSgn(wFL.speed_kmh,1); Serial.println(F("\033[K"));
        Serial.print(F("  FR: ")); pSgn(wFR.omega,3); Serial.print(F("  ")); pSgn(wFR.slip,4); Serial.print(F("  ")); pSgn(gFxFR,1); Serial.print(F("  ")); pSgn(wFR.speed_kmh,1); Serial.println(F("\033[K"));
        Serial.print(F("  RL: ")); pSgn(wRL.omega,3); Serial.print(F("  ")); pSgn(wRL.slip,4); Serial.print(F("  ")); pSgn(gFxRL,1); Serial.print(F("  ")); pSgn(wRL.speed_kmh,1); Serial.println(F("\033[K"));
        Serial.print(F("  RR: ")); pSgn(wRR.omega,3); Serial.print(F("  ")); pSgn(wRR.slip,4); Serial.print(F("  ")); pSgn(gFxRR,1); Serial.print(F("  ")); pSgn(wRR.speed_kmh,1); Serial.println(F("\033[K"));

        // ── line 12: longitudinal force balance ──────────────────────
        Serial.print(F("[LON] FxSum:")); pSgn(gFxSum_d,  1);
        Serial.print(F("N  Fdrag:"));   pSgn(-gFdrag,   1);
        Serial.print(F("N  Froll:"));   pSgn(-gFrollB,  1);
        Serial.print(F("N  Fnet:"));    pSgn(gFnet,     1);
        Serial.println(F("N\033[K"));

        // ── line 13: vehicle longitudinal state ──────────────────────
        Serial.print(F("[VEH] Accel:")); pSgn(gAccel_ms2, 3);
        Serial.print(F("m/s2  Speed:")); pSgn(speedKmh,   2);
        Serial.print(F("km/h ("));       pSgn(gSpeed_ms,  3);
        Serial.println(F("m/s)\033[K"));

        // ── line 14: lateral model inputs ────────────────────────────
        Serial.print(F("[LAT] vxLat:")); pSgn(gVxLat,   3);
        Serial.print(F("m/s  delta:")); pSgn(gDeltaRad, 4);
        Serial.print(F("rad ("));        pSgn(gDeltaRad * 180.0f / (float)M_PI, 2);
        Serial.println(F("deg)\033[K"));

        // ── line 15: lateral tyre slip angles & forces ───────────────
        Serial.print(F("[LAT] aF:"));  pSgn(gAlphaF, 4);
        Serial.print(F("rad  aR:"));   pSgn(gAlphaR, 4);
        Serial.print(F("rad  Fyf:"));  pSgn(gFyf,    1);
        Serial.print(F("N  Fyr:"));    pSgn(gFyr,    1);
        Serial.println(F("N\033[K"));

        // ── line 16: lateral derivatives & lateral velocity ──────────
        Serial.print(F("[LAT] vy_dot:")); pSgn(gVyDot,  3);
        Serial.print(F("m/s2  r_dot:"));  pSgn(gRDot,   4);
        Serial.print(F("r/s2  Vy:"));     pSgn(gVy_ms,  3);
        Serial.println(F("m/s\033[K"));

        // ── line 17: lateral outputs ─────────────────────────────────
        Serial.print(F("[OUT] ay:"));   pSgn(gLatAccel_ms2, 3);
        Serial.print(F("m/s2  yr:"));   pSgn(gYawRate_rads, 4);
        Serial.print(F("r/s  beta:"));  pSgn(gSideSlip_deg, 3);
        Serial.println(F("deg\033[K"));
    }
}
