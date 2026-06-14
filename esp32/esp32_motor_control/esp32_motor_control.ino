#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <esp_camera.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#include <ArduinoWebsockets.h>
#include <lwip/sockets.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

using namespace websockets;

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const bool USE_STATIC_IP = false;  // 手机热点直连建议 false；固定路由器 IP 时改 true。
IPAddress localIp(192, 168, 152, 11);
IPAddress gateway(192, 168, 152, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(192, 168, 152, 1);

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int PULSES_PER_REV = 11;
const unsigned long RPM_INTERVAL_MS = 250;
const bool ENABLE_DEBUG_LOG = false;

const uint32_t USB_BAUD = 115200;
const char* PYTHON_SERVER_IP = "192.168.152.216";   // 运行 all_in_one_webui.py 的电脑 IP
const uint16_t IMU_HTTP_PORT = 8000;             // all_in_one_webui.py 的 HTTP 端口，用于上报姿态
const char* IMU_HTTP_PATH = "/imu";              // 姿态 JSON 上报路径
// BNO080/BNO085 走 I2C：SDA=D6/GPIO43，SCL=D7/GPIO44（复用原 UART IMU 的两根线）。
const int I2C_SDA_PIN = 43;
const int I2C_SCL_PIN = 44;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint16_t IMU_REPORT_INTERVAL_MS = 20;      // BNO080 报告周期
const uint16_t IMU_POST_INTERVAL_MS = 50;        // 向电脑 WebUI 上报姿态的周期

const char* CAMERA_WS_HOST = PYTHON_SERVER_IP;
const uint16_t CAMERA_WS_PORT = 8081;
const char* CAMERA_WS_PATH = "/ws/camera";
const uint32_t SERVER_RECONNECT_MS = 500;
const uint32_t CAMERA_WS_RECONNECT_MS = 3000;
const uint32_t WIFI_CHECK_MS = 2000;
const uint32_t TCP_KEEPALIVE_IDLE_SEC = 5;
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_QVGA;
const int CAMERA_JPEG_QUALITY = 17;
const int CAMERA_FB_COUNT = 2;
volatile int cameraTargetFps = 8;

// Seeed Studio XIAO ESP32S3 Sense OV2640 camera pins.
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

struct MotorState {
  int id;
  int encoderA;
  int encoderB;
  int in1;
  int in2;
  int pwmChannel1;
  int pwmChannel2;
  volatile long encoderCount;
  long lastEncoderCount;
  unsigned long lastRpmMillis;
  float rpm;
  int commandPercent; // -100 到 100：负数反转，正数正转。
  String direction;
};

// 电机1：编码器 D0/D1，驱动 D2/D3。
MotorState motor1 = {1, 1, 2, 3, 4, 0, 1, 0, 0, 0, 0.0, 0, "stop"};

// 电机2：编码器 D4/D5，驱动 D9/D10。
MotorState motor2 = {2, 5, 6, 8, 9, 2, 3, 0, 0, 0, 0.0, 0, "stop"};

WebServer server(80);
WebServer cameraServer(81);
BNO080 imuSensor;
WebsocketsClient cameraWsClient;

typedef camera_fb_t* CameraFramePtr;
QueueHandle_t cameraFrameQueue = nullptr;
SemaphoreHandle_t cameraWsMutex = nullptr;

uint32_t lastTcpAttemptMs = 0;
uint32_t lastWifiCheckMs = 0;
volatile bool imuReady = false;
uint8_t imuI2cAddress = 0;
uint32_t imuSampleCount = 0;
uint32_t imuPostCount = 0;
uint32_t lastCameraRetryMs = 0;
volatile bool cameraReady = false;
volatile bool cameraWsReady = false;
volatile bool cameraStreamEnabled = true;
volatile unsigned long cameraCapturedCount = 0;
volatile unsigned long cameraSentCount = 0;
volatile unsigned long cameraDroppedCount = 0;
volatile int driveThrottle = 0;
volatile int driveSteering = 0;
volatile bool imuEulerValid = false;
volatile float imuRollDeg = 0.0;
volatile float imuPitchDeg = 0.0;
volatile float imuYawDeg = 0.0;
// IMU 航向角速度，由欧拉角数值微分得到，单位 度/秒。
volatile float imuYawRate = 0.0;
volatile bool imuGyroValid = false;
volatile float imuGyroX = 0.0;
volatile float imuGyroY = 0.0;
volatile float imuGyroZ = 0.0;
volatile float imuQuatI = 0.0, imuQuatJ = 0.0, imuQuatK = 0.0, imuQuatReal = 1.0;
volatile float imuAccelX = 0.0, imuAccelY = 0.0, imuAccelZ = 0.0;
volatile float imuMagX = 0.0, imuMagY = 0.0, imuMagZ = 0.0;
volatile uint8_t imuQuatAccuracy = 0, imuMagAccuracy = 0;

// ---------- 运动控制环状态 ----------
// HTTP 指令目标（-100..100）。pwm 模式下逐电机目标存在 motorN.commandPercent 之外的下面两组里。
volatile int cmdThrottle = 0;      // /drive 油门目标
volatile int cmdSteering = 0;      // /drive 转向目标
volatile float outThrottle = 0.0;  // 斜坡后的实际油门输出
volatile float outSteering = 0.0;  // 斜坡后的实际转向输出
volatile uint32_t lastCmdMs = 0;   // 最近一次有效指令时间戳（看门狗用）

// 运行模式：0=停止, 1=差速驾驶(/drive，带辅助), 2=手动逐电机(/pwm，只走斜坡+看门狗)
volatile int controlMode = 0;
// 手动模式下两电机目标百分比（带斜坡）。
volatile int manualTarget1 = 0;
volatile int manualTarget2 = 0;
volatile float manualOut1 = 0.0;
volatile float manualOut2 = 0.0;

// 辅助开关与目标航向。
volatile bool headingHoldOn = true;   // 航向锁定/直行纠偏
volatile bool headingValid = false;   // 目标航向是否已建立
volatile float headingSetpoint = 0.0; // 目标航向(yaw, 度)
volatile float lastSteerCorr = 0.0;   // 最近一次航向纠偏量（调试显示）
volatile bool driveVectorValid = false;     // 摇杆方向角是否已建立
volatile float driveVectorAngleDeg = 0.0;   // 最近一次作为基准的摇杆方向角（速度大小变化不更新）
volatile int assistMode = 1;                // IMU 辅助模式：1=相对模式，2=绝对方向模式
volatile bool absForwardValid = false;      // 绝对模式的“前方”是否已设定
volatile float absForwardYaw = 0.0;         // 绝对模式的前方 yaw
volatile float absTargetYaw = 0.0;          // 绝对模式当前目标 yaw
volatile float absYawErr = 0.0;             // 绝对模式当前航向误差

// 可调参数（可经 /config 在线修改）。
volatile float RAMP_UP = 450.0;       // 加速斜率：百分比/秒（0→100 约 0.22s，跟手又不急冲）
volatile float RAMP_DOWN = 600.0;     // 减速/归零斜率：百分比/秒
volatile float HEAD_KP = 1.6;         // 航向比例增益（%/度）
volatile float HEAD_KD = 0.06;        // 航向角速度阻尼（%/(度/秒)）
volatile int STEER_DEADZONE = 6;      // 转向死区，判定是否在直行
volatile int HEAD_CORR_LIMIT = 35;    // 航向纠偏输出上限（百分比）
volatile int HEAD_SIGN = 1;           // 航向纠偏方向：普通模式左右纠偏方向
volatile float STEER_GAIN = 0.45;     // 转向灵敏度：移动中按曲率缩放内侧轮，避免原地打转
volatile uint32_t CMD_TIMEOUT_MS = 350; // 看门狗超时：超时未收到指令则归零滑停
volatile float ABS_HEAD_KP = 6;      // 绝对模式航向比例增益
volatile float ABS_HEAD_KD = 0.8;     // 绝对模式航向角速度阻尼
volatile int ABS_TURN_SIGN = -1;       // 绝对模式原地旋转校正方向，实测反了就镜像
volatile int ABS_PIVOT_LIMIT = 85;     // 绝对模式原地校正最大输出
volatile float ABS_PIVOT_START_DEG = 10.0; // 绝对模式：航向误差小于该角度即放行满速前进（基本已对准）
volatile float ABS_PIVOT_FULL_DEG = 45.0;  // 绝对模式：航向误差大于该角度则纯原地旋转、暂不前进

const uint32_t MOTION_TICK_MS = 5;    // 控制环周期（约 200Hz）
const float DRIVE_VECTOR_DEADZONE = 8.0f;      // 摇杆中心死区，松手/接近中心会释放航向基准
const float DRIVE_VECTOR_RESET_DEG = 12.0f;    // 摇杆方向角变化超过该阈值才重建航向基准
const float MOVING_THROTTLE_DEADZONE = 5.0f;   // 判断是否为前进/后退曲率转向
const float PIVOT_BLEND_START_DEG = 80.0f;      // 相对正前/正后的偏角超过 80° 后才开始混到“原地转”
const float PIVOT_BLEND_END_DEG = 90.0f;        // 纯左/纯右时完全进入原地转
const float PIVOT_GAIN = 0.75f;                 // 原地转最大力度，独立于移动转向灵敏度

const uint32_t DIRECT_MAGIC = 0x52444345UL; // "ECDR" little-endian: ESP32 car direct remote
const uint8_t DIRECT_VERSION = 1;
const uint8_t DIRECT_TYPE_HELLO = 1;
const uint8_t DIRECT_TYPE_DRIVE = 2;
const uint8_t DIRECT_TYPE_STOP = 3;
const uint8_t DIRECT_TYPE_CONFIG = 4;
const uint8_t DIRECT_TYPE_ACK = 5;
const uint8_t DIRECT_FLAG_HEADING_HOLD = 0x01;
const uint8_t DIRECT_FLAG_SET_ABS_FORWARD = 0x02;

struct DirectControlPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  int16_t throttle;
  int16_t steering;
  uint8_t assistMode;
  uint8_t flags;
  uint32_t ms;
} __attribute__((packed));

portMUX_TYPE directMux = portMUX_INITIALIZER_UNLOCKED;
DirectControlPacket directPendingPacket;
uint8_t directPendingMac[6] = {0};
volatile bool directPacketPending = false;
volatile bool directReady = false;
volatile uint32_t directRxCount = 0;
volatile uint32_t directBadCount = 0;
volatile uint32_t directAckCount = 0;
volatile uint32_t directLastRxMs = 0;
volatile uint16_t directLastSeq = 0;
volatile uint8_t directLastType = 0;
uint8_t directLastPeer[6] = {0};

void processDirectControlPacket();

void IRAM_ATTR handleEncoder1A() {
  bool a = digitalRead(motor1.encoderA);
  bool b = digitalRead(motor1.encoderB);
  motor1.encoderCount += (a == b) ? 1 : -1;
}

void IRAM_ATTR handleEncoder2A() {
  bool a = digitalRead(motor2.encoderA);
  bool b = digitalRead(motor2.encoderB);
  motor2.encoderCount += (a == b) ? 1 : -1;
}

MotorState* getMotorFromArg() {
  int motorId = server.hasArg("motor") ? server.arg("motor").toInt() : 1;
  return motorId == 2 ? &motor2 : &motor1;
}

void writePwm(MotorState& motor, int in1Duty, int in2Duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motor.in1, constrain(in1Duty, 0, PWM_MAX));
  ledcWrite(motor.in2, constrain(in2Duty, 0, PWM_MAX));
#else
  ledcWrite(motor.pwmChannel1, constrain(in1Duty, 0, PWM_MAX));
  ledcWrite(motor.pwmChannel2, constrain(in2Duty, 0, PWM_MAX));
#endif
}

void setMotorPercent(MotorState& motor, int percent) {
  percent = constrain(percent, -100, 100);
  motor.commandPercent = percent;
  int duty = map(abs(percent), 0, 100, 0, PWM_MAX);

  if (percent > 0) {
    // DRV8871 常见控制：IN1 PWM，IN2 LOW，正转。
    writePwm(motor, duty, 0);
    motor.direction = "forward";
  } else if (percent < 0) {
    // 反转：IN1 LOW，IN2 PWM。
    writePwm(motor, 0, duty);
    motor.direction = "reverse";
  } else {
    // 停止/滑行：IN1 LOW，IN2 LOW。
    writePwm(motor, 0, 0);
    motor.direction = "stop";
  }
}

void setDifferentialDrive(int throttle, int steering) {
  throttle = constrain(throttle, -100, 100);
  steering = constrain(steering, -100, 100);
  driveThrottle = throttle;
  driveSteering = steering;

  int leftPercent = constrain(throttle + steering, -100, 100);
  int rightPercent = constrain(throttle - steering, -100, 100);
  setMotorPercent(motor1, leftPercent);
  setMotorPercent(motor2, rightPercent);
}

static inline float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// 选择当前应该用的斜率：朝更大幅度（且同向）= 加速，否则（减速/归零/换向）= 减速。
static inline float rampStep(float current, float target, float up, float down) {
  bool sameSign = (target >= 0.0f) == (current >= 0.0f);
  return (sameSign && fabsf(target) >= fabsf(current)) ? up : down;
}

static inline float rampToward(float current, float target, float maxStep) {
  float diff = target - current;
  if (diff > maxStep) return current + maxStep;
  if (diff < -maxStep) return current - maxStep;
  return target;
}

static inline float smoothStep01(float x) {
  x = constrain(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

static inline float pivotBlendForVector(float throttle, float steering) {
  float absThrottle = fabsf(throttle);
  float absSteering = fabsf(steering);
  if (absThrottle + absSteering <= 1.0f) {
    return 0.0f;
  }
  float sideAngleDeg = atan2f(absSteering, absThrottle) * 180.0f / PI;
  return smoothStep01((sideAngleDeg - PIVOT_BLEND_START_DEG) / (PIVOT_BLEND_END_DEG - PIVOT_BLEND_START_DEG));
}

static inline void joystickToWheelMix(float throttle, float steering, float& left, float& right) {
  // 主速度使用摇杆半径：同一半径速度相同；角度只决定差速/转弯形态。
  float vectorSpeed = constrain(hypotf(throttle, steering), 0.0f, 100.0f);
  float steerSign = steering >= 0.0f ? 1.0f : -1.0f;
  float pivotLeft = steerSign * vectorSpeed * PIVOT_GAIN;
  float pivotRight = -steerSign * vectorSpeed * PIVOT_GAIN;
  float pivotBlend = pivotBlendForVector(throttle, steering);
  float curveLeft = 0.0f;
  float curveRight = 0.0f;

  if (fabsf(throttle) > MOVING_THROTTLE_DEADZONE) {
    float throttleSign = throttle >= 0.0f ? 1.0f : -1.0f;
    float outer = vectorSpeed;
    float innerScale = 1.0f - constrain(fabsf(steering) * STEER_GAIN / 100.0f, 0.0f, 0.85f);
    float inner = outer * innerScale;
    if (steering > STEER_DEADZONE) {
      curveLeft = throttleSign * outer;
      curveRight = throttleSign * inner;
    } else if (steering < -STEER_DEADZONE) {
      curveLeft = throttleSign * inner;
      curveRight = throttleSign * outer;
    } else {
      curveLeft = throttle;
      curveRight = throttle;
    }
  } else {
    curveLeft = pivotLeft;
    curveRight = pivotRight;
    pivotBlend = 1.0f;
  }

  left = curveLeft * (1.0f - pivotBlend) + pivotLeft * pivotBlend;
  right = curveRight * (1.0f - pivotBlend) + pivotRight * pivotBlend;
}

// 高频运动控制环：看门狗归零、加减速斜坡、IMU 航向闭环纠偏。
void taskMotionControl(void*) {
  const float dt = MOTION_TICK_MS / 1000.0f;
  float prevYaw = 0.0f;
  bool havePrev = false;
  float yawRateF = 0.0f;
  bool absReverseDrive = false;   // 绝对模式：当前用车尾对准目标方向(倒车)，带迟滞防 90° 抖动

  for (;;) {
    uint32_t now = millis();

    // ---- 看门狗：超时未收到指令 → 油门归零滑停（航向保持仍可继续）----
    if (now - lastCmdMs > CMD_TIMEOUT_MS) {
      cmdThrottle = 0;
      cmdSteering = 0;
      manualTarget1 = 0;
      manualTarget2 = 0;
    }

    // ---- 航向角速度估计：对 yaw 做数值微分 + EMA 平滑，供航向 D 项使用 ----
    float curYaw = imuYawDeg;
    if (havePrev && imuEulerValid) {
      float yawRateRaw = wrap180(curYaw - prevYaw) / dt;
      yawRateF = 0.7f * yawRateF + 0.3f * yawRateRaw;
    }
    prevYaw = curYaw;
    havePrev = true;
    imuYawRate = yawRateF;

    if (controlMode == 2) {
      // ---- 手动逐电机模式：只走斜坡 + 看门狗，不做姿态辅助 ----
      manualOut1 = rampToward(manualOut1, (float)manualTarget1,
                              rampStep(manualOut1, (float)manualTarget1, RAMP_UP, RAMP_DOWN) * dt);
      manualOut2 = rampToward(manualOut2, (float)manualTarget2,
                              rampStep(manualOut2, (float)manualTarget2, RAMP_UP, RAMP_DOWN) * dt);
      lastSteerCorr = 0.0f;
      setMotorPercent(motor1, (int)lroundf(constrain(manualOut1, -100.0f, 100.0f)));
      setMotorPercent(motor2, (int)lroundf(constrain(manualOut2, -100.0f, 100.0f)));
    } else if (controlMode == 1) {
      // ---- 差速驾驶模式：斜坡 + 航向闭环 ----
      outThrottle = rampToward(outThrottle, (float)cmdThrottle,
                               rampStep(outThrottle, (float)cmdThrottle, RAMP_UP, RAMP_DOWN) * dt);
      outSteering = rampToward(outSteering, (float)cmdSteering,
                               rampStep(outSteering, (float)cmdSteering, RAMP_UP, RAMP_DOWN) * dt);

      // 航向：相对模式以摇杆方向角建立临时基准；绝对模式把摇杆方向当成“想去的世界方向”。
      float steerCorr = 0.0f;
      float vectorMag = hypotf((float)cmdThrottle, (float)cmdSteering);
      bool vectorActive = vectorMag > DRIVE_VECTOR_DEADZONE;
      float pivotIntent = pivotBlendForVector((float)cmdThrottle, (float)cmdSteering);
      if (!vectorActive) {
        driveVectorValid = false;
        headingValid = false;
        absYawErr = 0.0f;
        absReverseDrive = false;
      } else if (headingHoldOn && imuEulerValid) {
        if (assistMode == 2 && absForwardValid) {
          // 绝对“指哪打哪”：摇杆角度=想去的世界方向。车头/车尾哪端转得更少就用哪端对准，
          // 因此最大只需原地转 90°；尾端更近时倒车走，依旧朝同一个世界方向移动。
          float vectorAngle = atan2f((float)cmdSteering, (float)cmdThrottle) * 180.0f / PI;
          driveVectorAngleDeg = vectorAngle;
          driveVectorValid = true;
          float travelYaw = wrap180(absForwardYaw + vectorAngle);
          float headErr = wrap180(travelYaw - curYaw);            // 车头对准→前进
          float tailErr = wrap180(travelYaw + 180.0f - curYaw);   // 车尾对准→倒车
          const float REVERSE_HYST_DEG = 12.0f;                   // 迟滞，避免 ~90° 边界来回切换抖动
          if (absReverseDrive) {
            if (fabsf(headErr) + REVERSE_HYST_DEG < fabsf(tailErr)) absReverseDrive = false;
          } else {
            if (fabsf(tailErr) + REVERSE_HYST_DEG < fabsf(headErr)) absReverseDrive = true;
          }
          absYawErr = absReverseDrive ? tailErr : headErr;        // 选定那一端的航向误差(已≤90°)
          absTargetYaw = wrap180(curYaw + absYawErr);
          headingSetpoint = absTargetYaw;
          headingValid = true;
          float pivotCorr = ABS_TURN_SIGN * (ABS_HEAD_KP * absYawErr - ABS_HEAD_KD * yawRateF);
          steerCorr = constrain(pivotCorr, (float)-ABS_PIVOT_LIMIT, (float)ABS_PIVOT_LIMIT);
        } else {
          float vectorAngle = atan2f((float)cmdSteering, (float)cmdThrottle) * 180.0f / PI;
          bool vectorChanged = !driveVectorValid || fabsf(wrap180(vectorAngle - driveVectorAngleDeg)) > DRIVE_VECTOR_RESET_DEG;
          if (vectorChanged || !headingValid) {
            driveVectorAngleDeg = vectorAngle;
            driveVectorValid = true;
            headingSetpoint = curYaw;
            headingValid = true;
          }
          float err = wrap180(headingSetpoint - curYaw);
          absYawErr = err;
          // HEAD_SIGN 整体翻转纠偏方向（Kp 与 Kd 同时翻转，保证阻尼方向一致）。
          steerCorr = HEAD_SIGN * (HEAD_KP * err - HEAD_KD * yawRateF);
          // 越接近纯左/纯右，用户越是在主动原地转，航向锁定随之淡出，避免和旋转意图打架。
          steerCorr *= (1.0f - pivotIntent);
          steerCorr = constrain(steerCorr, (float)-HEAD_CORR_LIMIT, (float)HEAD_CORR_LIMIT);
        }
      } else {
        driveVectorValid = false;
        headingValid = false;
        absYawErr = 0.0f;
        absReverseDrive = false;
      }
      lastSteerCorr = steerCorr;

      float left = 0.0f;
      float right = 0.0f;
      if (assistMode == 2 && headingHoldOn && imuEulerValid && absForwardValid && vectorActive) {
        // 指哪打哪：行进速度=摇杆半径（走过斜坡的值）；航向越对准放行越多，
        // 偏差大时 forwardGate→0，几乎只剩 steerCorr 让左右轮反转做纯原地旋转。
        // 对准后再行进：车头对准就前进，车尾对准就倒车，二者都朝同一个世界方向移动。
        float speed = constrain(hypotf(outThrottle, outSteering), 0.0f, 100.0f);
        float alignSpan = fmaxf(ABS_PIVOT_FULL_DEG - ABS_PIVOT_START_DEG, 1.0f);
        float forwardGate = 1.0f - smoothStep01((fabsf(absYawErr) - ABS_PIVOT_START_DEG) / alignSpan);
        float drive = speed * forwardGate * (absReverseDrive ? -1.0f : 1.0f);
        left = drive + steerCorr;
        right = drive - steerCorr;
      } else {
        // 相对模式 / 关闭辅助：仍走原来的曲率差速混合 + 航向纠偏。
        float baseLeft = 0.0f;
        float baseRight = 0.0f;
        joystickToWheelMix(outThrottle, outSteering, baseLeft, baseRight);
        left = baseLeft + steerCorr;
        right = baseRight - steerCorr;
      }
      setMotorPercent(motor1, (int)lroundf(constrain(left, -100.0f, 100.0f)));
      setMotorPercent(motor2, (int)lroundf(constrain(right, -100.0f, 100.0f)));
    } else {
      // ---- 停止模式：所有输出平滑衰减到 0 再滑行，关闭辅助 ----
      outThrottle = rampToward(outThrottle, 0.0f, RAMP_DOWN * dt);
      outSteering = rampToward(outSteering, 0.0f, RAMP_DOWN * dt);
      manualOut1 = rampToward(manualOut1, 0.0f, RAMP_DOWN * dt);
      manualOut2 = rampToward(manualOut2, 0.0f, RAMP_DOWN * dt);
      lastSteerCorr = 0.0f;
      headingValid = false;
      driveVectorValid = false;
      float left = outThrottle + outSteering + manualOut1;
      float right = outThrottle - outSteering + manualOut2;
      setMotorPercent(motor1, (int)lroundf(constrain(left, -100.0f, 100.0f)));
      setMotorPercent(motor2, (int)lroundf(constrain(right, -100.0f, 100.0f)));
    }

    vTaskDelay(pdMS_TO_TICKS(MOTION_TICK_MS));
  }
}

void updateRpm(MotorState& motor) {
  unsigned long now = millis();
  if (now - motor.lastRpmMillis < RPM_INTERVAL_MS) {
    return;
  }

  long countSnapshot;
  noInterrupts();
  countSnapshot = motor.encoderCount;
  interrupts();

  unsigned long elapsed = now - motor.lastRpmMillis;
  long delta = countSnapshot - motor.lastEncoderCount;
  motor.rpm = (delta * 60000.0) / (elapsed * PULSES_PER_REV);
  motor.lastEncoderCount = countSnapshot;
  motor.lastRpmMillis = now;
}

bool beginImuAt(uint8_t address) {
  if (!imuSensor.begin(address, Wire)) {
    return false;
  }
  imuI2cAddress = address;
  // 用 Game Rotation Vector（仅陀螺+加速度，不依赖磁力计），避免电机磁铁干扰航向，等效原 IMU 的 VRU 模式。
  imuSensor.enableGameRotationVector(IMU_REPORT_INTERVAL_MS);
  imuSensor.enableAccelerometer(IMU_REPORT_INTERVAL_MS);
  imuSensor.enableGyro(IMU_REPORT_INTERVAL_MS);
  imuSensor.enableMagnetometer(IMU_REPORT_INTERVAL_MS);
  return true;
}

// 初始化 BNO080/BNO085（I2C）。找不到不阻塞，置 imuReady=false 后照常启动电机控制。
bool beginImu() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  if (beginImuAt(0x4B) || beginImuAt(0x4A)) {
    imuReady = true;
    Serial.printf("[IMU] BNO080/085 已连接 0x%02X (SDA=GPIO%d SCL=GPIO%d)\n", imuI2cAddress, I2C_SDA_PIN, I2C_SCL_PIN);
    return true;
  }
  imuReady = false;
  Serial.println("[IMU] 未检测到 BNO080/085，请检查 VCC/GND/SDA(GPIO43)/SCL(GPIO44)、PS0/PS1=I2C、ADD 地址。");
  return false;
}

// 四元数转欧拉角（度），与单测一致：yaw 取 ±180。
void updateEulerFromQuaternion(float w, float x, float y, float z) {
  float sinrCosp = 2.0f * (w * x + y * z);
  float cosrCosp = 1.0f - 2.0f * (x * x + y * y);
  imuRollDeg = atan2f(sinrCosp, cosrCosp) * 180.0f / PI;

  float sinp = 2.0f * (w * y - z * x);
  if (fabsf(sinp) >= 1.0f) {
    imuPitchDeg = copysignf(90.0f, sinp);
  } else {
    imuPitchDeg = asinf(sinp) * 180.0f / PI;
  }

  float sinyCosp = 2.0f * (w * z + x * y);
  float cosyCosp = 1.0f - 2.0f * (y * y + z * z);
  imuYawDeg = atan2f(sinyCosp, cosyCosp) * 180.0f / PI;
}

// 从 BNO080 读出全部可用报告，刷新全局姿态量（航向控制只依赖 imuYawDeg / imuEulerValid）。
void readImuSensor() {
  while (imuSensor.dataAvailable()) {
    imuQuatI = imuSensor.getQuatI();
    imuQuatJ = imuSensor.getQuatJ();
    imuQuatK = imuSensor.getQuatK();
    imuQuatReal = imuSensor.getQuatReal();
    imuQuatAccuracy = imuSensor.getQuatAccuracy();

    imuAccelX = imuSensor.getAccelX();
    imuAccelY = imuSensor.getAccelY();
    imuAccelZ = imuSensor.getAccelZ();

    // BNO080 陀螺单位 rad/s，转成 度/秒，与原协议保持一致。
    imuGyroX = imuSensor.getGyroX() * 180.0f / PI;
    imuGyroY = imuSensor.getGyroY() * 180.0f / PI;
    imuGyroZ = imuSensor.getGyroZ() * 180.0f / PI;
    imuGyroValid = true;

    imuMagX = imuSensor.getMagX();
    imuMagY = imuSensor.getMagY();
    imuMagZ = imuSensor.getMagZ();
    imuMagAccuracy = imuSensor.getMagAccuracy();

    updateEulerFromQuaternion(imuQuatReal, imuQuatI, imuQuatJ, imuQuatK);
    imuEulerValid = true;
    imuSampleCount++;
  }
}

void ensureWifiConnected() {
  static wl_status_t lastWifiStatus = WL_DISCONNECTED;
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (lastWifiStatus != WL_CONNECTED) {
      lastTcpAttemptMs = 0;
      lastCameraRetryMs = 0;
      Serial.println("[WiFi] 已恢复连接，立即重连服务器");
    }
    lastWifiStatus = status;
    return;
  }

  if (lastWifiStatus == WL_CONNECTED) {
    if (cameraWsClient.available()) {
      cameraWsClient.close();
    }
    cameraWsReady = false;
    lastTcpAttemptMs = 0;
    lastCameraRetryMs = 0;
  }
  lastWifiStatus = status;

  uint32_t now = millis();
  if (now - lastWifiCheckMs < WIFI_CHECK_MS) {
    return;
  }
  lastWifiCheckMs = now;

  Serial.println("[WiFi] 连接断开，正在重连...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// 构造上报给电脑 WebUI 的 BNO080 姿态 JSON（字段沿用单测格式，便于复用可视化）。
String buildImuPostJson() {
  String json;
  json.reserve(440);
  json += "{\"device\":\"esp32-car-bno080\"";
  json += ",\"seq\":" + String(imuPostCount++);
  json += ",\"ms\":" + String(millis());
  json += ",\"i2c_address\":\"0x" + String(imuI2cAddress, HEX) + "\"";
  json += ",\"euler_deg\":{\"roll\":" + String(imuRollDeg, 2) + ",\"pitch\":" + String(imuPitchDeg, 2) + ",\"yaw\":" + String(imuYawDeg, 2) + "}";
  json += ",\"quat\":{\"i\":" + String(imuQuatI, 6) + ",\"j\":" + String(imuQuatJ, 6) + ",\"k\":" + String(imuQuatK, 6) + ",\"real\":" + String(imuQuatReal, 6) + ",\"accuracy\":" + String(imuQuatAccuracy) + "}";
  json += ",\"accel_ms2\":{\"x\":" + String(imuAccelX, 4) + ",\"y\":" + String(imuAccelY, 4) + ",\"z\":" + String(imuAccelZ, 4) + "}";
  json += ",\"gyro_dps\":{\"x\":" + String(imuGyroX, 4) + ",\"y\":" + String(imuGyroY, 4) + ",\"z\":" + String(imuGyroZ, 4) + "}";
  json += ",\"mag_ut\":{\"x\":" + String(imuMagX, 4) + ",\"y\":" + String(imuMagY, 4) + ",\"z\":" + String(imuMagZ, 4) + ",\"accuracy\":" + String(imuMagAccuracy) + "}";
  json += "}";
  return json;
}

// 把最新姿态 POST 到电脑 WebUI（独立任务里调用，短超时，电脑不在线也不拖慢航向读取）。
void postImuState() {
  if (!imuReady || WiFi.status() != WL_CONNECTED) {
    return;
  }
  HTTPClient http;
  http.setConnectTimeout(400);
  http.setTimeout(500);
  String url = String("http://") + PYTHON_SERVER_IP + ":" + String(IMU_HTTP_PORT) + IMU_HTTP_PATH;
  if (!http.begin(url)) {
    return;
  }
  http.addHeader("Content-Type", "application/json");
  http.POST(buildImuPostJson());
  http.end();
}

String imuJson() {
  String json = "{";
  json += "\"ready\":" + String(imuReady ? "true" : "false") + ",";
  json += "\"i2cAddress\":\"0x" + String(imuI2cAddress, HEX) + "\",";
  json += "\"sampleCount\":" + String(imuSampleCount) + ",";
  json += "\"postCount\":" + String(imuPostCount) + ",";
  json += "\"quatAccuracy\":" + String(imuQuatAccuracy) + ",";
  json += "\"magAccuracy\":" + String(imuMagAccuracy) + ",";
  json += "\"eulerValid\":" + String(imuEulerValid ? "true" : "false") + ",";
  json += "\"eulerDeg\":[";
  json += String(imuRollDeg, 3) + ",";
  json += String(imuPitchDeg, 3) + ",";
  json += String(imuYawDeg, 3) + "],";
  json += "\"server\":\"" + String(PYTHON_SERVER_IP) + ":" + String(IMU_HTTP_PORT) + IMU_HTTP_PATH + "\"";
  json += "}";
  return json;
}

// IMU 读取环：未就绪则每 2s 重试初始化；就绪后持续读取 BNO080。
void handleImuBridge() {
  if (!imuReady) {
    static uint32_t lastRetryMs = 0;
    uint32_t now = millis();
    if (now - lastRetryMs > 2000) {
      lastRetryMs = now;
      beginImu();
    }
    return;
  }
  readImuSensor();
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_7;
  config.ledc_timer = LEDC_TIMER_1;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = CAMERA_FRAME_SIZE;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.fb_count = CAMERA_FB_COUNT;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    if (ENABLE_DEBUG_LOG) {
      Serial.printf("[CAM] init failed: 0x%x\n", err);
    }
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 1);
  }

  if (ENABLE_DEBUG_LOG) {
    Serial.println("[CAM] Camera OK");
  }
  return true;
}

void setupCameraWebSocket() {
  cameraWsClient.onEvent([](WebsocketsEvent event, String) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      cameraWsReady = true;
      Serial.println("[CAM] WebSocket 已连接");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      cameraWsReady = false;
      Serial.println("[CAM] WebSocket 已断开，准备重连");
    }
  });

  cameraWsClient.onMessage([](WebsocketsMessage message) {
    if (!message.isText()) {
      return;
    }
    String text = message.data();
    text.trim();
    if (text.startsWith("SET:FPS=")) {
      int fps = text.substring(8).toInt();
      cameraTargetFps = (fps <= 0) ? 0 : constrain(fps, 1, 15);
      if (ENABLE_DEBUG_LOG) {
        Serial.printf("[CAM] target fps=%d\n", cameraTargetFps);
      }
    }
  });
}

void connectCameraWebSocket() {
  ensureWifiConnected();
  if (!cameraReady || !cameraStreamEnabled || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (cameraWsMutex != nullptr && xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;
  }

  if (cameraWsReady || cameraWsClient.available()) {
    if (cameraWsMutex != nullptr) {
      xSemaphoreGive(cameraWsMutex);
    }
    return;
  }

  uint32_t now = millis();
  if (now - lastCameraRetryMs < CAMERA_WS_RECONNECT_MS) {
    if (cameraWsMutex != nullptr) {
      xSemaphoreGive(cameraWsMutex);
    }
    return;
  }
  lastCameraRetryMs = now;

  cameraWsReady = false;
  Serial.printf("[CAM] 连接 WebSocket %s:%u%s ...\n", CAMERA_WS_HOST, CAMERA_WS_PORT, CAMERA_WS_PATH);
  bool ok = cameraWsClient.connect(CAMERA_WS_HOST, CAMERA_WS_PORT, CAMERA_WS_PATH);
  if (ok) {
    cameraWsReady = true;
    Serial.println("[CAM] WebSocket 连接成功");
  } else {
    cameraWsClient.close();
    Serial.printf("[CAM] WebSocket 连接失败，%ums 后重试\n", CAMERA_WS_RECONNECT_MS);
  }

  if (cameraWsMutex != nullptr) {
    xSemaphoreGive(cameraWsMutex);
  }
}

void pollCameraWebSocket() {
  if (!cameraReady || !cameraStreamEnabled) {
    return;
  }
  if (cameraWsMutex != nullptr && xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;
  }
  cameraWsClient.poll();
  if (cameraWsMutex != nullptr) {
    xSemaphoreGive(cameraWsMutex);
  }
}

void setCameraStreamEnabled(bool enabled) {
  cameraStreamEnabled = enabled;
  if (enabled) {
    lastCameraRetryMs = 0;
    return;
  }

  if (cameraWsMutex == nullptr || xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    cameraWsClient.close();
    cameraWsReady = false;
    if (cameraWsMutex != nullptr) {
      xSemaphoreGive(cameraWsMutex);
    }
  }

  if (cameraFrameQueue != nullptr) {
    CameraFramePtr dropped = nullptr;
    while (xQueueReceive(cameraFrameQueue, &dropped, 0) == pdPASS) {
      if (dropped) {
        esp_camera_fb_return(dropped);
        cameraDroppedCount++;
      }
    }
  }
}

void taskCameraCapture(void*) {
  uint32_t lastCaptureMs = 0;
  for (;;) {
    if (!cameraReady || !cameraStreamEnabled || !cameraWsReady || cameraFrameQueue == nullptr || cameraTargetFps <= 0) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    uint32_t now = millis();
    uint32_t frameIntervalMs = 1000UL / static_cast<uint32_t>(cameraTargetFps);
    if (lastCaptureMs != 0 && (now - lastCaptureMs) < frameIntervalMs) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb && fb->format == PIXFORMAT_JPEG) {
      lastCaptureMs = now;
      cameraCapturedCount++;
      if (xQueueSend(cameraFrameQueue, &fb, 0) != pdPASS) {
        CameraFramePtr dropped = nullptr;
        if (xQueueReceive(cameraFrameQueue, &dropped, 0) == pdPASS && dropped) {
          esp_camera_fb_return(dropped);
          cameraDroppedCount++;
        }
        xQueueSend(cameraFrameQueue, &fb, 0);
      }
    } else if (fb) {
      esp_camera_fb_return(fb);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void taskCameraSend(void*) {
  for (;;) {
    CameraFramePtr fb = nullptr;
    if (cameraFrameQueue != nullptr && xQueueReceive(cameraFrameQueue, &fb, pdMS_TO_TICKS(100)) == pdPASS) {
      if (fb && cameraStreamEnabled && cameraWsReady) {
        bool ok = false;
        if (cameraWsMutex == nullptr || xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          ok = cameraWsClient.sendBinary(reinterpret_cast<const char*>(fb->buf), fb->len);
          if (!ok) {
            cameraWsClient.close();
            cameraWsReady = false;
            lastCameraRetryMs = 0;
          }
          if (cameraWsMutex != nullptr) {
            xSemaphoreGive(cameraWsMutex);
          }
        }
        if (ok) {
          cameraSentCount++;
        }
      }
      if (fb) {
        esp_camera_fb_return(fb);
      }
    }
  }
}

String cameraJson() {
  String json = "{";
  json += "\"ready\":" + String(cameraReady ? "true" : "false") + ",";
  json += "\"streamEnabled\":" + String(cameraStreamEnabled ? "true" : "false") + ",";
  json += "\"wsConnected\":" + String(cameraWsReady ? "true" : "false") + ",";
  json += "\"host\":\"" + String(CAMERA_WS_HOST) + ":" + String(CAMERA_WS_PORT) + "\",";
  json += "\"targetFps\":" + String(cameraTargetFps) + ",";
  json += "\"captured\":" + String(cameraCapturedCount) + ",";
  json += "\"sent\":" + String(cameraSentCount) + ",";
  json += "\"dropped\":" + String(cameraDroppedCount);
  json += "}";
  return json;
}

String driveJson() {
  String json = "{";
  json += "\"throttle\":" + String(driveThrottle) + ",";
  json += "\"steering\":" + String(driveSteering) + ",";
  json += "\"left\":" + String(motor1.commandPercent) + ",";
  json += "\"right\":" + String(motor2.commandPercent);
  json += "}";
  return json;
}

String controlJson() {
  String json = "{";
  json += "\"mode\":" + String(controlMode) + ",";
  json += "\"outThrottle\":" + String(outThrottle, 1) + ",";
  json += "\"outSteering\":" + String(outSteering, 1) + ",";
  json += "\"steerCorr\":" + String(lastSteerCorr, 1) + ",";
  json += "\"assistMode\":" + String(assistMode) + ",";
  json += "\"headingHold\":" + String(headingHoldOn ? "true" : "false") + ",";
  json += "\"headingValid\":" + String(headingValid ? "true" : "false") + ",";
  json += "\"headingSetpoint\":" + String(headingSetpoint, 1) + ",";
  json += "\"driveVectorValid\":" + String(driveVectorValid ? "true" : "false") + ",";
  json += "\"driveVectorAngle\":" + String(driveVectorAngleDeg, 1) + ",";
  json += "\"absForwardValid\":" + String(absForwardValid ? "true" : "false") + ",";
  json += "\"absForwardYaw\":" + String(absForwardYaw, 1) + ",";
  json += "\"absTargetYaw\":" + String(absTargetYaw, 1) + ",";
  json += "\"absYawErr\":" + String(absYawErr, 1) + ",";
  json += "\"imuYaw\":" + String(imuYawDeg, 1) + ",";
  json += "\"imuPitch\":" + String(imuPitchDeg, 1) + ",";
  json += "\"yawRate\":" + String(imuYawRate, 1) + ",";
  json += "\"headKp\":" + String(HEAD_KP, 2) + ",";
  json += "\"headKd\":" + String(HEAD_KD, 2) + ",";
  json += "\"rampUp\":" + String(RAMP_UP, 0) + ",";
  json += "\"rampDown\":" + String(RAMP_DOWN, 0) + ",";
  json += "\"steerDeadzone\":" + String(STEER_DEADZONE) + ",";
  json += "\"corrLimit\":" + String(HEAD_CORR_LIMIT) + ",";
  json += "\"headSign\":" + String(HEAD_SIGN) + ",";
  json += "\"steerGain\":" + String(STEER_GAIN, 2) + ",";
  json += "\"absHeadKp\":" + String(ABS_HEAD_KP, 2) + ",";
  json += "\"absHeadKd\":" + String(ABS_HEAD_KD, 2) + ",";
  json += "\"absTurnSign\":" + String(ABS_TURN_SIGN) + ",";
  json += "\"absPivotLimit\":" + String(ABS_PIVOT_LIMIT) + ",";
  json += "\"absPivotStartDeg\":" + String(ABS_PIVOT_START_DEG, 1) + ",";
  json += "\"absPivotFullDeg\":" + String(ABS_PIVOT_FULL_DEG, 1) + ",";
  json += "\"timeoutMs\":" + String(CMD_TIMEOUT_MS);
  json += "}";
  return json;
}

String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

String directJson() {
  uint8_t peer[6];
  memcpy(peer, directLastPeer, sizeof(peer));
  String json = "{";
  json += "\"ready\":" + String(directReady ? "true" : "false") + ",";
  json += "\"rx\":" + String(directRxCount) + ",";
  json += "\"bad\":" + String(directBadCount) + ",";
  json += "\"ack\":" + String(directAckCount) + ",";
  json += "\"lastRxMs\":" + String(directLastRxMs) + ",";
  json += "\"lastSeq\":" + String(directLastSeq) + ",";
  json += "\"lastType\":" + String(directLastType) + ",";
  json += "\"peer\":\"" + macToString(peer) + "\"";
  json += "}";
  return json;
}

void taskMotorHttp(void*) {
  for (;;) {
    processDirectControlPacket();
    server.handleClient();
    updateRpm(motor1);
    updateRpm(motor2);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void taskImuBridge(void*) {
  for (;;) {
    handleImuBridge();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void taskImuUpload(void*) {
  for (;;) {
    postImuState();
    vTaskDelay(pdMS_TO_TICKS(IMU_POST_INTERVAL_MS));
  }
}

void taskCameraWsMaintain(void*) {
  for (;;) {
    connectCameraWebSocket();
    pollCameraWebSocket();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void taskCameraHttp(void*) {
  for (;;) {
    cameraServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

String motorJson(MotorState& motor) {
  long countSnapshot;
  noInterrupts();
  countSnapshot = motor.encoderCount;
  interrupts();

  String json = "{";
  json += "\"id\":" + String(motor.id) + ",";
  json += "\"direction\":\"" + motor.direction + "\",";
  json += "\"mode\":\"pwm\",";
  json += "\"percent\":" + String(motor.commandPercent) + ",";
  json += "\"pwmDuty\":" + String(map(abs(motor.commandPercent), 0, 100, 0, PWM_MAX)) + ",";
  json += "\"encoderCount\":" + String(countSnapshot) + ",";
  json += "\"rpm\":" + String(motor.rpm, 2);
  json += "}";
  return json;
}

String statusJson() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"motors\":[";
  json += motorJson(motor1);
  json += ",";
  json += motorJson(motor2);
  json += "],";
  json += "\"imu\":";
  json += imuJson();
  json += ",";
  json += "\"camera\":";
  json += cameraJson();
  json += ",";
  json += "\"drive\":";
  json += driveJson();
  json += ",";
  json += "\"control\":";
  json += controlJson();
  json += ",";
  json += "\"direct\":";
  json += directJson();
  json += "}";
  return json;
}

void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendCameraCorsHeaders() {
  cameraServer.sendHeader("Access-Control-Allow-Origin", "*");
  cameraServer.sendHeader("Access-Control-Allow-Methods", "GET,OPTIONS");
  cameraServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendStatus() {
  sendCorsHeaders();
  server.send(200, "application/json; charset=utf-8", statusJson());
}

void handleOptions() {
  sendCorsHeaders();
  server.send(204);
}

bool applyDriveCommand(int throttle, int steering) {
  if (throttle < -100 || throttle > 100 || steering < -100 || steering > 100) {
    return false;
  }

  // 差速驾驶模式：只设目标，由控制环做斜坡 + 航向纠偏。
  controlMode = 1;
  cmdThrottle = throttle;
  cmdSteering = steering;
  manualTarget1 = 0;
  manualTarget2 = 0;
  driveThrottle = throttle;
  driveSteering = steering;
  lastCmdMs = millis();
  return true;
}

void applyFullStopCommand() {
  // 显式全部停止：进入停止模式，所有输出平滑滑停并关闭辅助。
  controlMode = 0;
  cmdThrottle = 0;
  cmdSteering = 0;
  manualTarget1 = 0;
  manualTarget2 = 0;
  driveThrottle = 0;
  driveSteering = 0;
  headingValid = false;
  driveVectorValid = false;
  lastCmdMs = millis();
}

void applyAssistConfig(int nextAssistMode, bool hasHeadingHold, bool nextHeadingHold, bool setAbsForward) {
  assistMode = constrain(nextAssistMode, 1, 2);
  if (setAbsForward && imuEulerValid) {
    absForwardYaw = imuYawDeg;
    absTargetYaw = absForwardYaw;
    absForwardValid = true;
  }
  if (hasHeadingHold) {
    headingHoldOn = nextHeadingHold;
  }
  if (assistMode == 2 && !absForwardValid && imuEulerValid) {
    absForwardYaw = imuYawDeg;
    absTargetYaw = absForwardYaw;
    absForwardValid = true;
  }
  // 重新建立航向基准，避免旧设定点造成跳变。
  headingValid = false;
  driveVectorValid = false;
}

void rememberDirectPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);
}

void sendDirectAck(const uint8_t* mac, const DirectControlPacket& request) {
  rememberDirectPeer(mac);

  DirectControlPacket ack = {};
  ack.magic = DIRECT_MAGIC;
  ack.version = DIRECT_VERSION;
  ack.type = DIRECT_TYPE_ACK;
  ack.seq = request.seq;
  ack.throttle = driveThrottle;
  ack.steering = driveSteering;
  ack.assistMode = constrain(assistMode, 1, 2);
  ack.flags = headingHoldOn ? DIRECT_FLAG_HEADING_HOLD : 0;
  ack.ms = millis();

  if (esp_now_send(mac, reinterpret_cast<uint8_t*>(&ack), sizeof(ack)) == ESP_OK) {
    directAckCount++;
  }
}

void processDirectControlPacket() {
  DirectControlPacket packet;
  uint8_t peer[6];
  bool hasPacket = false;

  portENTER_CRITICAL(&directMux);
  if (directPacketPending) {
    packet = directPendingPacket;
    memcpy(peer, directPendingMac, sizeof(peer));
    directPacketPending = false;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&directMux);

  if (!hasPacket) {
    return;
  }

  rememberDirectPeer(peer);
  switch (packet.type) {
    case DIRECT_TYPE_HELLO:
      break;
    case DIRECT_TYPE_DRIVE:
      if (!applyDriveCommand(packet.throttle, packet.steering)) {
        directBadCount++;
      }
      break;
    case DIRECT_TYPE_STOP:
      applyFullStopCommand();
      break;
    case DIRECT_TYPE_CONFIG:
      applyAssistConfig(
        packet.assistMode,
        true,
        (packet.flags & DIRECT_FLAG_HEADING_HOLD) != 0,
        (packet.flags & DIRECT_FLAG_SET_ABS_FORWARD) != 0
      );
      break;
    default:
      directBadCount++;
      return;
  }

  sendDirectAck(peer, packet);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDirectDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info ? info->src_addr : nullptr;
#else
void onDirectDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (mac == nullptr || data == nullptr || len != static_cast<int>(sizeof(DirectControlPacket))) {
    directBadCount++;
    return;
  }

  DirectControlPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != DIRECT_MAGIC || packet.version != DIRECT_VERSION) {
    directBadCount++;
    return;
  }

  portENTER_CRITICAL(&directMux);
  directPendingPacket = packet;
  memcpy(directPendingMac, mac, sizeof(directPendingMac));
  memcpy(directLastPeer, mac, sizeof(directLastPeer));
  directPacketPending = true;
  directRxCount++;
  directLastRxMs = millis();
  directLastSeq = packet.seq;
  directLastType = packet.type;
  portEXIT_CRITICAL(&directMux);
}

void setupDirectNow() {
  if (esp_now_init() != ESP_OK) {
    directReady = false;
    Serial.println("ESP-NOW 接收初始化失败");
    return;
  }

  esp_now_register_recv_cb(onDirectDataRecv);
  directReady = true;

  uint8_t primaryChannel = 0;
  wifi_second_chan_t secondChannel = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);
  Serial.printf("ESP-NOW 接收已启动，MAC=%s，信道=%u\n", WiFi.macAddress().c_str(), primaryChannel);
}

void handlePwm() {
  int motorId = server.hasArg("motor") ? server.arg("motor").toInt() : 1;
  int percent = server.arg("value").toInt();

  if (percent < -100 || percent > 100) {
    sendCorsHeaders();
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"percent invalid\"}");
    return;
  }

  // 手动逐电机模式：只设目标，由控制环做斜坡 + 看门狗（不做姿态辅助）。
  controlMode = 2;
  if (motorId == 2) {
    manualTarget2 = percent;
  } else {
    manualTarget1 = percent;
  }
  cmdThrottle = 0;
  cmdSteering = 0;
  driveThrottle = 0;
  driveSteering = 0;
  lastCmdMs = millis();
  sendStatus();
}

void handleDrive() {
  int throttle = server.hasArg("throttle") ? server.arg("throttle").toInt() : 0;
  int steering = server.hasArg("steering") ? server.arg("steering").toInt() : 0;

  if (!applyDriveCommand(throttle, steering)) {
    sendCorsHeaders();
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"drive range invalid\"}");
    return;
  }

  sendStatus();
}

void handleStop() {
  // 指定 motor 时只停该电机（手动调试用），保持手动模式；否则整体停止滑停。
  if (server.hasArg("motor")) {
    controlMode = 2;
    if (server.arg("motor").toInt() == 2) {
      manualTarget2 = 0;
    } else {
      manualTarget1 = 0;
    }
    lastCmdMs = millis();
    sendStatus();
    return;
  }
  applyFullStopCommand();
  sendStatus();
}

void handleConfig() {
  int nextAssistMode = server.hasArg("assistMode") ? server.arg("assistMode").toInt() : assistMode;
  bool hasHeadingHold = server.hasArg("headingHold");
  bool nextHeadingHold = hasHeadingHold ? (server.arg("headingHold").toInt() != 0) : headingHoldOn;
  bool setAbsForward = server.hasArg("setAbsForward") && server.arg("setAbsForward").toInt() != 0;
  applyAssistConfig(nextAssistMode, hasHeadingHold, nextHeadingHold, setAbsForward);
  if (server.hasArg("headKp")) HEAD_KP = constrain(server.arg("headKp").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("headKd")) HEAD_KD = constrain(server.arg("headKd").toFloat(), 0.0f, 5.0f);
  if (server.hasArg("rampUp")) RAMP_UP = constrain(server.arg("rampUp").toFloat(), 10.0f, 2000.0f);
  if (server.hasArg("rampDown")) RAMP_DOWN = constrain(server.arg("rampDown").toFloat(), 10.0f, 2000.0f);
  if (server.hasArg("steerDeadzone")) STEER_DEADZONE = constrain(server.arg("steerDeadzone").toInt(), 0, 50);
  if (server.hasArg("corrLimit")) HEAD_CORR_LIMIT = constrain(server.arg("corrLimit").toInt(), 0, 100);
  if (server.hasArg("headSign")) HEAD_SIGN = (server.arg("headSign").toInt() < 0) ? -1 : 1;
  if (server.hasArg("steerGain")) STEER_GAIN = constrain(server.arg("steerGain").toFloat(), 0.1f, 1.0f);
  if (server.hasArg("timeoutMs")) CMD_TIMEOUT_MS = (uint32_t)constrain(server.arg("timeoutMs").toInt(), 100, 5000);
  if (server.hasArg("absHeadKp")) ABS_HEAD_KP = constrain(server.arg("absHeadKp").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("absHeadKd")) ABS_HEAD_KD = constrain(server.arg("absHeadKd").toFloat(), 0.0f, 5.0f);
  if (server.hasArg("absTurnSign")) ABS_TURN_SIGN = (server.arg("absTurnSign").toInt() < 0) ? -1 : 1;
  if (server.hasArg("absPivotLimit")) ABS_PIVOT_LIMIT = constrain(server.arg("absPivotLimit").toInt(), 0, 100);
  if (server.hasArg("absPivotStartDeg")) ABS_PIVOT_START_DEG = constrain(server.arg("absPivotStartDeg").toFloat(), 0.0f, 90.0f);
  if (server.hasArg("absPivotFullDeg")) ABS_PIVOT_FULL_DEG = constrain(server.arg("absPivotFullDeg").toFloat(), ABS_PIVOT_START_DEG + 1.0f, 180.0f);
  if (ABS_PIVOT_FULL_DEG <= ABS_PIVOT_START_DEG) ABS_PIVOT_FULL_DEG = ABS_PIVOT_START_DEG + 1.0f;
  sendStatus();
}

void handleCameraStreamControl() {
  if (server.hasArg("enabled")) {
    setCameraStreamEnabled(server.arg("enabled").toInt() != 0);
  }
  sendStatus();
}

void handleCameraJpg() {
  if (!cameraReady) {
    sendCameraCorsHeaders();
    cameraServer.send(503, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"camera not ready\"}");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) {
      esp_camera_fb_return(fb);
    }
    sendCameraCorsHeaders();
    cameraServer.send(503, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"camera frame unavailable\"}");
    return;
  }

  sendCameraCorsHeaders();
  cameraServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  cameraServer.setContentLength(fb->len);
  cameraServer.send(200, "image/jpeg", "");
  WiFiClient client = cameraServer.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void setupPwm(MotorState& motor) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(motor.in1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor.in2, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(motor.pwmChannel1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motor.pwmChannel2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(motor.in1, motor.pwmChannel1);
  ledcAttachPin(motor.in2, motor.pwmChannel2);
#endif
  setMotorPercent(motor, 0);
}

void printEsp32Ip() {
  Serial.println();
  Serial.println("========== ESP32 网络信息 ==========");
  Serial.print("IP 地址: ");
  Serial.println(WiFi.localIP());
  Serial.print("控制页面: http://");
  Serial.println(WiFi.localIP());
  Serial.print("摄像头快照: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/camera.jpg");
  Serial.println("==================================");
  Serial.println();
  Serial.flush();
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  if (USE_STATIC_IP) {
    WiFi.config(localIp, gateway, subnet, dns1);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("正在连接 WiFi: ");
  Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  printEsp32Ip();
  Serial.printf("电脑 WebUI: %s (IMU上报 HTTP %u%s, 摄像头 WS %u)\n", PYTHON_SERVER_IP, IMU_HTTP_PORT, IMU_HTTP_PATH, CAMERA_WS_PORT);
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(800);  // 等待 USB 串口监视器就绪
  connectWifi();
  setupDirectNow();

  beginImu();  // 初始化 BNO080（I2C）；找不到不阻塞，电机控制照常启动。
  xTaskCreatePinnedToCore(taskImuBridge, "imu_read", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskImuUpload, "imu_upload", 6144, nullptr, 1, nullptr, 0);

  pinMode(motor1.encoderA, INPUT_PULLUP);
  pinMode(motor1.encoderB, INPUT_PULLUP);
  pinMode(motor2.encoderA, INPUT_PULLUP);
  pinMode(motor2.encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor1.encoderA), handleEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.encoderA), handleEncoder2A, CHANGE);

  setupPwm(motor1);
  setupPwm(motor2);

  cameraReady = initCamera();
  if (cameraReady) {
    cameraFrameQueue = xQueueCreate(3, sizeof(CameraFramePtr));
    cameraWsMutex = xSemaphoreCreateMutex();
    setupCameraWebSocket();
    connectCameraWebSocket();
    cameraServer.on("/camera.jpg", HTTP_GET, handleCameraJpg);
    cameraServer.onNotFound([]() {
      sendCameraCorsHeaders();
      cameraServer.send(404, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"not found\"}");
    });
    cameraServer.begin();
    xTaskCreatePinnedToCore(taskCameraCapture, "cam_cap", 10240, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(taskCameraSend, "cam_snd", 8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(taskCameraWsMaintain, "cam_ws", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(taskCameraHttp, "cam_http", 6144, nullptr, 1, nullptr, 0);
  }

  unsigned long now = millis();
  motor1.lastRpmMillis = now;
  motor2.lastRpmMillis = now;

  server.on("/status", HTTP_GET, sendStatus);
  server.on("/pwm", HTTP_POST, handlePwm);
  server.on("/pwm", HTTP_GET, handlePwm);
  server.on("/drive", HTTP_POST, handleDrive);
  server.on("/drive", HTTP_GET, handleDrive);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/config", HTTP_POST, handleConfig);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/camera-stream", HTTP_POST, handleCameraStreamControl);
  server.on("/camera-stream", HTTP_GET, handleCameraStreamControl);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      handleOptions();
      return;
    }
    sendCorsHeaders();
    server.send(404, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"not found\"}");
  });

  server.begin();
  lastCmdMs = millis();
  xTaskCreatePinnedToCore(taskMotorHttp, "motor_http", 6144, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskMotionControl, "motion_ctrl", 4096, nullptr, 4, nullptr, 1);

  Serial.println("HTTP 服务已启动");
  printEsp32Ip();
}

void loop() {
  ensureWifiConnected();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
