/*
  Seeed Studio XIAO ESP32-S3 实体摇杆遥控器

  接线（摇杆模块 → XIAO ESP32-S3 右侧，与 GND/3.3V 同侧集约）：
    摇杆 GND  → XIAO GND
    摇杆 +5V  → XIAO 3.3V-OUT   （务必接 3.3V，不要接 5V/VBUS）
    摇杆 VRx  → XIAO D10 / A10 / GPIO9  （X 轴，左右）
    摇杆 VRy  → XIAO D9 / A9 / GPIO8    （Y 轴，前后）
    摇杆 SW   → XIAO D8 / A8 / GPIO7    （按下按键，内部上拉，按下为 LOW）

  功能：
    - 读取摇杆，映射为 throttle / steering（-100 ~ 100）
    - 优先通过 ESP-NOW 直连发送到小车 ESP32，HTTP /drive 作为备用
    - 按住摇杆时 120ms 心跳保活，避免小车看门狗误判松手
    - XIAO ESP32S3 Sense 板载麦克风 16kHz/20ms PCM 流式上传到语音识别 sidecar
    - SW 短按 → 设当前 IMU 方向为绝对前方；按住 1s 以上 → 重新校准中心
    - 本机 80 端口提供 /status，供 Python WebUI 读取遥测

  Arduino IDE 依赖：
    - esp32 开发板包（XIAO ESP32S3）
    - ArduinoWebsockets
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#include <ESP_I2S.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

// ============ 请按你的网络修改 ============
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* REMOTE_HOSTNAME = "esp32-joystick-remote";

// 小车（电机控制板）ESP32 的 IP，与 app.py 里 ESP32_BASE_URL 一致
const char* ROBOT_ESP32_IP = "192.168.152.11";
const uint16_t ROBOT_ESP32_PORT = 80;
const bool DEFAULT_ROBOT_FORWARD_ENABLED = true;  // 上电后默认允许摇杆直接控制小车；可通过 WebUI /forward 关闭。

// 语音方向控制 sidecar（运行 voice_drive_realtime.py 的电脑）
const bool VOICE_AUDIO_ENABLED = true;
const char* VOICE_SERVER_HOST = "192.168.152.216";
const uint16_t VOICE_SERVER_PORT = 8091;
const char* VOICE_WS_PATH = "/ws_voice_audio";

const bool USE_STATIC_IP = false;
IPAddress localIp(192, 168, 152, 60);
IPAddress gateway(192, 168, 152, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(192, 168, 152, 1);

// ============ 引脚（右侧 D8~D10，与 GND/3.3V 同侧） ============
const int PIN_JOY_X = 9;   // D10 / A10 / GPIO9
const int PIN_JOY_Y = 8;   // D9 / A9 / GPIO8
const int PIN_JOY_SW = 7;  // D8 / A8 / GPIO7

// XIAO ESP32S3 Sense 板载 PDM 麦克风。若是非 Sense 版本，请把 VOICE_AUDIO_ENABLED 设为 false。
const int I2S_MIC_CLOCK_PIN = 42;
const int I2S_MIC_DATA_PIN = 41;

// ============ 摇杆参数 ============
const int ADC_MAX = 4095;
const int DEAD_ZONE = 12;       // 实体摇杆中心会漂移，稍大死区避免无操作误触发。
const int ADC_FILTER_SAMPLES = 8;
const float JOYSTICK_FILTER_ALPHA = 0.22f; // 简单一阶低通：越大越跟手，越小越稳。
const int JOYSTICK_OUTPUT_HOLD_DELTA = 3;  // 输出变化小于该值时保持上一档，减少手抖跳变。
const int CALIB_SAMPLES = 64;
const int SAMPLE_INTERVAL_MS = 5;
const uint32_t DRIVE_SEND_INTERVAL_MS = 45;   // 与 WebUI 节流一致
const uint32_t DRIVE_KEEPALIVE_MS = 120;      // 看门狗保活
const uint32_t HTTP_TIMEOUT_MS = 120;         // 小车不可达时快速失败，避免卡住摇杆/WebUI。
const uint32_t ROBOT_FAIL_BACKOFF_MS = 600;   // 连不上小车时不要每一帧都阻塞重试。
const uint32_t STATUS_PRINT_MS = 100;   // 有操作时每 100ms 打印一次
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t WIFI_RECONNECT_INTERVAL_MS = 3000;
const uint32_t MDNS_RETRY_MS = 5000;
const int CALIBRATION_VERSION = 2;
const int DEFAULT_CENTER_X = 2746;
const int DEFAULT_CENTER_Y = 2814;
const int DEFAULT_CAL_X_MIN = 146;
const int DEFAULT_CAL_X_MAX = 4095;
const int DEFAULT_CAL_Y_MIN = 292;
const int DEFAULT_CAL_Y_MAX = 4095;
const int DEFAULT_CONTROL_ROTATION_DEG = 0;   // 0/90/180/270，用于让手柄方向对齐小车方向。
const uint32_t BUTTON_DEBOUNCE_MS = 40;
const uint32_t BUTTON_CENTER_CAL_MS = 1000;
const uint32_t DIRECT_ACK_FRESH_MS = 1500;
const uint32_t DIRECT_PEER_STALE_MS = 3500;
const uint32_t DIRECT_DISCOVERY_MS = 700;
const int VOICE_SAMPLE_RATE = 16000;
const int VOICE_CHUNK_MS = 20;
const int VOICE_BYTES_PER_CHUNK = VOICE_SAMPLE_RATE * VOICE_CHUNK_MS / 1000 * 2;
const uint32_t VOICE_WS_RETRY_MS = 3000;

const uint32_t DIRECT_MAGIC = 0x52444345UL; // "ECDR" little-endian: ESP32 car direct remote
const uint8_t DIRECT_VERSION = 1;
const uint8_t DIRECT_TYPE_HELLO = 1;
const uint8_t DIRECT_TYPE_DRIVE = 2;
const uint8_t DIRECT_TYPE_STOP = 3;
const uint8_t DIRECT_TYPE_CONFIG = 4;
const uint8_t DIRECT_TYPE_ACK = 5;
const uint8_t DIRECT_FLAG_HEADING_HOLD = 0x01;
const uint8_t DIRECT_FLAG_SET_ABS_FORWARD = 0x02;
const uint8_t DIRECT_BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

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

struct AudioChunk {
  size_t n;
  uint8_t data[VOICE_BYTES_PER_CHUNK];
};

WebServer server(80);
Preferences prefs;
I2SClass i2sIn;
WebsocketsClient wsVoice;
QueueHandle_t qAudio = nullptr;

int centerX = DEFAULT_CENTER_X;
int centerY = DEFAULT_CENTER_Y;
int calXMin = DEFAULT_CAL_X_MIN;
int calXMax = DEFAULT_CAL_X_MAX;
int calYMin = DEFAULT_CAL_Y_MIN;
int calYMax = DEFAULT_CAL_Y_MAX;
volatile int rawX = 0;
volatile int rawY = 0;
volatile float physicalNormX = 0.0f; // 物理摇杆位置 -100~100（端点 remap 后，未旋转）
volatile float physicalNormY = 0.0f;
volatile float normX = 0.0f;         // 控制向量 -100~100（按 controlRotationDeg 旋转后）
volatile float normY = 0.0f;
volatile int throttleRaw = 0;
volatile int steeringRaw = 0;
volatile float vectorMag = 0.0f;
volatile int throttle = 0;
volatile int steering = 0;
volatile bool driveActive = false;
volatile bool buttonPressed = false;
volatile bool robotForwardEnabled = DEFAULT_ROBOT_FORWARD_ENABLED;
volatile int controlRotationDeg = DEFAULT_CONTROL_ROTATION_DEG;
volatile int remoteAssistMode = 1;   // 1=相对方向矫正，2=绝对方向模式
volatile bool robotReachable = false;
volatile int lastHttpCode = 0;
volatile uint32_t driveSentCount = 0;
volatile uint32_t driveFailCount = 0;
volatile uint32_t driveSkipCount = 0;
volatile uint32_t lastDriveSentMs = 0;
volatile uint32_t lastRobotAttemptMs = 0;
volatile uint32_t lastRobotResponseMs = 0;
volatile uint32_t wifiReconnectCount = 0;
String lastRobotError = "";

volatile bool directReady = false;
volatile bool directPeerKnown = false;
volatile uint32_t directSendCount = 0;
volatile uint32_t directFailCount = 0;
volatile uint32_t directAckCount = 0;
volatile uint32_t directBadCount = 0;
volatile uint32_t directLastSendMs = 0;
volatile uint32_t directLastAckMs = 0;
volatile uint16_t directLastAckSeq = 0;
volatile uint8_t directLastAckType = 0;
volatile uint32_t directPeerResetCount = 0;
uint16_t directSeq = 0;
uint32_t lastDirectDiscoveryMs = 0;
uint8_t directPeerMac[6] = {0};

volatile bool voiceMicReady = false;
volatile bool voiceWsReady = false;
volatile bool voiceStreamEnabled = false;
volatile uint32_t voiceAudioSentCount = 0;
volatile uint32_t voiceAudioFailCount = 0;
volatile uint32_t voiceLastConnectMs = 0;
volatile uint32_t voiceLastChunkMs = 0;
String voiceLastMessage = "";

float filteredControlX = 0.0f;
float filteredControlY = 0.0f;
bool joystickFilterReady = false;
int heldThrottle = 0;
int heldSteering = 0;
bool joystickOutputHoldReady = false;
uint32_t lastKeepaliveMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t lastWifiReconnectAttemptMs = 0;
uint32_t lastMdnsAttemptMs = 0;
bool mdnsStarted = false;
int lastPrintedThrottle = 0;
int lastPrintedSteering = 0;
int pendingThrottle = 0;
int pendingSteering = 0;
bool hasPendingDrive = false;
bool lastButtonPressed = false;
bool buttonLongPressHandled = false;
uint32_t buttonPressedAtMs = 0;

static void saveCalibration();
static void loadCalibration();

static int readAxis(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_FILTER_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / ADC_FILTER_SAMPLES);
}

static float remapAxis(int value, int center, int minValue, int maxValue) {
  int lowSpan = max(center - minValue, 1);
  int highSpan = max(maxValue - center, 1);
  if (value >= center) {
    return constrain((float)(value - center) / highSpan, 0.0f, 1.0f);
  }
  return -constrain((float)(center - value) / lowSpan, 0.0f, 1.0f);
}

static int normalizeRotation(int degrees) {
  int r = degrees % 360;
  if (r < 0) r += 360;
  if (r < 45) return 0;
  if (r < 135) return 90;
  if (r < 225) return 180;
  if (r < 315) return 270;
  return 0;
}

static void rotateControlVector(float inX, float inY, float& outX, float& outY) {
  switch (normalizeRotation(controlRotationDeg)) {
    case 90:
      outX = inY;
      outY = -inX;
      break;
    case 180:
      outX = -inX;
      outY = -inY;
      break;
    case 270:
      outX = -inY;
      outY = inX;
      break;
    default:
      outX = inX;
      outY = inY;
      break;
  }
}

static void mapJoystickToDrive(int axisX, int axisY, int& outThrottle, int& outSteering) {
  // 与 WebUI 圆形摇杆一致：上推为正 throttle，右推为负 steering。
  float px = remapAxis(axisX, centerX, calXMin, calXMax);
  float py = -remapAxis(axisY, centerY, calYMin, calYMax);
  float cx = 0.0f;
  float cy = 0.0f;
  rotateControlVector(px, py, cx, cy);

  float mag = hypotf(cx, cy);
  float dead = DEAD_ZONE / 100.0f;
  if (mag < dead) {
    filteredControlX = 0.0f;
    filteredControlY = 0.0f;
    joystickFilterReady = false;
  } else if (!joystickFilterReady) {
    filteredControlX = cx;
    filteredControlY = cy;
    joystickFilterReady = true;
  } else {
    filteredControlX += (cx - filteredControlX) * JOYSTICK_FILTER_ALPHA;
    filteredControlY += (cy - filteredControlY) * JOYSTICK_FILTER_ALPHA;
  }

  physicalNormX = px * 100.0f;
  physicalNormY = py * 100.0f;
  normX = filteredControlX * 100.0f;
  normY = filteredControlY * 100.0f;
  throttleRaw = (int)lroundf(normY);
  steeringRaw = (int)lroundf(-normX);
  vectorMag = hypotf(normX, normY);
  int candidateThrottle = constrain(throttleRaw, -100, 100);
  int candidateSteering = constrain(steeringRaw, -100, 100);
  if (candidateThrottle == 0 && candidateSteering == 0) {
    heldThrottle = 0;
    heldSteering = 0;
    joystickOutputHoldReady = false;
  } else if (!joystickOutputHoldReady) {
    heldThrottle = candidateThrottle;
    heldSteering = candidateSteering;
    joystickOutputHoldReady = true;
  } else {
    if (candidateThrottle == 0 || abs(candidateThrottle - heldThrottle) >= JOYSTICK_OUTPUT_HOLD_DELTA) {
      heldThrottle = candidateThrottle;
    }
    if (candidateSteering == 0 || abs(candidateSteering - heldSteering) >= JOYSTICK_OUTPUT_HOLD_DELTA) {
      heldSteering = candidateSteering;
    }
  }
  outThrottle = heldThrottle;
  outSteering = heldSteering;
}

static bool isOperating() {
  return driveActive || buttonPressed;
}

static void maybePrintDriveStatus() {
  // 测试通过后默认不在串口持续打印摇杆坐标，避免串口监视器刷屏。
}

static String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static String jsonEscape(String value) {
  value.replace("\\", "\\\\");
  value.replace("\"", "\\\"");
  value.replace("\n", "\\n");
  value.replace("\r", "\\r");
  return value;
}

static bool directAckFresh() {
  return directReady && directPeerKnown && (millis() - directLastAckMs) <= DIRECT_ACK_FRESH_MS;
}

static void forgetDirectPeerIfStale() {
  if (!directPeerKnown) {
    return;
  }
  uint32_t now = millis();
  if (directLastAckMs == 0 || now - directLastAckMs <= DIRECT_PEER_STALE_MS) {
    return;
  }
  directPeerKnown = false;
  directPeerResetCount++;
}

static void ensureDirectPeer(const uint8_t* mac) {
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

static void rememberDirectPeer(const uint8_t* mac) {
  memcpy(directPeerMac, mac, 6);
  directPeerKnown = true;
}

static bool sendDirectPacket(uint8_t type, int t = 0, int s = 0, bool setAbsForward = false, bool forceBroadcast = false) {
  if (!directReady) {
    return false;
  }

  DirectControlPacket packet = {};
  packet.magic = DIRECT_MAGIC;
  packet.version = DIRECT_VERSION;
  packet.type = type;
  packet.seq = ++directSeq;
  packet.throttle = constrain(t, -100, 100);
  packet.steering = constrain(s, -100, 100);
  packet.assistMode = constrain(remoteAssistMode, 1, 2);
  packet.flags = DIRECT_FLAG_HEADING_HOLD | (setAbsForward ? DIRECT_FLAG_SET_ABS_FORWARD : 0);
  packet.ms = millis();

  const uint8_t* target = (forceBroadcast || !directPeerKnown) ? DIRECT_BROADCAST_MAC : directPeerMac;
  ensureDirectPeer(target);
  esp_err_t result = esp_now_send(target, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  directLastSendMs = millis();
  if (result == ESP_OK) {
    directSendCount++;
    return true;
  }

  directFailCount++;
  return false;
}

static void maybeSendDirectDiscovery() {
  if (!directReady) {
    return;
  }
  forgetDirectPeerIfStale();
  if (directAckFresh()) {
    return;
  }
  uint32_t now = millis();
  if (now - lastDirectDiscoveryMs < DIRECT_DISCOVERY_MS) {
    return;
  }
  lastDirectDiscoveryMs = now;
  sendDirectPacket(DIRECT_TYPE_HELLO, 0, 0, false, true);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDirectDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info ? info->src_addr : nullptr;
#else
static void onDirectDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (mac == nullptr || data == nullptr || len != static_cast<int>(sizeof(DirectControlPacket))) {
    directBadCount++;
    return;
  }

  DirectControlPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != DIRECT_MAGIC || packet.version != DIRECT_VERSION || packet.type != DIRECT_TYPE_ACK) {
    directBadCount++;
    return;
  }

  rememberDirectPeer(mac);
  directAckCount++;
  directLastAckMs = millis();
  directLastAckSeq = packet.seq;
  directLastAckType = packet.type;
  robotReachable = true;
  lastRobotResponseMs = directLastAckMs;
}

static void setupDirectNow() {
  if (esp_now_init() != ESP_OK) {
    directReady = false;
    Serial.println("ESP-NOW 发送初始化失败");
    return;
  }

  ensureDirectPeer(DIRECT_BROADCAST_MAC);
  esp_now_register_recv_cb(onDirectDataRecv);
  directReady = true;

  uint8_t primaryChannel = 0;
  wifi_second_chan_t secondChannel = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);
  Serial.printf("ESP-NOW 发送已启动，MAC=%s，信道=%u\n", WiFi.macAddress().c_str(), primaryChannel);
}

static void initVoiceMic() {
  if (!VOICE_AUDIO_ENABLED) {
    voiceLastMessage = "voice disabled";
    return;
  }
  i2sIn.setPinsPdmRx(I2S_MIC_CLOCK_PIN, I2S_MIC_DATA_PIN);
  if (!i2sIn.begin(I2S_MODE_PDM_RX, VOICE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    voiceMicReady = false;
    voiceLastMessage = "mic init failed";
    Serial.println("语音麦克风初始化失败");
    return;
  }
  voiceMicReady = true;
  voiceLastMessage = "mic ready";
  Serial.println("语音麦克风已启动");
}

static void setupVoiceWebSocketCallbacks() {
  if (!VOICE_AUDIO_ENABLED) {
    return;
  }

  wsVoice.onEvent([](WebsocketsEvent event, String) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      voiceWsReady = true;
      voiceStreamEnabled = true;
      voiceLastMessage = "voice ws connected";
      if (qAudio) xQueueReset(qAudio);
      wsVoice.send("START");
      Serial.println("语音 WebSocket 已连接");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      voiceWsReady = false;
      voiceStreamEnabled = false;
      voiceLastMessage = "voice ws disconnected";
      Serial.println("语音 WebSocket 已断开");
    }
  });

  wsVoice.onMessage([](WebsocketsMessage msg) {
    if (!msg.isText()) {
      return;
    }
    String s = msg.data();
    s.trim();
    voiceLastMessage = s;
    if (s == "START") {
      voiceStreamEnabled = true;
      if (qAudio) xQueueReset(qAudio);
      wsVoice.send("OK:STARTED");
    } else if (s == "STOP") {
      voiceStreamEnabled = false;
      wsVoice.send("OK:STOPPED");
    } else if (s == "RESTART") {
      voiceStreamEnabled = false;
      if (qAudio) xQueueReset(qAudio);
      delay(60);
      voiceStreamEnabled = true;
      wsVoice.send("START");
    } else if (s.startsWith("ERROR:")) {
      voiceStreamEnabled = false;
    }
  });
}

static void maintainVoiceSocket() {
  if (!VOICE_AUDIO_ENABLED || !voiceMicReady || WiFi.status() != WL_CONNECTED) {
    return;
  }
  uint32_t now = millis();
  if (!wsVoice.available() && now - voiceLastConnectMs >= VOICE_WS_RETRY_MS) {
    voiceLastConnectMs = now;
    voiceLastMessage = "voice ws connecting";
    if (wsVoice.connect(VOICE_SERVER_HOST, VOICE_SERVER_PORT, VOICE_WS_PATH)) {
      Serial.println("语音 WebSocket 发起连接");
    }
  }
  wsVoice.poll();
}

void taskVoiceMicCapture(void*) {
  const int samplesPerChunk = VOICE_BYTES_PER_CHUNK / 2;
  for (;;) {
    if (VOICE_AUDIO_ENABLED && voiceMicReady && voiceWsReady && voiceStreamEnabled) {
      AudioChunk chunk;
      chunk.n = VOICE_BYTES_PER_CHUNK;
      int16_t* out = reinterpret_cast<int16_t*>(chunk.data);
      int i = 0;
      int retry = 0;
      while (i < samplesPerChunk && retry < 1000) {
        int value = i2sIn.read();
        if (value == -1) {
          delay(1);
          retry++;
          continue;
        }
        out[i++] = static_cast<int16_t>(value);
        retry = 0;
      }
      if (i == samplesPerChunk && qAudio) {
        if (xQueueSend(qAudio, &chunk, 0) != pdPASS) {
          AudioChunk dump;
          xQueueReceive(qAudio, &dump, 0);
          xQueueSend(qAudio, &chunk, 0);
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void taskVoiceMicUpload(void*) {
  for (;;) {
    if (VOICE_AUDIO_ENABLED && voiceWsReady && voiceStreamEnabled && qAudio) {
      AudioChunk chunk;
      if (xQueueReceive(qAudio, &chunk, pdMS_TO_TICKS(100)) == pdPASS) {
        bool ok = wsVoice.sendBinary(reinterpret_cast<const char*>(chunk.data), chunk.n);
        if (ok) {
          voiceAudioSentCount++;
          voiceLastChunkMs = millis();
        } else {
          voiceAudioFailCount++;
          voiceLastMessage = "voice binary send failed";
          wsVoice.close();
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

static bool postRobot(const String& path, int& httpCodeOut, String& errorOut) {
  lastRobotAttemptMs = millis();
  if (!robotForwardEnabled) {
    httpCodeOut = 0;
    errorOut = "robot forward disabled";
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    httpCodeOut = 0;
    errorOut = "wifi disconnected";
    return false;
  }

  HTTPClient http;
  String url = String("http://") + ROBOT_ESP32_IP + ":" + ROBOT_ESP32_PORT + path;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.begin(url);
  httpCodeOut = http.POST((uint8_t*)nullptr, 0);
  if (httpCodeOut > 0) {
    http.getString();
    errorOut = "";
    http.end();
    return httpCodeOut >= 200 && httpCodeOut < 300;
  }

  errorOut = http.errorToString(httpCodeOut);
  http.end();
  return false;
}

static bool robotSendAllowed(bool force) {
  if (force || robotReachable || driveFailCount == 0) {
    return true;
  }
  return (millis() - lastRobotAttemptMs) >= ROBOT_FAIL_BACKOFF_MS;
}

static void sendDriveNow(int t, int s) {
  if (!robotForwardEnabled) {
    driveSkipCount++;
    lastRobotError = "robot forward disabled";
    return;
  }

  bool directWasFresh = directAckFresh();
  bool directSent = sendDirectPacket(DIRECT_TYPE_DRIVE, t, s);
  lastDriveSentMs = millis();
  driveSentCount++;
  if (directSent && directWasFresh) {
    robotReachable = true;
    lastRobotError = "";
    return;
  }

  if (!robotSendAllowed(false)) {
    driveSkipCount++;
    lastRobotError = directSent ? "espnow sent, http retry backoff" : "robot retry backoff";
    return;
  }

  String path = String("/drive?throttle=") + t + "&steering=" + s;
  int code = 0;
  String err;
  bool ok = postRobot(path, code, err);
  lastHttpCode = code;
  if (ok) {
    robotReachable = true;
    lastRobotResponseMs = millis();
    lastRobotError = "";
  } else {
    driveFailCount++;
    robotReachable = false;
    lastRobotError = err.length() ? err : String("http ") + code;
  }
}

static void sendConfigToRobot(bool setAbsForward = false) {
  if (!robotForwardEnabled) {
    lastRobotError = "robot forward disabled";
    return;
  }

  bool directWasFresh = directAckFresh();
  bool directSent = sendDirectPacket(DIRECT_TYPE_CONFIG, 0, 0, setAbsForward);
  if (directSent && directWasFresh) {
    robotReachable = true;
    lastRobotError = "";
    return;
  }

  String path = String("/config?assistMode=") + remoteAssistMode + "&headingHold=1";
  if (setAbsForward && remoteAssistMode == 2) {
    path += "&setAbsForward=1";
  }

  int code = 0;
  String err;
  bool ok = postRobot(path, code, err);
  lastHttpCode = code;
  if (ok) {
    robotReachable = true;
    lastRobotResponseMs = millis();
    lastRobotError = "";
  } else {
    driveFailCount++;
    robotReachable = false;
    lastRobotError = err.length() ? err : String("http ") + code;
  }
}

static void toggleAssistMode() {
  remoteAssistMode = (remoteAssistMode == 1) ? 2 : 1;
  saveCalibration();
  sendConfigToRobot(false);
}

static void setAbsForwardNow() {
  remoteAssistMode = 2;
  saveCalibration();
  sendConfigToRobot(true);
}

static void queueDrive(int t, int s) {
  pendingThrottle = t;
  pendingSteering = s;
  hasPendingDrive = true;

  uint32_t now = millis();
  if (now - lastDriveSentMs >= DRIVE_SEND_INTERVAL_MS) {
    sendDriveNow(t, s);
    hasPendingDrive = false;
  }
}

static void flushPendingDrive() {
  if (!hasPendingDrive) {
    return;
  }
  if (millis() - lastDriveSentMs < DRIVE_SEND_INTERVAL_MS) {
    return;
  }
  sendDriveNow(pendingThrottle, pendingSteering);
  hasPendingDrive = false;
}

static void sendStop() {
  if (!robotForwardEnabled) {
    lastRobotError = "robot forward disabled";
    throttle = 0;
    steering = 0;
    driveActive = false;
    pendingThrottle = 0;
    pendingSteering = 0;
    hasPendingDrive = false;
    return;
  }

  // 停车命令允许跳过退避，尽量立即发给小车。
  bool directWasFresh = directAckFresh();
  bool directSent = sendDirectPacket(DIRECT_TYPE_STOP);
  lastDriveSentMs = millis();
  if (directSent && directWasFresh) {
    robotReachable = true;
    lastRobotResponseMs = millis();
    lastRobotError = "";
    throttle = 0;
    steering = 0;
    driveActive = false;
    pendingThrottle = 0;
    pendingSteering = 0;
    hasPendingDrive = false;
    return;
  }

  int code = 0;
  String err;
  bool ok = postRobot("/stop", code, err);
  lastHttpCode = code;
  if (ok) {
    robotReachable = true;
    lastRobotResponseMs = millis();
    lastRobotError = "";
  } else {
    robotReachable = false;
    lastRobotError = err.length() ? err : String("http ") + code;
  }
  throttle = 0;
  steering = 0;
  driveActive = false;
  pendingThrottle = 0;
  pendingSteering = 0;
  hasPendingDrive = false;
}

static void calibrateCenter() {
  Serial.println("请保持摇杆居中，正在校准...");
  delay(300);
  long sumX = 0;
  long sumY = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sumX += readAxis(PIN_JOY_X);
    sumY += readAxis(PIN_JOY_Y);
    delay(SAMPLE_INTERVAL_MS);
  }
  centerX = (int)(sumX / CALIB_SAMPLES);
  centerY = (int)(sumY / CALIB_SAMPLES);
  calXMin = min(calXMin, centerX - 100);
  calXMax = max(calXMax, centerX + 100);
  calYMin = min(calYMin, centerY - 100);
  calYMax = max(calYMax, centerY + 100);
  saveCalibration();
  Serial.printf("校准完成 centerX=%d centerY=%d\n", centerX, centerY);
}

static void resetEdgeCalibration() {
  centerX = DEFAULT_CENTER_X;
  centerY = DEFAULT_CENTER_Y;
  calXMin = DEFAULT_CAL_X_MIN;
  calXMax = DEFAULT_CAL_X_MAX;
  calYMin = DEFAULT_CAL_Y_MIN;
  calYMax = DEFAULT_CAL_Y_MAX;
}

static void saveCalibration() {
  prefs.putInt("ver", CALIBRATION_VERSION);
  prefs.putInt("cx", centerX);
  prefs.putInt("cy", centerY);
  prefs.putInt("xmin", calXMin);
  prefs.putInt("xmax", calXMax);
  prefs.putInt("ymin", calYMin);
  prefs.putInt("ymax", calYMax);
  prefs.putInt("rot", normalizeRotation(controlRotationDeg));
  prefs.putInt("mode", remoteAssistMode);
}

static void loadCalibration() {
  prefs.begin("joycal", false);
  if (prefs.getInt("ver", 0) != CALIBRATION_VERSION) {
    resetEdgeCalibration();
    saveCalibration();
    return;
  }
  centerX = prefs.getInt("cx", DEFAULT_CENTER_X);
  centerY = prefs.getInt("cy", DEFAULT_CENTER_Y);
  calXMin = prefs.getInt("xmin", DEFAULT_CAL_X_MIN);
  calXMax = prefs.getInt("xmax", DEFAULT_CAL_X_MAX);
  calYMin = prefs.getInt("ymin", DEFAULT_CAL_Y_MIN);
  calYMax = prefs.getInt("ymax", DEFAULT_CAL_Y_MAX);
  controlRotationDeg = normalizeRotation(prefs.getInt("rot", DEFAULT_CONTROL_ROTATION_DEG));
  remoteAssistMode = constrain(prefs.getInt("mode", 1), 1, 2);
}

static void recordEdge(const String& edge) {
  int sampleX = readAxis(PIN_JOY_X);
  int sampleY = readAxis(PIN_JOY_Y);
  if (edge == "center") {
    centerX = sampleX;
    centerY = sampleY;
  } else if (edge == "left") {
    calXMin = sampleX;
  } else if (edge == "right") {
    calXMax = sampleX;
  } else if (edge == "up") {
    calYMin = sampleY;
  } else if (edge == "down") {
    calYMax = sampleY;
  }

  if (calXMin > centerX) calXMin = centerX - 1;
  if (calXMax < centerX) calXMax = centerX + 1;
  if (calYMin > centerY) calYMin = centerY - 1;
  if (calYMax < centerY) calYMax = centerY + 1;
  saveCalibration();
}

static String jsonStatus() {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"device\":\"physical_remote\",";
  json += "\"driveMapping\":\"webui\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"robot\":\"" + String(ROBOT_ESP32_IP) + "\",";
  json += "\"throttle\":" + String(throttle) + ",";
  json += "\"steering\":" + String(steering) + ",";
  json += "\"throttleRaw\":" + String(throttleRaw) + ",";
  json += "\"steeringRaw\":" + String(steeringRaw) + ",";
  json += "\"physicalNormX\":" + String(physicalNormX, 1) + ",";
  json += "\"physicalNormY\":" + String(physicalNormY, 1) + ",";
  json += "\"normX\":" + String(normX, 1) + ",";
  json += "\"normY\":" + String(normY, 1) + ",";
  json += "\"vectorMag\":" + String(vectorMag, 1) + ",";
  json += "\"filterAlpha\":" + String(JOYSTICK_FILTER_ALPHA, 2) + ",";
  json += "\"outputHoldDelta\":" + String(JOYSTICK_OUTPUT_HOLD_DELTA) + ",";
  json += "\"driveActive\":" + String(driveActive ? "true" : "false") + ",";
  json += "\"buttonPressed\":" + String(buttonPressed ? "true" : "false") + ",";
  json += "\"robotForwardEnabled\":" + String(robotForwardEnabled ? "true" : "false") + ",";
  json += "\"controlRotationDeg\":" + String(normalizeRotation(controlRotationDeg)) + ",";
  json += "\"remoteAssistMode\":" + String(remoteAssistMode) + ",";
  json += "\"centerX\":" + String(centerX) + ",";
  json += "\"centerY\":" + String(centerY) + ",";
  json += "\"calXMin\":" + String(calXMin) + ",";
  json += "\"calXMax\":" + String(calXMax) + ",";
  json += "\"calYMin\":" + String(calYMin) + ",";
  json += "\"calYMax\":" + String(calYMax) + ",";
  json += "\"rawX\":" + String(rawX) + ",";
  json += "\"rawY\":" + String(rawY) + ",";
  json += "\"deadZone\":" + String(DEAD_ZONE) + ",";
  json += "\"robotReachable\":" + String(robotReachable ? "true" : "false") + ",";
  json += "\"lastHttpCode\":" + String(lastHttpCode) + ",";
  json += "\"wifiStatus\":" + String((int)WiFi.status()) + ",";
  json += "\"wifiReconnectCount\":" + String(wifiReconnectCount) + ",";
  json += "\"driveSentCount\":" + String(driveSentCount) + ",";
  json += "\"driveFailCount\":" + String(driveFailCount) + ",";
  json += "\"driveSkipCount\":" + String(driveSkipCount) + ",";
  json += "\"directReady\":" + String(directReady ? "true" : "false") + ",";
  json += "\"directPeerKnown\":" + String(directPeerKnown ? "true" : "false") + ",";
  json += "\"directAckFresh\":" + String(directAckFresh() ? "true" : "false") + ",";
  json += "\"directPeer\":\"" + macToString(directPeerMac) + "\",";
  json += "\"directSendCount\":" + String(directSendCount) + ",";
  json += "\"directFailCount\":" + String(directFailCount) + ",";
  json += "\"directAckCount\":" + String(directAckCount) + ",";
  json += "\"directBadCount\":" + String(directBadCount) + ",";
  json += "\"directLastSendMs\":" + String(directLastSendMs) + ",";
  json += "\"directLastAckMs\":" + String(directLastAckMs) + ",";
  json += "\"directLastAckSeq\":" + String(directLastAckSeq) + ",";
  json += "\"directPeerResetCount\":" + String(directPeerResetCount) + ",";
  json += "\"voiceAudioEnabled\":" + String(VOICE_AUDIO_ENABLED ? "true" : "false") + ",";
  json += "\"voiceMicReady\":" + String(voiceMicReady ? "true" : "false") + ",";
  json += "\"voiceWsReady\":" + String(voiceWsReady ? "true" : "false") + ",";
  json += "\"voiceStreamEnabled\":" + String(voiceStreamEnabled ? "true" : "false") + ",";
  json += "\"voiceAudioSentCount\":" + String(voiceAudioSentCount) + ",";
  json += "\"voiceAudioFailCount\":" + String(voiceAudioFailCount) + ",";
  json += "\"voiceLastChunkMs\":" + String(voiceLastChunkMs) + ",";
  json += "\"voiceLastMessage\":\"" + jsonEscape(voiceLastMessage) + "\",";
  json += "\"lastRobotAttemptMs\":" + String(lastRobotAttemptMs) + ",";
  json += "\"updatedAt\":" + String(millis()) + ",";
  json += "\"lastRobotError\":\"" + jsonEscape(lastRobotError) + "\"";
  json += "}";
  return json;
}

static void handleStatus() {
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleStopHttp() {
  sendStop();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"stopped\":true}");
}

static void handleRecalibrate() {
  calibrateCenter();
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleCalibrateEdge() {
  if (!server.hasArg("edge")) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"missing edge\"}");
    return;
  }

  String edge = server.arg("edge");
  if (edge != "center" && edge != "left" && edge != "right" && edge != "up" && edge != "down") {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"invalid edge\"}");
    return;
  }

  recordEdge(edge);
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleCalibrationReset() {
  resetEdgeCalibration();
  saveCalibration();
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleForward() {
  if (server.hasArg("enabled")) {
    robotForwardEnabled = server.arg("enabled").toInt() != 0;
    robotReachable = false;
    driveFailCount = 0;
    driveSkipCount = 0;
    lastRobotError = robotForwardEnabled ? "robot forward enabled" : "robot forward disabled";
    if (robotForwardEnabled) {
      sendConfigToRobot();
    }
  }
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleOrientation() {
  if (server.hasArg("rotate")) {
    controlRotationDeg = normalizeRotation(server.arg("rotate").toInt());
    saveCalibration();
  }
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void handleMode() {
  if (server.hasArg("assistMode")) {
    remoteAssistMode = constrain(server.arg("assistMode").toInt(), 1, 2);
    saveCalibration();
    sendConfigToRobot();
  } else if (server.hasArg("toggle") && server.arg("toggle").toInt() != 0) {
    toggleAssistMode();
  }
  server.send(200, "application/json; charset=utf-8", jsonStatus());
}

static void setupHttp() {
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/stop", HTTP_POST, handleStopHttp);
  server.on("/stop", HTTP_GET, handleStopHttp);
  server.on("/recalibrate", HTTP_POST, handleRecalibrate);
  server.on("/recalibrate", HTTP_GET, handleRecalibrate);
  server.on("/calibrate-edge", HTTP_POST, handleCalibrateEdge);
  server.on("/calibrate-edge", HTTP_GET, handleCalibrateEdge);
  server.on("/calibration-reset", HTTP_POST, handleCalibrationReset);
  server.on("/calibration-reset", HTTP_GET, handleCalibrationReset);
  server.on("/forward", HTTP_POST, handleForward);
  server.on("/forward", HTTP_GET, handleForward);
  server.on("/orientation", HTTP_POST, handleOrientation);
  server.on("/orientation", HTTP_GET, handleOrientation);
  server.on("/mode", HTTP_POST, handleMode);
  server.on("/mode", HTTP_GET, handleMode);
  server.onNotFound([]() {
    server.send(404, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"not found\"}");
  });
  server.begin();
  Serial.println("HTTP 服务已启动 :80");
}

static void startMdnsIfNeeded() {
  if (mdnsStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }
  uint32_t now = millis();
  if (lastMdnsAttemptMs != 0 && now - lastMdnsAttemptMs < MDNS_RETRY_MS) {
    return;
  }
  lastMdnsAttemptMs = now;
  if (MDNS.begin(REMOTE_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.print("遥控器 mDNS: http://");
    Serial.print(REMOTE_HOSTNAME);
    Serial.println(".local");
  } else {
    Serial.println("遥控器 mDNS 启动失败");
  }
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  if (USE_STATIC_IP) {
    WiFi.config(localIp, gateway, subnet, dns1);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("连接 WiFi");
  uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    lastRobotError = "wifi connect timeout";
    Serial.println("WiFi 连接超时，继续启动并在后台重连");
    return;
  }
  Serial.print("遥控器 IP: ");
  Serial.println(WiFi.localIP());
  startMdnsIfNeeded();
  Serial.print("小车地址: http://");
  Serial.print(ROBOT_ESP32_IP);
  Serial.println("/drive");
}

static void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    startMdnsIfNeeded();
    if (!directReady) {
      setupDirectNow();
    }
    return;
  }

  robotReachable = false;
  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
  }
  if (voiceWsReady) {
    wsVoice.close();
    voiceWsReady = false;
    voiceStreamEnabled = false;
  }

  uint32_t now = millis();
  if (now - lastWifiReconnectAttemptMs < WIFI_RECONNECT_INTERVAL_MS) {
    return;
  }
  lastWifiReconnectAttemptMs = now;
  wifiReconnectCount++;
  lastRobotError = "wifi reconnecting";
  Serial.println("WiFi 已断开，尝试重连...");
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  loadCalibration();
  connectWiFi();
  setupDirectNow();
  if (VOICE_AUDIO_ENABLED) {
    qAudio = xQueueCreate(10, sizeof(AudioChunk));
  }
  initVoiceMic();
  setupVoiceWebSocketCallbacks();
  Serial.printf(
    "已加载摇杆校准 center=(%d,%d) x=(%d,%d) y=(%d,%d)\n",
    centerX,
    centerY,
    calXMin,
    calXMax,
    calYMin,
    calYMax
  );
  setupHttp();
  if (VOICE_AUDIO_ENABLED) {
    xTaskCreatePinnedToCore(taskVoiceMicCapture, "voice_cap", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(taskVoiceMicUpload, "voice_upl", 4096, nullptr, 2, nullptr, 0);
  }
}

void loop() {
  server.handleClient();
  maintainWiFi();
  maintainVoiceSocket();

  rawX = readAxis(PIN_JOY_X);
  rawY = readAxis(PIN_JOY_Y);
  buttonPressed = digitalRead(PIN_JOY_SW) == LOW;
  bool wasDriveActive = driveActive;
  uint32_t now = millis();
  if (buttonPressed && !lastButtonPressed) {
    buttonPressedAtMs = now;
    buttonLongPressHandled = false;
  }
  if (buttonPressed && !buttonLongPressHandled && (now - buttonPressedAtMs) >= BUTTON_CENTER_CAL_MS) {
    buttonLongPressHandled = true;
    calibrateCenter();
  }
  if (!buttonPressed && lastButtonPressed) {
    uint32_t pressedFor = now - buttonPressedAtMs;
    if (!buttonLongPressHandled && pressedFor >= BUTTON_DEBOUNCE_MS && pressedFor < BUTTON_CENTER_CAL_MS) {
      setAbsForwardNow();
    } else if (!buttonLongPressHandled && pressedFor >= BUTTON_CENTER_CAL_MS) {
      calibrateCenter();
    }
  }
  lastButtonPressed = buttonPressed;

  int t = 0;
  int s = 0;
  mapJoystickToDrive(rawX, rawY, t, s);  // 始终更新物理孪生数据 normX/normY

  bool shouldSendDrive = false;
  bool shouldSendStop = false;
  if (buttonPressed) {
    throttle = 0;
    steering = 0;
    driveActive = false;
  } else {
    throttle = t;
    steering = s;
    bool active = !(t == 0 && s == 0);
    driveActive = active;
    shouldSendDrive = active;
    shouldSendStop = !active && wasDriveActive;
  }

  // 先更新串口/WebUI 可见状态，再尝试网络发送，避免小车不可达拖慢本地反馈。
  maybePrintDriveStatus();
  maybeSendDirectDiscovery();
  maintainVoiceSocket();
  server.handleClient();

  if (!robotForwardEnabled) {
    hasPendingDrive = false;
    pendingThrottle = throttle;
    pendingSteering = steering;
  } else if (shouldSendStop) {
    sendStop();
  } else if (shouldSendDrive) {
    queueDrive(throttle, steering);
  }

  if (driveActive && now - lastKeepaliveMs >= DRIVE_KEEPALIVE_MS) {
    lastKeepaliveMs = now;
    sendDriveNow(throttle, steering);
  }
  flushPendingDrive();
  maintainVoiceSocket();

  delay(10);
}
