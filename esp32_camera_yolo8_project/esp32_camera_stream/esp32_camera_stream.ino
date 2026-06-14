#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <ArduinoWebsockets.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

using namespace websockets;

// 修改为你的 Wi-Fi 和运行 app_camera_yolo.py 的电脑 IP。
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* CAMERA_WS_HOST = "192.168.152.216";
const uint16_t CAMERA_WS_PORT = 8081;
const char* CAMERA_WS_PATH = "/ws/camera";

const uint32_t USB_BAUD = 115200;
const bool ENABLE_DEBUG_LOG = true;

// 比 QVGA 更清晰，同时比 VGA 更稳、更低延迟。
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_CIF;
const int CAMERA_JPEG_QUALITY = 14;
const int CAMERA_FB_COUNT = 2;
volatile int cameraTargetFps = 8;

// Seeed Studio XIAO ESP32S3 Sense OV2640 camera pins.
// 如果你用 AI Thinker ESP32-CAM，请替换成对应引脚。
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

WebServer statusServer(80);
WebsocketsClient cameraWsClient;

typedef camera_fb_t* CameraFramePtr;
QueueHandle_t cameraFrameQueue = nullptr;
SemaphoreHandle_t cameraWsMutex = nullptr;

uint32_t lastCameraRetryMs = 0;
volatile bool cameraReady = false;
volatile bool cameraWsReady = false;
volatile unsigned long cameraCapturedCount = 0;
volatile unsigned long cameraSentCount = 0;
volatile unsigned long cameraDroppedCount = 0;

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
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
    Serial.printf("[CAM] init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_hmirror(sensor, 1);
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 1);
    sensor->set_sharpness(sensor, 1);
  }

  Serial.println("[CAM] Camera OK");
  return true;
}

void setupCameraWebSocket() {
  cameraWsClient.onEvent([](WebsocketsEvent event, String) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      cameraWsReady = true;
      Serial.println("[CAM] WebSocket connected");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      cameraWsReady = false;
      Serial.println("[CAM] WebSocket disconnected");
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
      Serial.printf("[CAM] target fps=%d\n", cameraTargetFps);
    }
  });
}

void connectCameraWebSocket() {
  if (!cameraReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (cameraWsMutex != nullptr && xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;
  }

  if (cameraWsClient.available()) {
    if (cameraWsMutex != nullptr) {
      xSemaphoreGive(cameraWsMutex);
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastCameraRetryMs < 2000) {
    if (cameraWsMutex != nullptr) {
      xSemaphoreGive(cameraWsMutex);
    }
    return;
  }
  lastCameraRetryMs = now;

  cameraWsReady = false;
  Serial.printf("[CAM] Connecting ws://%s:%u%s\n", CAMERA_WS_HOST, CAMERA_WS_PORT, CAMERA_WS_PATH);
  cameraWsClient.connect(CAMERA_WS_HOST, CAMERA_WS_PORT, CAMERA_WS_PATH);

  if (cameraWsMutex != nullptr) {
    xSemaphoreGive(cameraWsMutex);
  }
}

void pollCameraWebSocket() {
  if (!cameraReady) {
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

void taskCameraCapture(void*) {
  uint32_t lastCaptureMs = 0;
  for (;;) {
    if (!cameraReady || !cameraWsReady || cameraFrameQueue == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    uint32_t now = millis();
    if (cameraTargetFps > 0) {
      uint32_t frameIntervalMs = 1000UL / static_cast<uint32_t>(cameraTargetFps);
      if (lastCaptureMs != 0 && (now - lastCaptureMs) < frameIntervalMs) {
        vTaskDelay(pdMS_TO_TICKS(2));
        continue;
      }
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
      if (fb && cameraWsReady) {
        bool ok = false;
        if (cameraWsMutex == nullptr || xSemaphoreTake(cameraWsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          ok = cameraWsClient.sendBinary(reinterpret_cast<const char*>(fb->buf), fb->len);
          if (!ok) {
            cameraWsClient.close();
            cameraWsReady = false;
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

void taskCameraWsMaintain(void*) {
  for (;;) {
    connectCameraWebSocket();
    pollCameraWebSocket();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

String statusJson() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"ready\":" + String(cameraReady ? "true" : "false") + ",";
  json += "\"wsConnected\":" + String(cameraWsReady ? "true" : "false") + ",";
  json += "\"target\":\"ws://" + String(CAMERA_WS_HOST) + ":" + String(CAMERA_WS_PORT) + String(CAMERA_WS_PATH) + "\",";
  json += "\"targetFps\":" + String(cameraTargetFps) + ",";
  json += "\"captured\":" + String(cameraCapturedCount) + ",";
  json += "\"sent\":" + String(cameraSentCount) + ",";
  json += "\"dropped\":" + String(cameraDroppedCount);
  json += "}";
  return json;
}

void sendCorsHeaders() {
  statusServer.sendHeader("Access-Control-Allow-Origin", "*");
  statusServer.sendHeader("Access-Control-Allow-Methods", "GET,OPTIONS");
  statusServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleStatus() {
  sendCorsHeaders();
  statusServer.send(200, "application/json; charset=utf-8", statusJson());
}

void handleCameraJpg() {
  if (!cameraReady) {
    sendCorsHeaders();
    statusServer.send(503, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"camera not ready\"}");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) {
      esp_camera_fb_return(fb);
    }
    sendCorsHeaders();
    statusServer.send(503, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"camera frame unavailable\"}");
    return;
  }

  sendCorsHeaders();
  statusServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  statusServer.setContentLength(fb->len);
  statusServer.send(200, "image/jpeg", "");
  WiFiClient client = statusServer.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void taskStatusHttp(void*) {
  for (;;) {
    statusServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(800);

  connectWifi();
  cameraReady = initCamera();
  if (!cameraReady) {
    Serial.println("[CAM] Camera init failed, check board type and pins.");
    return;
  }

  cameraFrameQueue = xQueueCreate(3, sizeof(CameraFramePtr));
  cameraWsMutex = xSemaphoreCreateMutex();
  setupCameraWebSocket();

  statusServer.on("/", HTTP_GET, handleStatus);
  statusServer.on("/status", HTTP_GET, handleStatus);
  statusServer.on("/camera.jpg", HTTP_GET, handleCameraJpg);
  statusServer.onNotFound([]() {
    sendCorsHeaders();
    statusServer.send(404, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"not found\"}");
  });
  statusServer.begin();

  xTaskCreatePinnedToCore(taskCameraCapture, "cam_cap", 10240, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskCameraSend, "cam_snd", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskCameraWsMaintain, "cam_ws", 4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(taskStatusHttp, "http", 4096, nullptr, 1, nullptr, 0);

  Serial.println("ESP32 camera stream started.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

