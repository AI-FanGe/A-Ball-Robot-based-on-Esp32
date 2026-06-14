# ESP32 Camera YOLOE 独立识别项目

这个目录是独立新项目，不会影响现有 `app.py` 和 `esp32/esp32_motor_control`。

## 文件说明

- `esp32_camera_stream/esp32_camera_stream.ino`：烧录到另一块 ESP32 摄像头板，负责采集 JPEG 并通过 WebSocket 发送到电脑。
- `app_camera_yolo.py`：电脑端接收 ESP32 摄像头帧，用 YOLOE (`yoloe-11l-seg.pt`) 做开放词汇识别，并提供网页预览。
- `requirements.txt`：Python 依赖清单。

## ESP32 烧录前要改

打开 `esp32_camera_stream/esp32_camera_stream.ino`，确认顶部配置：

```cpp
const char* WIFI_SSID = "你的WiFi名称";
const char* WIFI_PASSWORD = "你的WiFi密码";
const char* CAMERA_WS_HOST = "192.168.152.216";  // 运行 app_camera_yolo.py 的电脑 IP
```

当前引脚配置参考的是 `Seeed Studio XIAO ESP32S3 Sense OV2640`。如果你用的是 AI Thinker ESP32-CAM 或其他板子，需要替换摄像头引脚定义。

Arduino IDE 需要安装库：

- `esp32` 开发板包
- `ArduinoWebsockets`

## 电脑端运行

建议在这个目录里新建虚拟环境，不要全局安装依赖：

```powershell
cd "e:\沙粒云\自媒体\2026视频制作\20260610世界杯\test\esp32_camera_yolo8_project"
py -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python app_camera_yolo.py
```

启动后打开：

```text
http://127.0.0.1:8001
```

ESP32 会连接：

```text
ws://电脑IP:8081/ws/camera
```

第一次运行 `ultralytics` 会自动下载 `yoloe-11l-seg.pt`。默认检测提示词是 `football`，也可以在 WebUI 的“检测类型”里输入 `football, goal, person` 这类开放词汇提示词。

如果想启动时指定提示词：

```powershell
$env:TARGET_CLASSES="football"
.\.venv\Scripts\python app_camera_yolo.py
```

