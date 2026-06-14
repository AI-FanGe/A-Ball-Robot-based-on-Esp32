# 实体摇杆遥控器

独立项目，不影响现有 `app.py` 和小车 `esp32/esp32_motor_control`。

## 硬件

| 摇杆模块引脚 | 连接到 XIAO ESP32-S3（右侧） |
|-------------|---------------------------|
| GND         | GND                       |
| +5V         | **3.3V-OUT**（不要接 5V/VBUS） |
| VRx（X 轴） | D10 / A10 / GPIO9        |
| VRy（Y 轴） | D9 / A9 / GPIO8          |
| SW（按键）  | D8 / A8 / GPIO7          |

### 接线示意

右侧从上到下：GND、3.3V-OUT、D10、D9、D8，与摇杆排针顺序一一对应。

```
摇杆模块                XIAO ESP32-S3（右侧）
─────────              ───────────────────
  GND  ───────────────  GND
  +5V  ───────────────  3.3V-OUT
  VRx  ───────────────  D10 (GPIO9)
  VRy  ───────────────  D9  (GPIO8)
  SW   ───────────────  D8  (GPIO7)
```

> 这块遥控器板是**第二块** XIAO ESP32-S3，只负责读摇杆并发指令；小车电机、IMU、摄像头仍用原来的那块 ESP32。

## 烧录固件

1. 用 Arduino IDE 打开 `esp32_joystick_remote/esp32_joystick_remote.ino`
2. 开发板选 **Seeed XIAO ESP32S3**
3. 修改顶部配置：
   - `WIFI_SSID` / `WIFI_PASSWORD`
   - `ROBOT_ESP32_IP` → 小车 ESP32 的 IP（与主项目 `ESP32_BASE_URL` 一致）
4. 上电后**保持摇杆居中**，固件会自动校准中心点
5. 串口监视器 115200 可看到 `遥控器 IP: x.x.x.x`

## 运行 WebUI

```powershell
cd "e:\沙粒云\自媒体\2026视频制作\20260610世界杯\test\physical_remote"
python app_remote.py
```

浏览器打开 `http://127.0.0.1:8002`。

按需设置环境变量：

```powershell
$env:REMOTE_ESP32_URL="http://192.168.152.176"   # 遥控器 IP
$env:ROBOT_ESP32_URL="http://192.168.152.11"    # 小车 IP
python app_remote.py
```

## 控制逻辑

| 操作 | 行为 |
|------|------|
| 推动摇杆 | HTTP POST 小车 `/drive?throttle=&steering=` |
| 回中 | 发送 `(0, 0)` 停车 |
| 按住摇杆 | 每 120ms 保活，防止小车看门狗超时 |
| 按下摇杆按键 | 切换相对方向矫正 / 绝对方向模式 |
| WebUI「重新校准」 | POST 遥控器 `/recalibrate` |
| WebUI「方向对齐」 | 旋转控制向量 0/90/180/270 度 |
| WebUI「小车转发」 | 开启或关闭遥控器向小车发送 `/drive` |

方向映射与主项目 WebUI 一致：

- **上推** → 前进（正 throttle）
- **右推** → 右转（正 steering）

## API

### 遥控器 ESP32（端口 80）

- `GET /status` — 摇杆读数、小车连接状态
- `POST /stop` — 本地触发停车
- `POST /recalibrate` — 重新校准中心
- `POST /forward?enabled=1` — 开启小车转发
- `POST /forward?enabled=0` — 关闭小车转发
- `POST /orientation?rotate=90` — 设置方向旋转，支持 `0/90/180/270`
- `POST /mode?assistMode=1` — 相对方向矫正模式
- `POST /mode?assistMode=2` — 绝对方向模式
- `POST /mode?toggle=1` — 切换相对/绝对模式

### Python WebUI（端口 8002）

- `GET /` — 监控页面
- `GET /api/remote/status` — 代理遥控器状态
- `POST /api/remote/stop` — 代理停车
- `GET /api/robot/status` — 代理小车状态
