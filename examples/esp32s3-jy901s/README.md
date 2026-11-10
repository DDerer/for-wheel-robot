# ESP32-S3 + JY901S 串口测试

独立 PlatformIO 工程，适用于照片中的 S3-N16R8 开发板。使用 Arduino 框架和原生 USB CDC 输出日志。

## 接线

| JY901S | ESP32-S3 |
| --- | --- |
| VCC | 5V |
| GND | GND |
| TX | GPIO12，主控 RX |
| RX | GPIO11，主控 TX |

使用已供电的开发板 5V 给模块供电。首次测试可用 USB 给开发板供电，传感器与开发板共地。D0、D1 保持悬空。

## 编译和运行

### 首次使用 VS Code

1. 按 `Ctrl+Shift+X` 打开扩展，搜索并安装官方 **PlatformIO IDE**，扩展 ID 为 `platformio.platformio-ide`。等待初始化完成，再按提示重启或重载 VS Code。
2. 使用“文件 → 打开文件夹”，打开 `E:\code\for-wheel\examples\esp32s3-jy901s`。左侧应直接看到 `platformio.ini` 和 `src`。
3. 点击左侧 PlatformIO 图标，在 **PROJECT TASKS → esp32s3-jy901s → General** 中点击 **Build**。首次使用会下载开发平台和编译工具，等待终端出现 `SUCCESS`。
4. 用支持数据传输的 USB 线连接开发板的原生 USB 接口，通常标为 USB 或 OTG。另一个 USB 转串口接口的日志输出位置不同，按开发板丝印区分。
5. 在 PlatformIO 的 **Devices** 中查看新增的 COM 端口，也可以打开 Windows 设备管理器查看。拔插前后对比可以辨认开发板端口。
6. 在上述 **General** 任务中点击 **Upload**，工具会自动检测端口。如检测到多台串口设备，可在 `platformio.ini` 环境段内增加 `upload_port = COM实际端口号` 和 `monitor_port = COM实际端口号`。
7. 烧录成功后点击 **Monitor**，串口监视器速度已设为 115200。按开发板 RST 键，观察启动和检测日志。

烧录是把编译生成的固件写入开发板 Flash，断电后固件会保留。重新上电后程序会从 `setup()` 开始运行，随后重复执行 `loop()`。每次修改代码后执行 Upload，工具会先重新编译再写入。

板型、16MB Flash、USB 日志和 IMU 引脚已经在工程中配置。本示例使用片内 RAM。

安装 PlatformIO CLI 后，也可以在本目录执行以下命令。

```powershell
pio run
pio device list
pio run -t upload --upload-port COM实际端口号
pio device monitor -p COM实际端口号 -b 115200
```

原生 USB 烧录后，设备可能重新枚举，需要再次查看端口号。若首次烧录无法连接，按住 BOOT，按下并松开 RST，再松开 BOOT 后重试。打开监视器后可以按 RST 重新观察启动日志。

如果端口被占用，先关闭其他串口工具或正在运行的监视器。Windows 设备管理器出现未知设备时，需要根据开发板 USB 接口实际使用的芯片确认驱动。原生 USB 接口直接连接 ESP32-S3，板载 USB 转串口口则经过额外的桥接芯片。

## 观察结果

启动后从 9600 波特率开始自动搜索，每档监听 6 秒。校验通过的加速度、角速度或角度帧会锁定波特率。

- `ACC` 是模块 X/Y/Z 加速度，单位 g，包含重力。静止时三轴向量模长应接近 1g。
- `GYRO` 是模块 X/Y/Z 角速度，单位度/秒。静止时应接近零，转动模块时应变化。
- `RPY` 是模块 Roll/Pitch/Yaw，单位度。缓慢倾斜模块时应连续变化，角度范围边界处可能跳变。
- `age` 是该类数据距上次接收的毫秒数，`frames` 是当前搜索档位以来该类数据的帧数。三类数据独立更新。
- `waiting` 表示尚未收到对应类型，检查模块是否启用了该类输出。
- `bytes` 和 `checksum_errors` 为启动以来累计值，搜索错误波特率时可能出现校验错误。

连续 15 秒未收到有效的上述数据会重新搜索。输出间隔超过 6 秒的模块需要增大代码中的搜索和超时常量。

如果 `bytes` 始终为零，检查供电、共地，以及模块 TX 到 GPIO12 的连接。如果一直搜索，检查接线、波特率和模块输出配置。若在 USB 转串口口烧录，应切换至原生 USB 口查看本示例日志。

此示例被动接收模块现有输出，用于验证模块 TX 到主控 RX 的链路。主控到模块的配置指令链路需要后续发送指令验证。姿态按模块坐标输出，机器人坐标映射在控制程序适配阶段处理。

协议依据为厂商的 [WIT 标准通信协议](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol)。

VS Code 操作参考 [PlatformIO IDE 官方说明](https://docs.platformio.org/en/latest/integration/ide/vscode.html)。
