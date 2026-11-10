# ESP32-S3 轮腿机器人主控

使用 S3-N16R8、JY901S 串口姿态模块和 TWAI CAN，基于 PlatformIO Arduino 框架。主控保留原工程的 LQR、VMC、腿部运动学、PID 和 BLE UUID。

## 接口

| 功能 | GPIO |
| --- | --- |
| CAN TX / RX | 4 / 5 |
| JY901S 主控 TX / RX | 11 / 12 |
| 按键中键 | 18，按下记录当前航向 |
| 按键停止方向 | 15，低电平停止输出 |
| 日志及命令 | 开发板原生 USB，115200 |

JY901S TX 接 GPIO12，RX 接 GPIO11，VCC 接5V，共地。底板电源灯接电源轨。电池 ADC 默认关闭，状态里的12V为名义值，电压补偿比例使用0.7。后续增加分压采样时，在 `src/board_config.h` 配置 ADC 引脚和实际分压比。

## 编译和烧录

在 VS Code 中打开本目录 `E:\code\for-wheel\esp32-controller\software`，通过 PlatformIO 的 Build、Upload、Monitor 操作。烧录和监视器端口通过 Devices 查看，首次下载可按住 BOOT，按一下 RST，再松开 BOOT。

PowerShell 可以执行以下命令，COM12 使用实际端口号。

```powershell
cd E:\code\for-wheel\esp32-controller\software
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio run
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio run -t upload --upload-port COM12
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio device monitor -p COM12 -b 115200
```

## IMU 配置与坐标

启动后自动识别 JY901S 波特率，每秒输出姿态、角速度、机身Z方向去重力加速度，以及三类数据的实际接收频率。状态行里的角度用度显示，控制内部使用 rad 和 rad/s，加速度使用 m/s²。

串口监视器输入 `imu-config` 并回车，将模块配置为115200波特率、100Hz，以及加速度/角速度/角度输出，并发送保存指令。配置过程中电机输出关闭。串口发送后必须检查三类实测帧率均接近100Hz，再断电重启检查配置是否保存。模块未收到配置指令时会继续按原设置输出，日志可以识别这种情况。

根据当前描述，模块正X朝机器人右侧、正Y朝前，采用机器人X朝前、Y朝左、Z朝上的右手坐标系。转换矩阵已设为 `bodyX=sensorY, bodyY=-sensorX, bodyZ=sensorZ`。机身朝向矩阵同时用于姿态、角速度和重力补偿。最终装配后需要验证Z方向、前倾符号及水平零位。

## 六电机映射与参数

| 位置 | 默认电机 ID | 反馈 CAN ID |
| --- | --- | --- |
| 左前关节 | 1 | 0x101 |
| 左后关节 | 2 | 0x102 |
| 左轮 | 3 | 0x103 |
| 右前关节 | 5 | 0x105 |
| 右后关节 | 6 | 0x106 |
| 右轮 | 7 | 0x107 |

映射在 `board_config.h` 的 `kMotorIds` 中配置，支持唯一的ID 1～8。主控按驱动代码的格式发送0x100和0x200电压指令，每个槽为有符号16位小端毫伏值，反馈按32位毫弧度和16位0.1RPM解析。CAN速率为1Mbps。

本地 `stm32-foc/software/Src/main.c` 当前在初始化中固定设置 `motorID = 5`。该文件代表一个测试ID，装到六块驱动板的实际固件应分别具有唯一ID。主控通过每个ID的online/offline状态检查接收情况。

`Motor_InitAll()` 中的关节角度偏移、方向、扭矩系数、电压上限，以及轮半径、腿部尺寸、质量和控制增益沿用原工程。需要用自己机器人的实际参数校准。

## 启动、命令与联调

启动状态为 `DISARMED`，周期发送零电压指令。可查看IMU与CAN反馈并配置模块。

| 命令 | 功能 |
| --- | --- |
| `status` | 输出状态，另有每秒自动状态输出 |
| `imu-config` | 配置并保存IMU通信及100Hz输出，关闭电机输出 |
| `arm` | 检查配置与反馈后使能控制 |
| `stop` | 关闭输出、取消运动目标 |
| `stand` | 使能后执行原工程的起立准备过程 |
| `heading` | 记录当前航向作为目标 |

命令以回车结束。原蓝牙目标指令0xAA、起立0xAB、采样0xAC和调试0xDB仍按原UUID接收，运动指令需要已使能。BLE断连触发停止。采样参数在创建任务时复制，避免使用回调缓存的失效地址。

首次使能前，在 `board_config.h` 完成安装方向和电机参数核对，再将 `kImuMountingConfirmed`、`kMotorCalibrationConfirmed` 设为true并重新编译。`arm` 要求六块电机反馈在100ms内更新，三类IMU数据在50ms内更新且实测频率均至少80Hz。反馈超时、CAN bus-off、机身/腿部角度超限、停止按键会关闭输出，恢复后需要重新发送arm。起立准备每个运动阶段有5秒超时。

联调顺序为配置和验证IMU、逐个检查电机ID及方向、核对关节零位和运动学、校准控制参数、使能后验证动作。

协议参考 [WIT 标准通信协议](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol) 和 [乐鑫 TWAI 文档](https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32s3/api-reference/peripherals/twai.html)。
