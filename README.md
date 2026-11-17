# ESP32 设备状态监测与断网续传终端

在 ESP32 上用六个 FreeRTOS 任务完成温湿度与振动采集、状态机告警、MQTT 上报和断网续传。核心逻辑与 Arduino 解耦，状态机、FFT 特征和环形缓存都能在主机上用 C++ 单测覆盖；完整硬件行为可在 Wokwi 中免硬件复现。

主要特性：

- 六任务架构，Core 1 跑采集/特征/控制/存储，Core 0 跑通信/监控，任务间用 Queue、Task Notification 和 Event Group 通信
- 五状态机（INIT/NORMAL/WARNING/FAULT/RECOVERY），带连续帧确认、回差区间和恢复升级
- 256 点振动窗口，Hann 窗 + FFT 提取主频，支持 Wokwi 合成信号与 MPU6050 实体采集两种来源
- LittleFS 环形缓存断网续传，QoS 1 收到 PUBACK 才删记录，重启后按 `bootId/sequence` 保序补传
- 六个任务各自订阅 ESP-IDF Task Watchdog，panic 复位后可回溯是哪个任务迟到

## 前置要求

| 依赖 | 用途 |
| --- | --- |
| [PlatformIO](https://platformio.org/)（VS Code 扩展或 Core CLI） | 构建与烧录 |
| [Wokwi for VS Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode) | 免硬件运行仿真 |
| Python 3 | 运行 `test/` 下的主机测试 |
| C++11 编译器（g++ / clang++） | 编译主机测试 |

## 在 Wokwi 中运行

用 VS Code 打开项目目录，启用 PlatformIO IDE 和 Wokwi 扩展。默认 Wi-Fi 为仿真器内置的 `Wokwi-GUEST`，实体板需要在本地 `include/secrets.h` 用 `WIFI_SSID` 和 `WIFI_PASSWORD` 覆盖。如需连 ThingsBoard，在同一文件填写设备 Access Token。首次配置可复制 [secrets.example.h](include/secrets.example.h)。默认 MQTT 地址为 `mqtt.eu.thingsboard.cloud:1883`，使用其他实例时可以在同一文件设置 `TB_MQTT_HOST` 和 `TB_MQTT_PORT`。省略令牌时仍可测试采集、报警、断网和缓存复位。

在 PlatformIO 终端执行：

```sh
pio run
pio run -t buildfs
```

运行 **Wokwi: Start Simulator**，查看仿真串口。日志速率为 115200。Wokwi 按 [flasher_args.json](wokwi/flasher_args.json) 从 `0x390000` 装载 LittleFS 镜像。测试缓存恢复时，在**同一次仿真**中按复位键，保持仿真 Flash。

应用分区各 1.75 MB，文件系统缩到 320 KB：缓存只需要约 2.3 KB，而固件已经接近旧分区的上限。修改 [partitions.csv](partitions.csv) 后必须重新执行 `pio run -t buildfs`，并同步三份 `flasher_args.json` 里的偏移。

| 元件 | GPIO | 操作 |
| --- | --- | --- |
| DHT22 | 4 | 点击传感器调整温度和湿度 |
| 电位器 | 34 | 调整 256 点模拟振动的幅度，启动时建议设到最小 |
| 报警 LED、无源蜂鸣器 | 2、25 | 查看当前状态的闪烁和鸣叫 |
| 黄色 Mode | 26 | 循环切换 POT → WARNING → FAULT → INVALID_DHT → POT |
| 蓝色 Network | 27 | 切换强制离线和自动重连 |
| 绿色 Reset | 33 | 请求软件复位，串口输入 `r` 也可触发 |

## 仿真验收步骤

1. **正常与恢复**。保持默认温湿度和电位器最小值，观察 `INIT → NORMAL`。按黄色键切到 WARNING，等三个采样帧后观察预警。再按一次切到 FAULT，等两个采样帧后观察故障。按两次回到 POT，保持安全值，等三帧进入 RECOVERY，再等四帧回到 NORMAL。也可停在 INVALID_DHT，观察无效帧计数、报警和 `valid=0`。
2. **断网与复位**。在线时按蓝键，确认 `[TEST] Forced offline`，观察 `[DATA]` 继续输出且 `[CACHE]` 的待传数量递增。等待几条记录后按绿键，观察 `[BOOT]` 编号增加和 `[STORAGE] ... recovered=`。复位后设备会自动尝试联网。
3. **FIFO 补传**。已配置 ThingsBoard 时，恢复网络后核对 `[REPLAY] PUBACK` 的 `boot/seq` 及剩余缓存数。复位前的记录应先于新启动的记录，缓存清空后出现 `[MQTT] Live PUBACK`。ThingsBoard 的 **Latest telemetry** 可查看 `record_id`、`sequence` 和 `replayed`，连续记录可在遥测历史中核对。
4. **缓存满与运行监控**。强制离线约 130 秒后，`cache` 保持在 64，`overwritten` 开始递增。`[MON]` 每五秒输出当前与最低 Heap、缓存条数与覆盖数、坏槽丢弃数、网络尝试、发送失败、补传计数、存储错误、日志丢弃和心跳超时位图。`[TASK]` 给出各任务的心跳年龄及最小剩余栈字节数，`[QUEUE]` 给出五个队列的长度、观测峰值和溢出次数。正常运行时 `blocked=0x00`、`dropped=0`。
5. **格式升级**。把固件回退到记录 32 字节的版本再前进到当前版本，第一次启动应打印 `[STORAGE] Cache v1/32 B is stale, rebuilding as v2/36 B`，随后缓存从空开始，设备照常采集与上报。

## 数据与状态

Core 1 运行 Sensor、Feature、Control、Storage，Core 0 运行 Communication、Monitor。业务帧通过 Queue 传递，GPIO 中断通过 Task Notification 传给任务，网络与停采状态通过 Event Group 共享。Sensor 使用 `vTaskDelayUntil` 维持约两秒的帧节拍，每帧包含 256 点振动。Feature 对最近五个有效温度求平均，计算去均值 RMS、峰峰值，并对去均值后的振动加 Hann 窗、执行 256 点 FFT，提取最大非直流频率桶。

日志统一走 `logPrintf`：六个任务共用一把互斥量，避免长行在多任务间被截断成两半。互斥量最多等 10 ms，拿不到就丢弃该行并计入 `[MON] logDrops=`，这样一次长串口输出不会把控制任务拖住。

| 指标 | 预警进入 | 故障进入 | 恢复安全条件 |
| --- | ---: | ---: | ---: |
| 平滑温度 | ≥ 35 °C | ≥ 45 °C | < 33 °C |
| 振动 RMS | ≥ 1.2 g | ≥ 1.8 g | < 1.0 g |
| 湿度 | ≥ 80% | — | < 75% |

INIT 需要连续两帧有效且没有预警条件的数据才进入 NORMAL。启动阶段也会直接确认持续预警或故障，避免高温或强振动启动时先报告正常。预警和故障分别需要连续三帧和两帧超限，任一传感来源连续三帧无效会预警，五帧无效会故障。连续三帧安全后进入 RECOVERY，随后再连续四帧安全才恢复 NORMAL。

报警位是一个位图，可同时表达多个问题：

| 位 | 值 | 含义 |
| --- | ---: | --- |
| 0 | 0x01 | 温度超限 |
| 1 | 0x02 | 振动超限 |
| 2 | 0x04 | 存在无效的传感来源 |
| 3 | 0x08 | MQTT 离线 |
| 4 | 0x10 | 湿度超限 |
| 5 | 0x20 | 振动来源无效 |
| 6 | 0x40 | 上一个采样间隔内有帧被队列丢弃 |

bit2 与 bit5 配合才能分清来源：只有 bit2 说明温湿度源失效，同时有 bit5 说明振动窗口失效。网络位和丢帧位不改变设备状态。

## 振动采集与频谱

默认 `esp32dev` 环境保留 Wokwi 电位器和模式键。信号模型为 25 Hz 正弦加固定种子的扰动，样本的模型时间为 `i / Fs`，生成过程可重复。它用于自动化验证，模型采样率与实际 CPU 循环速度分别定义。

两种来源共用 `Fs=200 Hz`、`N=256` 的处理窗口，窗长为 1.28 秒，FFT 频率桶间隔为 0.78125 Hz，奈奎斯特频率为 100 Hz。Hann 窗用于减小泄漏，实际相邻频率的可分辨能力还受窗主瓣影响。32 点也能执行 FFT，但可用频率桶较少，分析效果需要结合 Fs 和窗长判断。MPU6050 的 44 Hz 低通低于 100 Hz 奈奎斯特频率，因此抗混叠由器件滤波承担；若要分析高频机械特征，需要提高 ODR 并相应放宽截止频率。

`[FEATURE]` 日志输出来源、Fs、N、桶间隔、主频和振动有效性。RMS 和峰峰值使用加窗前的去均值数据，FFT 使用 Hann 窗。主频随遥测上报为 `dominantFrequencyHz` 字段，缓存与 MQTT 载荷都包含它，因此可以在云端画出主频趋势、核对缓存补传时的顺序。传感帧的 `valid` 要求 DHT 与振动来源都有效。

振动窗口的时间预算是 1.6 秒：256 点在 200 Hz 下需要 1.28 秒，剩下的余量留给 I2C 重试。轮询间隔 2 ms 小于一个采样周期，避免窗口被轮询节拍拖长到整帧周期。窗口超时、FIFO 溢出或总线错误会让整窗无效，此时遥测省略振动字段而不上报 0，避免云端把"没有数据"画成"振动为零"。

### MPU6050实体采集模式

新增 `esp32dev_mpu6050` 环境，直接使用 I2C 寄存器和硬件 FIFO。接线为 3.3 V、GND、SDA GPIO21、SCL GPIO22，AD0 接 GND，地址为 `0x68`。检查所用模块的电源规格，并确保 I2C 上拉电压为 3.3 V。

```sh
pio run -e esp32dev_mpu6050
pio run -e esp32dev_mpu6050 -t buildfs
```

驱动检查设备标识和配置回读，设为 ±4 g、200 Hz 输出、约 44 Hz 加速度低通。FIFO 保存三轴加速度包，当前取 X 轴换算为 g，并分批读取。采样间隔由器件时钟决定，轮询延时只用于让出 CPU。每个窗口重新启动 FIFO，读取完成后关闭，因而这是每两秒一次的窗口采集，窗口之间存在间隔。

I2C 错误、FIFO 溢出、超时或量程饱和会使整窗无效，随后进入状态机的无效计数流程。硬件模式下电位器和预警/故障波形模式不会替换真实测量，INVALID_DHT 模式仍可验证 DHT 故障路径。

可使用 `wokwi/mpu6050/wokwi.toml` 及同目录电路核对 I2C 集成。Wokwi 提供 MPU6050 的轴值设置，其 FIFO 时序仿真能力需要实际核实。FIFO 不工作会报告无效数据，驱动会继续尝试器件采集。真实振动验证需要实体板、传感器安装与标定，以及对实际信号带宽、滤波衰减和混叠的测量。当前低通设置用于低频验证，适用带宽与高频机械诊断需求需要另行评估。参考 [Wokwi器件说明](https://docs.wokwi.com/parts/wokwi-mpu6050)。

## 自动化测试

状态机和特征计算位于无 Arduino 依赖的 `monitor_model.h/.cpp`，固件与主机测试调用同一份实现。执行测试需要 Python 3 和 C++11 编译器，Windows 可使用已配置的 MinGW g++。

```sh
python test/run_tests.py
```

测试覆盖全部已实现的状态迁移、进入次数、回差区间、恢复确认与复发、传感器连续无效、NaN/Inf、启动严重异常，温度窗口、RMS、峰峰值、FFT 主频、来源有效性区分、报警位组合、心跳计时回绕与心跳恢复事件，以及缓存环形索引的容量、覆盖、排空与回绕不变量。

缓存索引单独放在无 Arduino 依赖的 `cache_ring.h`，因此"满环追加不覆盖 head""tail 永不等于 head""排空后可再次写满"这些不变量都有主机测试，不需要 LittleFS。测试失败时进程返回非零。主机测试验证逻辑和驱动适配行为，实体器件的采样时钟、Flash 及芯片复位结果需要运行测试。

## 看门狗与故障注入

六个应用任务进入入口前分别订阅 ESP-IDF Task Watchdog，每个任务在自己的心跳点喂狗，喂狗不受 Monitor 的诊断状态控制。TWDT 设为 15 秒，启用 panic 复位。Monitor 每约五秒检查另外五个业务任务，超过十秒未更新心跳时输出 `[HEALTH] Heartbeat late`，随后观察到心跳及时更新时输出 `[HEALTH] Heartbeat recovered` 并清除对应诊断位。一次软超时可以恢复，持续超时则由 TWDT 直接处置。Monitor 自身已订阅 TWDT，因此 Monitor 停止运行时仍有硬件定时监督。网络离线、空队列以及传感数据无效本身不会禁止正常循环喂狗。参考 [ESP-IDF看门狗说明](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32/api-reference/system/wdts.html)。

15 秒是所有订阅者共同参与的 TWDT 硬件窗口，实际触发时刻受窗口阶段和各任务喂狗周期影响。恢复日志表示软件心跳重新及时，恢复运行的任务仍须自行成功喂狗。已经触发的硬件超时或 panic 无法由后来的心跳撤销。

看门狗超时走 panic 恢复路径。它优先让系统脱离卡死状态，RAM 中未落盘的数据可能丢失，已提交的缓存在重启后重新加载。启动日志中的 `resetReason` 便于区分复位原因。卡死处置时不请求文件排空，因为存储任务自身可能已经阻塞。

panic 复位会丢掉"是哪个任务迟到"的现场。Monitor 在首次发现迟到时把位掩码写入 NVS，下次启动打印 `[BOOT] previous run had late heartbeats mask=0x..` 并清除，因此 `resetReason` 叠加该掩码就能把一次 TWDT 复位归因到具体任务。

专用测试环境才启用串口故障注入，默认固件关闭该功能。

```sh
pio run -e esp32dev_watchdog_test
pio run -e esp32dev_watchdog_test -t buildfs
```

在 VS Code 通过 **Wokwi: Select Config File** 选择 `wokwi/watchdog/wokwi.toml`，再启动仿真。该配置对应测试环境的固件和 LittleFS，并开启本机 RFC2217 端口 4000。切换回正常仿真时选择项目根目录的 `wokwi.toml`。MPU6050 集成模式则选择 `wokwi/mpu6050/wokwi.toml`。参考 [Wokwi配置说明](https://docs.wokwi.com/vscode/project-config)。

加载该固件后，串口输入 `0` 至 `5` 分别让 Sensor、Feature、Control、Communication、Storage、Monitor 停在下一个心跳点。调度器继续运行，而目标任务停止喂狗。应观察 TWDT 超时、自动复位、boot 编号增加以及缓存恢复。运行时避免连接会暂停看门狗的 JTAG/OpenOCD 调试器。

有实体串口或 Wokwi RFC2217 连接时，可运行 `python test/watchdog_smoke.py --port rfc2217://localhost:4000 --task all` 自动执行稳定运行基线和六个任务的卡死复位验收，实体串口可将地址改为实际 COM 端口。该脚本依赖 pyserial，PlatformIO 的 Python 环境已包含。仿真运行时保持 Wokwi 页签可见，以免模拟器暂停。主机测试通过与运行复位验收通过应分别记录。

## 存储与补传

NVS 为每次启动分配递增的 `bootId`，`sequence` 在每次启动时从 1 开始。MQTT 载荷中的 `boot_id` 对应启动编号，`record_id` 形如 `2-17`，`uptimeMs` 是采集时的本次启动运行时间。ThingsBoard 使用服务器接收时间入库，补传时可用 `record_id` 核对原始顺序。

LittleFS 保存 `/offline.bin` 和 `/offline.meta`。每条记录 36 字节，环形缓存最多保留 64 条，额外一个物理槽用于先写记录、再原子提交索引，缓存写满会覆盖最旧记录。元数据带版本号，记录布局变化时启动会自动重建缓存并打印 `Cache v1/32 B is stale`，不会因为格式不一致把存储判成不可用。升级时，如果旧的 `/offline.jsonl` 位于同一已挂载的文件系统，首次初始化会迁移最近 64 条可解析记录，旧文件保留，迁移记录使用 `bootId=0`。

head 槽读不出时（校验位非法或数据被破坏），Peek 会丢弃该槽并把 head 前移，而不是一直返回失败。否则一条坏记录会让缓存既发不出去也清不掉，上行停摆直到环形被追加 64 次覆盖掉它。丢弃计数出现在 `[MON] dropped=` 中。

Wi-Fi 和 MQTT 的失败重试分别采用 1 秒至 30 秒的指数退避。实时发送失败会转存，历史积压存在时新数据追加到缓存末尾。固件收到 QoS 1 PUBACK 后才删除对应的缓存记录；只有真正删除了槽位才计入 `replayed`，因此应答丢失后的重试不会把补传计数算两次。若确认后、删除前复位，同一个 `record_id` 可能重复发送。队列溢出、缓存覆盖、坏槽丢弃、存储错误和日志丢弃均在串口可见。软件复位最多等待约五秒写入已排队的遥测。

## 项目结构

```
src/                六个任务实现与应用入口
include/            对外头文件、分区常量、secrets 模板
test/               主机单测、看门狗冒烟脚本、振动驱动桩
wokwi/              三套仿真配置（默认 / MPU6050 / 看门狗）
data/               LittleFS 镜像内容
partitions.csv      双 OTA + LittleFS + coredump 分区表
platformio.ini      三个构建环境
```

`include/secrets.h` 保存本地凭据，已被 `.gitignore` 排除。首次配置复制 `include/secrets.example.h`，填写 Wi-Fi 与 Access Token。

## 许可证

[MIT](LICENSE)
