# 飞艇版 ArduPlane：Mission Planner 从零配置到 ReadyToFly

本文适用于本仓库编译的 Pixhawk4 飞艇演示固件。它不是通用固定翼配置：当 `AIR_ENABLE=1` 时，Mission Planner 中显示的 `LOITER`（模式 12）被固件解释为飞艇 `POSHOLD`。

> 安全底线：所有电机、舵机方向和混控检查必须先拆除螺旋桨。只有完成卸桨检查、系留测试和低高度测试后，才能进行自由飞行。

## 1. 硬件和通道约定

从飞艇尾部向前看：

| Pixhawk4 输出 | 执行器 | `SERVOx_FUNCTION` |
|---|---|---:|
| PWM1 / MAIN1 | 左电机 | 73（ThrottleLeft） |
| PWM2 / MAIN2 | 右电机 | 74（ThrottleRight） |
| PWM3 / MAIN3 | 两电机共同倾转舵机 | 75（TiltMotorLeft，有符号角度） |
| PWM4 / MAIN4 | 上尾翼 | 77（ElevonLeft） |
| PWM5 / MAIN5 | 右尾翼 | 80（VTailRight） |
| PWM6 / MAIN6 | 下尾翼 | 78（ElevonRight） |
| PWM7 / MAIN7 | 左尾翼 | 79（VTailLeft） |
| PWM8 / MAIN8 | 未使用 | 0（Disabled） |

PWM3 的 `SERVO3_TRIM` 必须对应电机竖直向上。固件把正倾角定义为向前、负倾角定义为向后；如果实际方向相反，使用 `SERVO3_REVERSED` 修正，不能交换 `SERVO3_MIN/MAX` 来掩盖错误。

Pixhawk4 的 MAIN 输出使用普通 PWM，本文统一设置 `SERVO_RATE=50`。不要给 MAIN1～MAIN8 配置 DShot。

## 2. 启动 Ubuntu 上的 Mission Planner

已经解压的 Mission Planner 位于：

```text
/home/zyoung/桌面/飞艇/MissionPlanner-latest/app
```

在终端运行：

```bash
cd "/home/zyoung/桌面/飞艇/MissionPlanner-latest/app"
env -u GTK_EXE_PREFIX \
    -u GTK_IM_MODULE_FILE \
    -u GTK_MODULES \
    -u GTK_PATH \
    XDG_DATA_HOME="$HOME/.local/share" \
    XDG_DATA_DIRS="/usr/share/ubuntu:/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop" \
    MONO_IOMAP=drive:case \
    mono MissionPlanner.exe
```

连接 Pixhawk4 时先选择实际串口（常见为 `/dev/ttyACM0` 或 `/dev/ttyACM1`），USB 连接通常选择 `115200`。端口号可能在重新插拔后变化，应以当前系统实际枚举为准。

## 3. 刷入自定义固件

固件文件：

```text
/home/zyoung/桌面/飞艇/blimp/build/Pixhawk4/bin/arduplane.apj
```

本次基于提交 `d5d0a13b10ee7df7bd7636d961d3c03edc865cb3` 的交付文件校验值为：

```text
SHA256  63a8a2086e2cfafba3e9016e6041a730d128a0cb8600c787b8c7310373ff3387
```

刷写前可在终端运行 `sha256sum build/Pixhawk4/bin/arduplane.apj` 复核；若结果不同，先确认源码或固件是否又被重新编译，不要把来源不明的 APJ 刷入飞控。

1. 拆除螺旋桨，断开电机主动力电池，只保留飞控 USB 供电。
2. 在 Mission Planner 打开“初始设置 → 安装固件 → 加载自定义固件”。
3. 选择上述 `arduplane.apj`，等待擦除、写入和校验完成。
4. 飞控重启后重新连接，确认载具类型为 Plane。
5. 保存一份完整参数文件，文件名包含日期和“刷机前”。
6. 若参数来自普通固定翼，优先恢复默认参数后重新配置；不要直接沿用未知的混控、QuadPlane 或舵机参数。

刷机后先保持 `AIR_ENABLE=0`。完成传感器、遥控和输出配置且卸桨矩阵测试全部通过后，最后再改为 `AIR_ENABLE=1` 并重启飞控。

## 4. 传感器初始化

### 4.1 加速度计和水平面

1. 固定好 Pixhawk4，确认飞控箭头与艇首方向一致；若不一致，先正确设置板载方向。
2. 完成六面加速度计校准。
3. 将飞艇放到真实的水平飞行姿态，执行“校准水平面”。
4. 重启后确认人工地平仪滚转、俯仰方向与实物一致。

### 4.2 罗盘

1. 远离电机、电调、大电流线、钢筋和磁性工具完成罗盘校准。
2. 电机动力线固定后再做一次带动力系统的干扰检查。
3. 手动转动飞艇，确认 Mission Planner 航向同步且方向正确。
4. 航向跳变、与手机/已知方向明显不符或开电机后变化很大时，不得继续。

### 4.3 GPS、气压计和 Home

1. 到室外等待 3D Fix。
2. 确认地图位置正确、HDOP 稳定、EKF 状态为绿色。
3. 等待 Mission Planner 明确显示 Home 已建立；RTL 和双圈演示前必须再次核对 Home 图标位置。
4. 静置时高度不应快速漂移。气压计必须避开螺旋桨气流和阳光直射。

## 5. SBUS 遥控校准和排障

### 5.1 接线与校准

1. 接收机按其说明设置为 SBUS 输出，信号接 Pixhawk4 的 `RC IN`，同时连接正确电压和地线。
2. 接收机与遥控器完成绑定，关闭接收机内部会输出危险油门值的自定义 failsafe。
3. 打开“遥控器校准”，依次移动两个摇杆、三段开关、两段开关和两个旋钮。
4. 确认 8 个通道中对应条形图都连续变化，再执行校准。
5. 常规约定为 CH1 横滚、CH2 俯仰、CH3 油门、CH4 偏航；其余通道以条形图实测为准。

建议映射：

| 控件 | 默认通道 | 用途 |
|---|---:|---|
| 三段开关 | CH5 | MANUAL / FBWA / POSHOLD |
| 两段开关 | CH6 | RTL 覆盖 |
| 旋钮 1 | CH7 | MANUAL/FBWA 倾转 |
| 旋钮 2 | CH8 | 暂不使用 |

如果条形图显示的实际通道不同，后文中的 CH5、CH6、CH7必须替换为实测通道，不能只按遥控器面板名称猜测。

### 5.2 遥控完全没有反应时

按以下顺序停止并检查：

1. 接收机指示灯是否显示已经绑定，而不是仅通电。
2. 接收机是否真的输出 SBUS，而不是 PWM、iBUS、CRSF 或关闭输出。
3. 信号是否接到 `RC IN`，地线是否共地，插头方向是否正确。
4. 遥控器模型是否启用了相应 8 个通道，通道监视器中是否有动作。
5. 断电后重新插接，再观察 Mission Planner 的 Radio Calibration 页面。
6. 仍无输入时，用接收机说明书确认 SBUS 是否需要专用端口或反相器；不要靠改变 ArduPlane 混控参数解决物理输入问题。

只要 CH1～CH8 任一所需控件没有稳定输入，就不能进入输出和电机测试。

## 6. 基础参数

在“完整参数树/完整参数列表”中设置并写入：

```text
Q_ENABLE         = 0
ARMING_REQUIRE   = 1
ARMING_CHECK     = 1
SERVO_RATE       = 50
MIXING_GAIN      = 0.5
MIXING_OFFSET    = 0
RUDD_DT_GAIN     = 5
MANUAL_RCMASK    = 0
OVERRIDE_CHAN    = 0
ROLL_LIMIT_DEG   = 8
WP_RADIUS        = 10
WP_MAX_RADIUS    = 10
THR_MIN          = 0
THR_MAX          = 100
THR_SLEWRATE     = 20
USE_REV_THRUST   = 0
```

如果 `Q_ENABLE` 曾经开启，写入 0 后必须重启。不要配置 QLOITER、QRTL 或 QuadPlane 倾转参数；这套飞艇控制不使用 QuadPlane。双电机只允许正推力，`THR_MIN` 必须精确为 0（正数也不允许，否则 HOLD 的零油门会被抬高），`USE_REV_THRUST` 必须为 0，`THR_MAX` 必须大于 0。

`ARMING_REQUIRE` 只允许 1（未解锁输出最小 PWM）或 2（未解锁输出零 PWM），本文使用 1；不得设成 0 或自动解锁类型。`ARMING_CHECK=1` 保留全部常规检查。飞艇固件还把关键输出、模式和参数检查设成不可由 `ARMING_CHECK=0` 或 force-arm 绕过，但这不构成关闭其他飞控检查的理由。

`MANUAL_RCMASK` 和 Pixhawk4 的 `OVERRIDE_CHAN` 必须为 0，避免 RC 直通或 IO 固定翼混控覆盖 PWM1～7。飞艇启用时固件会关闭 Pixhawk4 IO 协处理器的标准固定翼应急混控；FMU 停止工作时，MAIN1/2 使用电机最小 PWM，MAIN3～7 使用各自 `SERVOx_TRIM`，此时不能继续人工操纵。必须把它视为失效后的固定安全输出，而不是后备飞行模式。

### 6.1 飞艇参数

初始值：

```text
AIR_ENABLE       = 0
AIR_R_IN         = 10
AIR_R_OUT        = 35
AIR_ALT_BAND     = 20
AIR_RET_SPD      = 2.5
AIR_RET_THR      = 35
AIR_CLIMB_THR    = 25
AIR_TILT_FMAX    = 90
AIR_TILT_BMAX    = 45
AIR_TILT_RATE    = 10
AIR_YAW_P        = 1.0
AIR_YAW_MAX      = 50
AIR_MAN_CH       = 7
```

`AIR_ENABLE` 在每次启动时锁存。无论从 0 改成 1，还是从 1 改成 0，都必须重启；未重启时控制链保持启动时的状态，并显示 `reboot after AIR_ENABLE change`、拒绝解锁。飞行中禁止修改任何 `AIR_*`、`SERVO*`、`THR_*`、混控或解锁参数。

其中以下数值不能未经实测直接用于自由飞行：

- `AIR_TILT_FMAX/BMAX`：分别填写 PWM3 安全端点对应的真实前、后机械角度。
- `AIR_CLIMB_THR`：通过系留试验找到可以缓慢爬升、又不会猛升的最低可靠油门。
- `AIR_RET_THR`：通过系留或低高度直线试验确认能克服演示现场风速。
- `AIR_RET_SPD`：不得超过已验证的稳定低速飞行范围。

始终保持 `AIR_R_IN < AIR_R_OUT`。10/35 m 是演示初值，外圈之外还必须留出制动、人工接管和地理围栏余量。

## 7. 输出功能和倾转标定（必须卸桨）

写入：

```text
SERVO1_FUNCTION  = 73
SERVO2_FUNCTION  = 74
SERVO3_FUNCTION  = 75
SERVO4_FUNCTION  = 77
SERVO5_FUNCTION  = 80
SERVO6_FUNCTION  = 78
SERVO7_FUNCTION  = 79
SERVO8_FUNCTION  = 0
```

### 7.1 PWM3

1. 用 Mission Planner 舵机输出测试小幅移动 PWM3。
2. 设置 `SERVO3_TRIM`，使两个电机严格竖直向上。
3. 缓慢找到不会顶死机构的安全 `SERVO3_MIN` 和 `SERVO3_MAX`。
4. 测量从竖直到前极限、后极限的实际角度，分别写入 `AIR_TILT_FMAX/BMAX`。
5. 若正命令没有向前，切换 `SERVO3_REVERSED`。
6. 确认未解锁/解除锁定后、模式切换和重启时都回到竖直中位。回中同样受 `AIR_TILT_RATE` 限制；例如从 90° 以 10°/s 回中约需 9 秒，但未解锁状态下两电机保持零油门。

禁止让舵机在端点持续嗡鸣或让连杆越过机械死点。

### 7.2 四尾翼矩阵

开启 `AIR_ENABLE=1` 并重启，仍然保持卸桨。实际气动力方向由安装方式决定，因此以下检查关注“产生的飞艇力矩”，而不只是舵面视觉方向：

| 指令 | 上 PWM4 | 右 PWM5 | 下 PWM6 | 左 PWM7 | 期望力矩 |
|---|---|---|---|---|---|
| 抬头 | 与下翼同向作用 | 基本不动 | 与上翼同向作用 | 基本不动 | 艇首上仰 |
| 低头 | 抬头的反向 | 基本不动 | 抬头的反向 | 基本不动 | 艇首下俯 |
| 右偏航 | 基本不动 | 与左翼同向作用 | 基本不动 | 与右翼同向作用 | 艇首向右 |
| 左偏航 | 基本不动 | 右偏航的反向 | 基本不动 | 右偏航的反向 | 艇首向左 |
| 右滚 | 上下翼产生相反滚转作用 | 左右翼产生相反滚转作用 | 同左 | 同右 | 艇体右滚 |

逐个使用 `SERVO4_REVERSED`～`SERVO7_REVERSED` 修正。修正一种动作后必须重新检查全部五种动作，因为单个舵机同时参与两个轴。

## 8. 电机、电调和差速检查（先卸桨）

1. 根据电调说明完成行程设置；两个电调必须具有相同的起转点。
2. 短时低油门测试 PWM1 左电机、PWM2 右电机，确认没有互换。
3. 断电后安装方向标记，再确认两副螺旋桨安装方向和旋向产生向上/向前的正确推力。
4. PWM3 竖直时给偏航指令，左右电机不应出现明显差速。
5. PWM3 向前时给偏航指令，应出现很小的辅助差速；确认该差速帮助尾翼偏航，而不是反向。
6. PWM3 向后时，相同偏航指令的差速会自动反号，以保持偏航力矩方向。
7. 任一电机不能可靠同步起转、振动异常或电流明显不一致，停止测试。

电机测试完成后先断电，再进行任何接线或安装螺旋桨操作。

## 9. 飞行模式和 failsafe

设置实际三段开关通道，例如：

```text
FLTMODE_CH = 5
FLTMODE1   = 0     # MANUAL
FLTMODE2   = 0
FLTMODE3   = 5     # FBWA
FLTMODE4   = 5
FLTMODE5   = 12    # LOITER，在本固件中是 POSHOLD
FLTMODE6   = 12
```

三段开关常见的 1000/1500/2000 μs 会落入第 1、4、6 档；相邻档设置成相同模式可防止端点偏差造成意外模式。

两段 RTL 开关假设为 CH6：

```text
RC6_OPTION = 4
```

拨高进入 RTL，拨低返回三段开关当前选择的模式。逐档观察 HUD 模式文字确认，不能只看遥控器开关位置。

飞艇功能开启后，只支持 MANUAL、FBWA、LOITER/POSHOLD、RTL 和 AUTO，并且只允许在 MANUAL 或 FBWA 中解锁，避免在地面直接以 RTL/AUTO 的自动油门起动。未解锁时请求其他固定翼/Q 模式会被拒绝；已解锁时请求其他模式会告警并转入自定义 RTL。起飞前应把三段开关置于 MANUAL、RTL 开关置低；AUTO/POSHOLD/RTL 都只在人工起飞后切入。

RC failsafe：

```text
THR_FAILSAFE    = 1
THR_FS_VALUE    = 按接收机实际失控输出设置
FS_SHORT_ACTN   = 2
FS_LONG_ACTN    = 1
FS_LONG_TIMEOUT = 5
```

用卸桨状态关闭遥控器验证：应先进入 FBWA/零油门保护，持续约 5 秒后进入自定义 RTL。恢复遥控后再检查模式行为。

演示时建议 `FS_GCS_ENABL=0`，避免 Ubuntu/Mono 地面站短暂掉线触发模式改变；安全链路以 RC 为主。

## 10. 电池监测和保护

1. 选择正确的电源模块类型。
2. 用万用表校准显示电压，用已知电流或充电器数据校准电流。
3. 正确填写电池容量。
4. 按实际电池化学体系、串数和厂商允许值计算总包阈值：

```text
BATT_LOW_VOLT = 单节低电压阈值 × 串数
BATT_CRT_VOLT = 单节临界阈值 × 串数
```

`BATT_CRT_VOLT` 必须低于 `BATT_LOW_VOLT`，但高于电调、舵机和飞控会失效的电压。建议同时按实际容量设置 `BATT_LOW_MAH/BATT_CRT_MAH`，不要照抄其他电池的数值。

动作设置：

```text
BATT_FS_LOW_ACT = 1   # RTL
BATT_FS_CRT_ACT = 1   # RTL
BATT_LOW_TIMER  = 10
```

本演示版没有自动着陆控制，因此不要把 Plane 的 `Land` 动作误认为飞艇原地缓降。

## 11. 航点能力配置

首版 AUTO 仅允许普通 Waypoint：

1. 手动起飞并用 FBWA 建立稳定速度。
2. 任务中每一项都只能是普通 `MAV_CMD_NAV_WAYPOINT`；不能包含 TAKEOFF、LAND、LOITER、SPLINE、DO、CONDITION、相机、继电器或舵机指令。固件会在进入 AUTO 及任务在线变化时检查整份任务，发现其他指令就告警并转入自定义 RTL。
3. 各航点使用相对 Home 的高度，不使用 Terrain 高度；控制器允许约 ±20 m 高度误差。
4. `WP_RADIUS=10`，飞艇只有进入航点 10 m 水平半径才推进下一点。
5. 上传后重新下载任务逐点核对。
6. 从 Mission Planner 模式下拉框切 AUTO；三段开关仍保留 MANUAL/FBWA/POSHOLD，RTL 开关随时可用。
7. 任务结束会进入以 Home 为圆心的双圈 RTL。

第一次 AUTO 测试只使用一个位于上风侧、距离较近且空域完全可控的航点。

## 12. 分阶段实机验证

必须按顺序完成；任何一级不通过都不得进入下一级。

### A. 卸桨台架

- 传感器方向、RC 八通道、模式开关全部正确。
- PWM3 中位、端点、方向和斜率正确。
- 四尾翼五种指令矩阵全部正确。
- 两电机通道、起转点和差速方向正确。
- 上锁后电机为零、倾转竖直。
- 关闭遥控器后的 failsafe 符合预期。

### B. 系留竖直推力

- 在足够软且开阔的场地安装螺旋桨并建立多点系留。
- 测得刚好抵消负浮力和缓慢爬升所需油门，修正 `AIR_CLIMB_THR`。
- 检查电机气流不会让尾翼或艇体异常振动。
- 确认动力开启不会导致罗盘航向大幅变化。

### C. 系留/低高度前后倾转

- 从竖直以不超过 `AIR_TILT_RATE` 的速度逐步向前。
- 验证艇首方向、偏航修正和向后制动方向。
- 修正 `AIR_RET_THR`、`AIR_RET_SPD`，记录实际停止距离。

### D. 低高度自由飞行

- 首次只测 MANUAL 和 FBWA。
- 再测 POSHOLD 捕获位置和低能耗漂移。
- 最后从 Home 外侧切 RTL，确认距离持续减小且进入 10 m 后停止水平推进。
- AUTO 航点必须在上述测试全部通过后单独进行。

## 13. ReadyToFly 清单

只有每项均为“是”才能进入演示飞行：

- [ ] 固件文件和参数备份可恢复，`AIR_ENABLE=1` 后已重启。
- [ ] `ARMING_REQUIRE=1`、`ARMING_CHECK=1`、`OVERRIDE_CHAN=0`、`MANUAL_RCMASK=0`。
- [ ] 螺旋桨、电机、PWM1/2 左右标识和旋向正确。
- [ ] PWM3 中位竖直，前后方向和机械限位实测完成。
- [ ] 四尾翼混控矩阵全部通过。
- [ ] RC 八通道、MANUAL/FBWA/POSHOLD 和独立 RTL 开关全部通过。
- [ ] RC failsafe 和电池 failsafe 已卸桨验证。
- [ ] 电池电压、电流、容量及低/临界阈值正确。
- [ ] GPS 3D Fix、EKF 绿色、罗盘稳定，Home 位于真实起飞区。
- [ ] `AIR_R_IN=10`、`AIR_R_OUT=35`，围栏和场地在外圈外仍有充分余量。
- [ ] 当日持续风和阵风低于已经实测的返航能力。
- [ ] 系留爬升、前飞、偏航和向后制动均已通过。
- [ ] 飞手能立即切 FBWA，RTL 开关位置无需低头寻找。
- [ ] 现场人员隔离、观察员、灭火与急救措施就位。

满足这些条件只表示系统完成演示前准备，不代表已经适合大风、远距离或无人值守飞行。
