# 离线验证记录

日期：2026-11-10。以下均为本机软件验证，未连接机器人、未烧录、未在 RK 部署或执行实机测试。

## 已完成

| 项目 | 结果 |
| --- | --- |
| MCU 生产 C 测试 | 3 套、10 组通过；Windows Emscripten 与 WSL 原生 GCC 均验证 |
| G474 生产构建 | ARM GCC + STM32CubeG4 1.6.3，32 KiB 链接区，BIN 31,992 B，余量 776 B |
| RK 生产核心测试 | 18 项 Python unittest 通过，串口读取以本机 socketpair 字节传输验证 |
| 两端协议交叉 | 5 条真实 C 格式化器输出由 RK 生产解析器读取；验证启动无效样本、正常样本、重复、断段故障和锁存恢复反馈 |
| 时间/会话交叉 | 重复不发布、首次保守锚点、断段结束 TF 会话均符合双端约定 |
| 兼容与配置 | 8 个 Python 文件按 Python 3.8 语法解析；package.xml 和 2 个 launch XML 解析通过 |
| 独立审查 | 复核 CAN 对象字典、使能/停止路径、UART 错误重收、TF 发布者、无效激光处理和默认不发运动命令 |

生产 BIN SHA256：

```text
d13170b5a056d3f34c84129bae6e3ec23620e1054ca8b2d59406149780210ab6
```

G474 使用 Cortex-M4F 硬浮点 ABI 与 LTO。核对 ELF 启动路径为 SystemInit 开启 FPU、设置 VTOR 后进入 main。静态 RAM 为 1,616 B，链接脚本另保留 4 KiB 栈。链接使用 bare-metal nosys，构建输出的宿主系统调用桩警告不构成实物运行验证。

生产 C 测试直接编译控制、CAN 驱动和协议；UART 测试从板级源提取真实轮询/回调，硬件替身只提供字节和 HAL 时钟。RK 测试调用实际生产解析、计时、扫描裁剪和命令缓存。它们不能证明 CAN 或 RS485 的电气时序。

## 尚未执行

- ROS Noetic 环境下的 catkin 构建、roslaunch 启动、AMCL 实际收敛及 TF 查询。当前开发机没有 ROS Noetic 运行环境。
- RK 实串口、DE 转向延迟、UART 抗误码、CAN 周期与故障恢复实测。
- 轮径/轮距/雷达外参标定、地图比例验证、真实停止时间与距离、连续导航性能。

## 复现与本地证据

从仓库根目录执行：

```text
python -B mcu/build.py --host-tests --firmware --name offline-final
python -B scripts/check_offline.py --mcu-receipt build/offline-final/receipt.json
```

工具路径可按 `mcu/README.md` 指定。目录名已存在时换新名称，不覆盖旧收据。首次交付的本地证据位于 `build/mcu-v3-final-candidate/`、`build/native-final/`、`build/v3-independent-final-review-20261110/` 和 `build/offline_validation.json`，均被 Git 忽略。上传的是源码、测试及本说明，未上传私人地图、现场采集、工具 SDK、固件备份或凭据。
