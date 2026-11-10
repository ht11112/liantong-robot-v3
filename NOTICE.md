# 来源说明

- G474 的引脚、时钟、FDCAN/RS485 硬件配置与双轮 CAN 对象字典参考用户现有 `liantong-robot-v2` 的 `feature/rk-feedback-worker`，参考提交 `eecebeababbbf4fd020108cce3214b8b20010f8e`。本仓独立重写控制与文本协议，不部署到该版本的运行目录。
- 速度命令、STOP/RESUME 和命令超时语义参考用户提供的“树莓派3B自动导航小车项目总结”附件。未打包或发布整个第三方附件；输入、计量和故障处理差异见 `docs/BEHAVIOR_CHANGES.md`。
- ARM 构建使用外部 STM32Cube G4 HAL/CMSIS SDK。本仓不包含 SDK，相关许可由安装的 SDK 提供。
- ROS1 Noetic、AMCL、map_server、move_base 作为系统依赖使用，保留其各自许可。本仓不复制这些项目的实现。
- 仓库内标称轮径、轮距、雷达外参和参数不是本版本实测标定值。私人地图、现场记录、凭据和固件备份不属于上传内容。
