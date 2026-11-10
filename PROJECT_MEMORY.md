# V3 项目记录

## 2026-11-10 独立重写范围

用户授权新建并上传 `ht11112/liantong-robot-v3`：G474 MCU、RK ROS 桥和持续 AMCL 一起实现；单独保存，不覆盖现有程序，不烧录、不做实机测试。

参考现用 V2 第八分支提交 `eecebeababbbf4fd020108cce3214b8b20010f8e` 的硬件配置，以及树莓派附件的速度/STOP/RESUME/超时语义。旧程序记录中的 V12、进程、设备路径和地图位置不代表当前状态。开发期间仅读取源码与历史文件，没有联系机器人。

V3 采用新文本协议和持续 ROS 定位。差异、故障行为、标定步骤与离线验证分别记录在 docs/ 下。后续任何部署或实机操作均需单独授权并重新核实现场；本轮授权不包含它们。

## 2026-11-10 离线交付验证

MCU 3 套生产 C / 10 组通过，Windows Emscripten 与 WSL GCC 交叉验证；ARM BIN 31,992 字节，SHA256 `d13170b5a056d3f34c84129bae6e3ec23620e1054ca8b2d59406149780210ab6`，适用现有 32 KiB 应用区。RK 18 项 unittest、5 条生产 C ODO 到 Python 解析器交叉验证、Python 3.8 语法和 launch XML 检查通过。

当前机器无 ROS Noetic 运行环境，未运行 catkin/AMCL，不把静态检查写成 ROS 集成验收。未部署、烧录或实机测试。下一步如要联调，按 docs/CALIBRATION.md 重新确认设备与现场，不沿用旧日志中的状态。
