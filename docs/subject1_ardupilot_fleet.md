# 科目一六机 Gazebo/ArduPilot 联合仿真

## 已实现

`scripts/build_ardupilot_fleet_world.py` 从当前任务规划读取六架无人机起飞位置，复制 `iris_with_gimbal` 模型，并为每架机生成独立模型名与 FDM 端口：

- `uav_1` 至 `uav_6`
- `9002` 至 `9007`
- 每架机包含 ArduPilot 插件、IMU、NavSat 和 ZR-10 外形云台相机
- 相机话题为 `/uav/uav_N/gimbal/image_raw`

## 启动

在 WSL Ubuntu 中：

```bash
cd /home/zyc/zhixin_2026_ws
export AP_ROOT=$HOME/ardupilot
export AP_GZ_ROOT=$HOME/ardupilot_gazebo
./scripts/run_subject1_ardupilot_fleet.sh
```

启动器默认要求 ArduPilot 源码版本包含 `4.7.1`。当前环境若仍是其他版本，只能用下面的方式做临时进程/话题验证，不能作为比赛实机参数验收：

```bash
ALLOW_AP_VERSION_MISMATCH=1 ./scripts/run_subject1_ardupilot_fleet.sh
```

## 当前验收边界

已验证六机 Gazebo 世界可加载，并产生六个独立的云台相机话题。规划器、状态机、入口释放、返场预约和断点恢复使用当前确定性运动学仿真验证通过。

尚未宣称完成的部分：六个 ArduPilot 实例的 MAVLink 遥测闭环、JSON 控制后的真实位姿变化、Gazebo 位姿回灌网页、ZR-10 实机云台控制、YOLO 实际视频识别和 RTK/LoRa 实机链路。这些必须在 4.7.1 版本匹配后逐架确认，再扩大到六机。
