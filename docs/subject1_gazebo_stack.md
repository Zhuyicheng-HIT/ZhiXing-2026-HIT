# 科目一 Gazebo 六机仿真

## WSL 启动

在 WSL Ubuntu 中执行：

```bash
cd /home/zyc/zhixin_2026_ws
bash scripts/run_subject1_stack.sh
```

脚本会依次启动：

1. `fleet_kinematic.sdf` 六机 Gazebo 世界；
2. 六路 `/uav/uav_N/gimbal/image_raw` 图像采集桥；
3. 规划器和状态机网页服务。

## Windows 查看

在 Windows 浏览器打开：

```text
http://localhost:8765
```

网页的六个画面优先读取 `/api/camera/frame?vehicle_id=uav_N` 的真实 Gazebo JPEG；图像桥尚未产出帧时，才显示合成预览。

## YOLO 与反投影

先启动图像桥和网页，再另开 WSL 终端：

```bash
cd /home/zyc/zhixin_2026_ws
python3 scripts/gazebo_perception_loop.py \
  --state-url http://127.0.0.1:8765/api/state \
  --weights /path/to/best.pt
```

该循环使用 ZR-10 的 61.5° 水平视场、16:9 图像模型，把检测框中心反投影到 `mission_map` ENU，并通过 `ZR10Bridge` 计算云台跟踪指令。没有提供权重时，仍可用 `--once` 验证六机图像、状态和输出文件链路。

## 真实 ZR-10

真实设备使用时额外传入：

```bash
python3 scripts/gazebo_perception_loop.py \
  --real-zr10 --zr10-host 192.168.144.25 \
  --weights /path/to/best.pt
```

真实设备必须先完成 SDK 路径、网络、云台限位和相机标定确认；仿真产生的 `SIMULATED_OBSERVATION` 不能作为实机识别证据。

## 当前边界

本启动器已经是 Gazebo 相机—图像桥—网页—感知反投影链路，但当前 WSL 没有 `arducopter`、`sim_vehicle.py` 和 ROS 2，因此还不是 ArduPilot 飞控物理闭环。安装这些依赖后，入口可继续接到同一状态机和 ZR-10 桥，不需要改任务规划格式。
