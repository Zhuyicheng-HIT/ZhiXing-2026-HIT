# 科目一 ArduPilot/Gazebo 联调边界

## 当前可验证内容

- `fleet_kinematic.sdf`：六架任务机、六路Gazebo RGB图像话题和网页动态预览。
- `run_subject1_stack.sh`：科目一任务状态机、路线演示、相机桥和HTTP网页。
- `run_subject1_ardupilot_gazebo.sh`：单机ArduPilot JSON插件冒烟测试入口。

## 单机插件冒烟测试

```bash
cd /home/zyc/zhixin_2026_ws
ALLOW_AP_VERSION_MISMATCH=1 \
  AP_ROOT=/home/zyc/ardupilot \
  AP_GZ_ROOT=/home/zyc/ardupilot_gazebo \
  bash scripts/run_subject1_ardupilot_gazebo.sh
```

脚本默认要求ArduPilot源码版本包含`4.7.1`。当前WSL发现的源码版本是`ArduPilot-4.6.0-beta1-7195-gf9d619e260`，因此只有显式设置`ALLOW_AP_VERSION_MISMATCH=1`才会运行。该选项只用于验证插件能否拉起，不能作为比赛固件或实机闭环结论。

日志位于`runtime/ardupilot/`。目前单机测试已确认Gazebo和ArduCopter进程可以启动；JSON遥测、姿态控制和六机任务接管仍需在目标4.7.1源码上继续验证。

## 六机网页仿真

```bash
cd /home/zyc/zhixin_2026_ws
bash scripts/run_subject1_stack.sh
```

Windows浏览器打开`http://localhost:8765`；如果端口被占用，可设置`PORT=8766`。网页中的六路画面优先读取`/api/camera/frame?vehicle_id=uav_N`返回的Gazebo JPEG，没有图像时才回退为占位画布。
