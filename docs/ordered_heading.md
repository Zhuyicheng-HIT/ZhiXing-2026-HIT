# 保序航向前瞻：使用与验证

## 本次变更

依据用户提供的《搜救无人机_扫描与ROS对接规格_v0.1.md》，观察点与执行顺序由任务表确定，不允许自动重排。因此保持142个飞行站位、每站九点的原顺序、1278个观察命令及80m假设矩形不变，只改变同站位的机头航向选择。

`plan_ordered_hover_heading` 选择能服务最长连续观察点前缀的名义水平航向，同长度时选择当前航向附近转角最小的候选。使用1°候选步长及原有5°yaw余量。这是离散局部策略，不声称全局最优。执行器锁存站位航向；后续点在仍可达时复用，仍逐点验证新鲜实际姿态、机体航向误差≤3°、角速度≤5°/s、连续稳定0.5s及名义视线误差≤2°。没有删掉到位等待、缩短2s观察或扩大覆盖。

暂停、故障不清除站位航向和原命令；恢复后重新积累稳定时间，不跳点、不独立RTL。检查点新增站位航向及前缀剩余长度，但原生SITL仍禁止未经核验的服务重启存档恢复。配置策略版本已更新，进入逐点内容版本校验。

## 验证结果

| 证据 | 科一 | 科二搜索前段 |
| --- | ---: | ---: |
| 报告 | `sitl_ordered_heading_report.json` | `sitl_subject2_ordered_heading_report.json` |
| 从零运行、六机解锁、恢复、全部落地解除锁定 | 通过 | 通过 |
| 原序合成观察 / 可行名义视线 | 1278 / 1278 | 1278 / 1278 |
| 站位航向规划 / 后续点复用 | 284 / 994 | 284 / 994 |
| 飞控实际任务时长 | 2583.000s / 43.05min | 2558.002s / 42.63min |
| 比赛预算 | 25min，未通过 | 30min，未通过 |
| 采样最小三维间距 | 16.874m | 16.887m |
| 最大计算候选视线误差 | 0.105744° | 0.076203° |
| 位置审计样本数 | 38280 | 38550 |
| 起降区外最低模拟高度 | 39.962m | 39.956m |

报告位于 `validation/`。独立名义规划对照 `ordered_heading_report.json` 验证整个任务表未被修改、1278点原序可达，转向次数由逐点贪心580降为284。实际运行的284是航向规划次数，不是经过物理yaw测量得到的转向次数；994是后续点复用次数。单次前后运行的耗时差异不能当作稳定性能提升证明，也不能用更少的转向次数推断比赛达标。

本次外部接口冒烟另见 `gimbal_http_ordered_heading_report.json`：原生飞控悬停、故障后原命令恢复、不可观测结果保留断点、拒绝过期版本、重复回传幂等通过；六个模拟源帧使一个观察点完成一次。它没有使用真实视频、YOLO或ZR10。Windows及WSL的规划、执行器和扫描协议逻辑验证通过；网页截图和动态状态快照在 `ordered_heading_ui.png` / `ordered_heading_ui_dom.txt`。

两次完整运行都是六个官方ArduCopter4.7.1原生内部quad模型，不是700mm动力组标定模型。采样周界和带4m缓冲工作约束区的超出距离均为0；不证明精确分区边界严格不越界或连续轨迹绝无碰撞。最低高度略低于40m，不是严格高度合规。全部视线是名义几何候选，不是ZR10机械反馈或YOLO验收。全程使用明确勾选的条件仿真树林过境授权，不能代替主办方许可。科二没有投放、补给或三段舵机实测。

## 启动与查看

```bash
cd /home/zyc/zhixin_2026_ws
bash scripts/run_sitl.sh --speedup 5 --port 8766
```

浏览器访问 `http://127.0.0.1:8766`，等六机READY且有定位反馈。确认仿真树林过境假设后点击“启动 / 继续仿真”。网页显示实际FCU位置、航向到位状态、全组前瞻规划/复用计数及各点是否复用站位航向。暂停和故障恢复可检查原点是否继续。不要同时启动另一个占用8766端口的服务。

科二搜索前段需先停止本项目服务，然后使用：

```bash
bash scripts/run_sitl.sh --speedup 5 --port 8766 --mission-config config/sitl_subject2.json
```

原生SITL中网页科目/起点重规划被禁用，避免飞行中改变坐标系或任务。运动学预演可独立使用 `python3 scripts/sim_server.py`，不应与六机飞控闭环混淆。

## 复验

```bash
python3 scripts/validate_ordered_heading.py
python3 scripts/validate_waypoint_executor.py
python3 scripts/validate_scan_protocol.py
python3 scripts/validate_feasible_pointing.py
python3 scripts/verify_sitl.py --timeout 900 --authorize-forest-transit --report validation/new_ordered_heading_run.json
```

最后一条会启动六机仿真飞行；只对本项目的全新READY服务执行。20倍物理加速仅缩短墙钟等待，报告中的FCU秒仍按任务实际模拟时间判断比赛预算。下一步必须解决任务耗时和通道权限，并接入真实图像/云台反馈；不能直接将当前计划上传实机。
