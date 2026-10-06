# 入站预转向与比赛耗时诊断

## 改动范围

当前执行器在 `NAVIGATE`、`DESCEND_TO_WORK` 和 `STABILIZE` 阶段，提前求下一站首段连续观察点的保序机头方向，并同时发送位置与航向指令。经过其他状态或跨站转移爬升时不预转向；没有新鲜姿态反馈时也不启用。

提前规划不启动云台、不累计扫描证据、不改变航点、目标、观察点顺序或机械限位。真正到达扫描状态后，仍重新检查反馈航向误差、角速度、实际视线可行性，并从零累计稳定时间；暂停也仍清零稳定计时。每站中间第二段转向仍须等待真实到位。状态机没有用“已经发送目标方向”代替“实际方向已稳定”。

开关位于 `config/gimbal_geometry.json` 的 `simulation_heading_alignment.prepare_heading_during_approach`；默认 `true`。设为 `false` 并重启服务即可恢复到站后转向，其他安全参数不变。该文件的策略版本同步更新，旧观察证据不能悄悄套入新版本。

## 耗时证据与边界

科一 `validation/sitl_approach_heading_report.json` 从零完成六机1278次合成观察、故障恢复和全部落地。142站均使用提前准备方向，284次保序规划、994次站位航向复用；飞控任务2555.0s（42.58min）。对比上版2607.998s减少52.998s，是一次运行差异，不是重复受控实验或稳定提升保证。

六机航向稳定等待之和从933.045s减少到529.779s，但这不是整场节省403.266s：六机并行、通道排队和返场仍在关键路径上。新运行采样最小间距16.912m，周界/真实工作区越出0，区外最低40.463m；边界检查24947次、拒绝0。仍需树林条件仿真许可，云台/目标合成，未达到科一25min。

科二搜索前段 `validation/sitl_subject2_approach_heading_report.json` 也从零完成1278次合成观察、恢复和六机落地：2576.2s（42.94min），142次入站预转向，采样最小间距16.881m，区外最低40.463m，周界/真实工作区越出0，采样边界和估计包络检查通过。与上版2599.598s单次差值23.398s，仍未达30min，未包含投放。

外部观察并发复验 `validation/concurrent_approach_heading_sitl_report.json` 通过六机独立等待、其余五机继续、单机失败原点新尝试重试、拒绝旧尝试及协调故障保持；42.71s墙钟、采样最小间距16.973m、边界检查11286次拒绝0。此有界测试结束时暂停悬停，不是全程外部视觉验收。

耗时分析工具只读取从零完成的原生报告，输出来源SHA256、每机各阶段归因、近似关键机和对照差值：

```bash
python3 scripts/analyze_mission_timing.py validation/sitl_exact_boundary_report.json validation/sitl_approach_heading_report.json --output validation/approach_heading_timing_comparison.json
```

本次近似关键机为 `uav_6`：飞行约675.1s、观察852.3s、航向/视线等待270.3s、目标事件130.6s、排队597.1s。状态累计含采样误差，最初解锁/起飞时间不完全归入阶段，不能将各机耗时简单相加预测整场。

## 后续不能靠这些方式达标

- 不能把80m假设扫描矩形改成120m就声称真实覆盖；需要镜头倍率、标定、有效帧和目标尺寸实测。
- 当前2s观察是模拟假设；附件建议稳定后3s起测。真实适配应根据实测显式配置，不应为了25min降低观察时间或有效帧数。
- 高层进出场仍采用排他通道。并行放行需重新证明整条实际路线的冲突/高度隔离及故障处置，不能直接删掉通道令牌。
- 当前每机首站10s确认、次站120s跟踪是合成事件。实机应由有效目标事件驱动，不能以定时结束替代真实跟踪终止条件，也不能删除比赛要求的目标处理时长。
- 仿真提速只能减少测试墙钟，不能缩短飞控任务时间或比赛实际耗时。

## 复验

```bash
python3 scripts/validate_waypoint_executor.py
python3 scripts/validate_ordered_heading.py
python3 scripts/sitl_server.py --speedup 20
```

另一终端运行 `python3 scripts/verify_sitl.py --authorize-forest-transit --timeout 900 --report validation/sitl_approach_heading_report.json`。完整回归会实际解锁六个本项目SITL，不用于已连接实机的服务。网页显示入站预转向站数，并把航向/视线等待单独列出，避免阶段显示漏掉这部分时间；本轮网页仅验证脚本语法，未完成浏览器视觉验收。
