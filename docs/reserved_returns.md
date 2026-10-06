# 分层预约返场

**全高度隔离纠正：**本页双科目完整报告的路线有5段穿中央隔离带，不能作为全高度禁飞验收。当前默认拒绝执行；仅显式授权隔离穿越的条件演示保留旧路线。替代端部通道候选、最新条件演示与检查见 `isolation_airspace.md`。

## 执行方案

2026-10-06复验：科一1760.501s（29.34min），科二搜索1760.601s（29.34min）。报告分别为 `validation/sitl_reserved_returns_report.json`、`validation/sitl_subject2_reserved_returns_report.json`。各从零完成1008次合成观察、故障原点恢复、并行返场暂停恢复与六机落地，均采样观察到三机实际并行返场；最小实际间距16.967m / 16.946m，采样边界/高度检查通过。科一仍超25min，科二仅搜索低于30min，不包含投放，不能据此认定科二通过。

完整巡航保持仍记录最大位移25.105m / 28.058m，恢复到位门控没有消除惯性。有界下降冲突报告 `validation/return_descent_crossing_sitl_report.json` 单独检验在垂直运动未刹停时立即请求恢复：8次保持检查拒绝重新放行，游标不变，刹停后重新预约原目的点。最小授予路径距离12.086m，保持最大采样漂移1.380m。此有界场景以悬停结束，不是完整任务或实机安全验收。

默认 `config/coordination.json` 同时设置 `entry_policy` 与 `return_policy` 为 `layered_reserved`。搜索完成的飞机先在自身安全分区保持；六机全部进入任务区后，可以申请返场，不再等前一架落地才放行。仍按本机既定层高度垂直爬升，沿周界内的既定返场通道飞至实测起飞点正上方，再垂直下降和降落。其他飞机可以继续搜索，不发送独立RTL。

每一条 RETURN_CLIMB / EGRESS / RETURN_DESCEND / LAND 指令，都由原生执行器统一申请空间预约。将本机实际位置到目标的剩余直线，与其他飞机实际位置到已授权目标的剩余直线比较；无移动授权时按悬停点比较。默认至少12m，拒绝方锁定实际悬停位置，保留原游标和目的点，待冲突消失再申请。它不是依靠错开几秒假设两机不会同时出现。

下降与LAND也包含在预约中；不能只核对水平返场。实际落地且解除锁定后，仅释放本机路径，不能清掉其他飞机的独占转场拥有者。独占转场策略仍保留，完整返场不再共享其串行令牌。

## 暂停、异常与版本

正常GUIDED暂停/协调保持清除移动授权，锁定实际位置；恢复仍先核对原航段，不跳站、不重排九点、不独立RTL。已经进入LAND的飞控不会因为网页暂停被强行切模式，其剩余落地路径仍保留，其他GUIDED机协调保持。因此“暂停”不意味着能够让正在LAND的飞机立刻停在空中。

首轮完整巡航暂停曾采样到28.13m / 14.50m位移，旧实现恢复时立即放行，没有确认飞机已经刹停。相关报告保留为 `sitl_reserved_returns_pre_hold_settlement_report.json` / `sitl_subject2_reserved_returns_pre_hold_settlement_report.json`，不能作为修正后的恢复验收。现在清除的是新移动授权；尚在制动时保留旧路径终点，不把飞行中的飞机当成无占用静止点。恢复显示 `WAIT_HOLD_STABLE`，所有GUIDED机实际返回锁存保持点（误差<0.8m、速度<0.6m/s）后才释放旧终点并重新预约原航段。这沿用航点到位判据，不是缩短时间而忽略惯性；未添加强制RTL或观察失败的独立返航。

本修正防止在尚未刹停时重新放行，不保证漂移距离已缩小；速度、加速度、通信延迟和定位误差仍需要形成实测制动包络。`validation/hold_recovery_logic_report.json` 检查高速度/未到位继续保持、游标不变、保留旧终点与落地模式不被强制改变。

网页展示正在返场的飞机、返场策略和逐机航段预约等待。`active_returns` 与 `maximum_parallel_returns` 表示已放行返场的数量，不代表全部正在移动；完整复验另记录 `maximum_sampled_parallel_return_movements`，按真实FCU反馈速度大于0.6m/s计数，才用于证明观察到实际并行运动。

返场策略纳入计划版本。切换后不能复用旧断点或旧观察证据。需要回退时，停止服务，仅将 `return_policy` 改为 `exclusive`，保留 `entry_policy=layered_reserved`，再重启重新规划。旧先进先出返场令牌仍持续到实际落地解除锁定。不得在飞行中修改策略。

## 复验命令

```bash
cd /home/zyc/zhixin_2026_ws
python3 scripts/validate_return_reservations.py
python3 scripts/validate_hold_recovery.py
python3 scripts/validate_waypoint_executor.py
python3 scripts/validate_entry_reservations.py
python3 scripts/validate_flight_safety.py
bash scripts/run_sitl.sh --speedup 20 --port 8766
```

另一个终端执行完整原生任务（树林许可只用于条件仿真）：

```bash
python3 scripts/verify_sitl.py --authorize-forest-transit --authorize-isolation-transit --verify-return-recovery \
  --timeout 600 --report validation/sitl_reserved_returns_report.json
```

科二搜索复验需要先停止服务，以 `--mission-config config/sitl_subject2.json` 重新启动，再运行相同验证器，报告改为 `validation/sitl_subject2_reserved_returns_report.json`。不要把两个科目验证器同时接到同一组飞控上。

运动学名义时间轴仍使用独占返场排程，用于保守预演；它并不模拟原生的逐航段动态授权，不应据其名义时间预测预约返场耗时。原生网页位置与完成事件来自实际飞控反馈，实际耗时以FCU报告为准。

## 验收边界

逻辑报告 `validation/return_reservation_logic_report.json` 检查非FIFO放行、入场完成前保持、下降与水平线冲突、游标不变、暂停清权、原路径重申请、落地只释放自己、独占回退和拒绝旧断点。这只是合成位置逻辑，不是飞行验收。

安全边界、最低40m（指令40.5m）、20m隔离带、同机扫描顺序均未修改。12m是剩余直线指令的几何预约距离，不是带定位误差、通信延迟和制动的连续物理保证；10m反馈保持门限也不能替代安全设计。树林条件许可、未标定相机、合成视觉和未完成科二投放等限制不变。
