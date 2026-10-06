# 云台指向与机体姿态

`scripts/gimbal_geometry.py` 是可复用的纯 Python 几何模块，不发送 SIYI 命令。接口遵循用户提供的《搜救无人机_扫描与ROS对接规格_v0.1.md》：共享 `mission_map` 为 ENU，机体 `base_link` 为 FLU，四元数把子系向量变换到父系，规范角以 rad 表示，UI 度数显式带 `_deg`。

## 飞控边界

SITL 请求并接收 MAVLink `ATTITUDE`，保存原始 NED/FRD 欧拉角及 `time_boot_ms`。转换先处理 NED/FRD 到当地 ENU/FLU，再利用各机实际 EKF 原点和任务原点的 ECEF 旋转矩阵对齐切平面，输出 `quaternion_mission_from_body_flu_xyzw`。不能只用 `90°-yaw` 并忽略 roll、pitch 或各机坐标基准。

`telemetry.attitude` 包含父/子坐标系、FCU 时间和本机接收时间。没有有效 EKF 原点、姿态缺失或接收过旧时，指向显示无效，不拿零姿态充当反馈。姿态接收时间不是图像曝光时间；真实视觉定位仍需同步/插值，当前没有证明该同步能力。

## 指向计算

`solve_pointing(target_enu_m, body_position_enu_m, mission_from_body_xyzw, configuration)`：

1. 将安装平移从 FLU 旋转到 ENU，得到名义云台旋转参考点。
2. 目标减参考点得到视线，再逆旋转到机体和云台安装 FLU。
3. 输出规范化视线、`atan2(y,x)` 方位及正向上的仰角。
4. 正下视时保持指定方位，不用接近零的水平量制造方位跳变；目标与参考点重合时报无效。
5. 检查名义机械范围，不静默裁剪后宣称到位。

这些角是 **云台安装系的规范方位/仰角**，不是 SIYI 未核实的原始 yaw/pitch。`sdk_command_available=false` 始终保留。

## 当前未标定内容

`config/gimbal_geometry.json` 为零平移、单位安装四元数，明确 `calibrated=false`、`sdk_axis_mapping_verified=false`。它只是仿真参考，未把 GPS 天线、机体参考点、云台转轴和相机光心实测对齐，也未建模云台旋转导致的相机光心偏移。实机必须提供外参、模式和 SDK 正负方向测试，不能直接将 UI 值发给 ZR10。

安装配置哈希纳入逐点观察版本与断点配置校验；修改后需重新规划/启动，不能复用旧观察证据。

## 新发现的扫描可达性问题

当前九点矩形几何在**机头固定正北、名义水平姿态**下，1278 个观察点中有 142 个超出名义 ±135° yaw 范围，即每站一个后向点。名义范围取自用户接口草案列出的 SDK 保守范围，尚未做本机机械验收。对应报告是 `validation/gimbal_geometry_report.json`。

现在增加显式 `SCAN_ALIGN`：保持扫描站位，按名义水平悬停姿态选取满足 yaw 限位且保留 5° 余量的最小转向；每次候选增量为 1°。MAVLink 位置指令同时发送航向，航向通过各机实际 EKF 原点转到 LOCAL_NED，而非直接复用 ENU 数值。收到新鲜实际姿态后，机头误差不超过 3°、机体角速度模长不超过 5°/s，连续保持至少 0.5 秒，才开始观察计时或发出外部观察命令。暂停/故障保持后重新累计稳定时间，不独立 RTL。以上数值均为仿真假设，可配置，不是实机验收阈值。

`validation/heading_planner_report.json` 验证 1278 个点在**名义水平姿态**下均存在可达航向。实际 roll/pitch、位置误差仍可能使精确安装系指向超限，执行器保留 `actual_pointing_outside_nominal_limits`，不隐藏该计数。真实云台闭环稳定反馈、安装外参和 SDK 方向仍待接入。

## 限位内可执行视线

`feasible_pointing` 保留目标的精确单位视线，求名义两轴限位内与目标视线点积最大的可执行方向，而非将yaw裁剪后直接宣称到位。规范elevation限定在−90°到90°，对可行yaw的最优值只需检查目标方位及yaw边界；再检查最优elevation及其边界。用叉积模长和点积的 `atan2` 计算真实球面角误差，避免近零误差时 `acos` 的数值问题。

精确yaw在接近正下视时可越界，但一个yaw范围内的名义指向仍可能具有很小的真实视线误差。`pointing.executable` 明确给出候选角、误差、限位状态和可行性，不修改 `pointing.azimuth_deg` 的精确结果。可行阈值沿用 `scan_profile.json` 的2°，没有扩大扫描范围或降低阈值。执行器要求新鲜实际姿态下候选可行，并连续通过机体稳定检查，才开始合成计时或发出外部命令；不可行进入 `SCAN_WAIT_POINTING`，保留断点，不自动完成或RTL。

40个历史超限实例回放均可在名义两轴范围内指向，最大角误差0.10647°，见 `validation/feasible_pointing_report.json`。该证据是历史视线样本的算法回放，不是全程、曝光反馈或真实机械验收。命令 `pointing_at_issue` 仍仅是发出时刻的快照，候选角不能直接当成经过核实的SIYI SDK值。

此后完整六机飞控复验记录在 `validation/sitl_feasible_pointing_report.json` 与 `validation/sitl_subject2_feasible_pointing_report.json`，科一及科二搜索前段均完成1278次合成观察、故障恢复、全部落地解除锁定。每次都通过名义候选可行门控，最大计算视线误差分别为0.10024°/0.11487°；实际任务43.33/44.06分钟仍超时。它们证明执行器不再绕过名义指向不可行状态，不证明实际云台执行到位或比赛全流程完成。

## 验证

### 保序航向前瞻

用户提供的ROS扫描规格明确要求按给定点位顺序执行，因此不重排九点，不修改站位、覆盖矩形、目标坐标或命令ID。`plan_ordered_hover_heading` 在当前航向及正负1°至180°候选中，选择能服务最长连续点位前缀的航向；同长度时选择转动最小的候选。沿用5°名义yaw余量，同一站位复用选定航向，每一点仍重新验证实际姿态、角速度、连续稳定时间和候选视线误差。失效时保留原点，不跳点、不独立RTL。

站位航向和剩余前缀长度写入检查点，暂停不清除；这是当前进程内恢复和检查点记录，并不声称服务重启恢复已实现。策略修订进入几何配置哈希，避免新旧指令版本混用。`validation/ordered_heading_report.json` 的名义水平姿态对照中，1278个原序观察点全部可达，转向次数从逐点贪心的580降为284；该对照本身不证明实际飞行耗时改善，也不证明真实云台机械限位或到位反馈。

六帧配置及带指向快照的原生悬停接口冒烟记录为 `validation/gimbal_http_geometry_report.json`，验证了等待、故障后原命令恢复和幂等回传；仍只有模拟源帧，不是完整任务或真实云台验收。

```bash
python3 scripts/validate_gimbal_geometry.py
python3 scripts/validate_scan_protocol.py
python3 scripts/validate_waypoint_executor.py
python3 scripts/validate_heading_planner.py
python3 scripts/validate_feasible_pointing.py
python3 scripts/validate_ordered_heading.py
```

测试覆盖四个航向、俯仰/横滚基向量、100 组四元数往返、安装平移与旋转、正下视、后向超限、无效四元数和非有限输入。它们证明软件坐标约定，不证明实机精度。
