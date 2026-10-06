# 云台逐点观察接口：模拟与实机的边界

目前是本机 HTTP/JSON 接口，不是已经完成的 ROS、ZR10、YOLO 或 LoRa 驱动。原生 SITL 用真实飞控位置反馈推进航点；取消自动模拟结果后，实际悬停到位才产生本机观察命令。原生模式中每架机独立等待，不再因一架机正常等待云台而阻塞其余五架；暂停、定位失联和协调故障仍全组保持。运动学预演仍保留旧全组等待行为。不是已经验证的实机故障处理能力。

## 六机并发执行

实机适配器应按固定飞机ID轮询 `GET /api/gimbal/command?vehicle_id=uav_1`，直到 `uav_6`。响应只包含该机命令和执行状态；无参数查询仅是兼容旧演示的首个等待命令，不能作为六机调度接口。网页和 `/api/state` 的 `pending_commands` 显示所有等待命令，`pending` 是旧单命令别名。

每机最多一个活动尝试。完成、失败、超时、重试和迟到结果只作用于对应飞机，观察顺序不变。不允许同机覆盖未完成命令；正常等待及云台执行失败先保持本机原观察点，其余机继续本区域任务。飞控状态改变、全局协调异常、定位过期、人工故障注入或暂停仍触发全组保持，不绕过进出场令牌，不自动独立RTL。**并发隔离不是无碰撞证明，也不是实际链路失联后的自主安全策略验收。**

取消网页自动模拟结果后，可在六个终端分别启动模拟执行器，例如：

```bash
python3 scripts/gimbal_mock_client.py --url http://127.0.0.1:8766 --vehicle-id uav_1
```

其余终端改为 `uav_2` 至 `uav_6`。不要为同一飞机同时开多个客户端；执行器会话所有权冲突会被拒绝。客户端没有读取实际视频，不能将模拟标志用于真实任务。

有界六机原生验证：新启动服务、六机定位就绪且未起飞时，运行 `python3 scripts/verify_concurrent_sitl.py`。脚本明确授权条件仿真跨树林，等待六架实际FCU各自产生观察命令，并行提交其余五架模拟证据，保留一架原点等待，验证本机观察失败不阻塞另一架下一点、原点重试与迟到尝试拒绝，再验证故障全组保持和恢复。测试结束暂停、不自动降落；无人实机连接的SITL服务可Ctrl+C停止并重启。报告 `validation/concurrent_sitl_report.json` 仅验证这段过程，不是全任务、真实视觉或Gazebo动力学验收。20倍飞控和墙钟证据时间不同，不能据此推算比赛耗时。

## 运行接口演示

```bash
cd /home/zyc/zhixin_2026_ws
bash scripts/run_sitl.sh --speedup 5 --port 8766
```

打开网页，取消“模拟云台自动回传完成事件”，按当前演示条件明确选择过境权限后启动。在另一终端：

```bash
cd /home/zyc/zhixin_2026_ws
python3 scripts/gimbal_mock_client.py --url http://127.0.0.1:8766 --once
```

客户端等候一个活动命令，按墙钟至少观察两秒，提交六个不同的**模拟**源帧索引。它没有读取视频或推理，所有成功标志都是模拟值，只用于验证命令→等待→结果→航点推进链路。去掉 `--once` 可持续处理；加速 SITL 与墙钟观察时间不同，不能把该运行耗时当比赛耗时。旧 `gimbal_http_report.json` 为三帧版本的历史冒烟，不代表当前六帧配置。

## 命令

最新名义指向门控版接口冒烟为 `validation/gimbal_http_feasible_pointing_report.json`，发出命令包含 `pointing_at_issue.executable`，候选满足原有视线误差限制；等待、故障恢复、不可观察保留断点、六帧模拟结果与重复幂等回传均通过。单点快照及模拟源帧不构成真实云台/视觉验收。

新增航向到位状态后的原生接口复验为 `validation/gimbal_http_heading_aligned_report.json`：实际机体航向稳定后生成外部命令，等待期间不自动完成，故障后恢复同一命令，UNOBSERVABLE 保留断点，过期任务结果被拒绝，六帧模拟结果推进一次，重复结果幂等。它是单点接口冒烟，不是完整任务或真实 ZR10/YOLO 证据。

`GET /api/gimbal/command` 返回 `{active, command, execution, hardware_authorized:false}`。`execution` 是执行生命周期状态，不是观察证据。

命令 `schema=zhixin/gimbal-command/v1` 包含：

- `mission_epoch`、`command_id`、`vehicle_id`：本次任务及唯一观察点，不是仅航点编号。
- `plan_revision`、`point_revision`：统一坐标、起点、分区、站位及逐点内容的哈希；位置或观察条件改变后不能复用完成标记。
- `owner=fleet-coordinator`、`command_type=OBSERVE_POINT`、`setpoint_type=GROUND_POINT`。
- `command_frame_id`、`map_origin_id`、`target_enu_m`、`station_enu_m`、`actual_position_enu_m`：共享 ENU 米制坐标。云台适配器还需用实时机体姿态与安装外参转换，不能把 ENU 方位角直接当 ZR10 电机角度。
- `issued_at`、`accept_before`：UNIX UTC 秒；`execution_timeout_s` 是时长。接单和执行超时由本机单调时钟检测，超时保持断点，不跳点或返航。
- `attempt_id`、`execution_attempt`：同一观察点的本次执行尝试；重新执行保持command_id、epoch、点版本及目标不变，换用新的尝试ID和签发时刻。
- `scan_zoom`、`calibration_id`、`observation_conditions`：每点的观察约束。

当前高度参考为 `SIMULATION_FLAT_GROUND`；没有真实地形模型，不能当实机 AGL 保证。

## 结果

`POST /api/gimbal/result` 必须回传上述五个身份/版本字段，以及 `status`。完成状态为 `FINISHED`、`SUCCESS_FOUND` 或 `SUCCESS_NOT_FOUND`，必须带 `evidence`：

- `kind`：明确 `SIMULATED_OBSERVATION` 或 `REAL_OBSERVATION`。
- `calibration_id`、`zoom_actual`、`pointing_error_deg`、`stable_duration_s`、`time_uncertainty_s`。
- `decoded`、`inference_completed`、`clarity_passed`、`attitude_valid`、`pointing_valid`、`zoom_valid`：必须由适配器实际观测得出，不能由计时结束推断。
- `source_frames`：不同的 `{source_session,sequence,capture_time,processed_at}`；不上传图像。后端拒绝重复源帧、命令之前的帧、未来处理时间、过短观察、错误版本及不合格声明。

`UNOBSERVABLE`、`FAILED`、`CANCELLED`、`PAUSED` 必须带 `reason_code`，只记录问题并保持原点；**不算无人、不跳观察点、不自动 RTL**。恢复后必须请求新的原点执行尝试，再重新采集有效证据；不能用失败尝试的迟到结果完成新尝试。详见下一节。

接口返回 `result_ack`。相同完成结果幂等重传；同一命令提交不同完成内容会报冲突，已有证据不会被覆盖。网页人工完成按钮仍是显式模拟通道，没有真实证据，不属于本严格接口。

## 接单、执行和恢复生命周期

当前已实现 `ISSUED → ACCEPTED → RUNNING → SUCCEEDED`，失败分支为 `REJECTED / FAILED / CANCELED / UNOBSERVABLE / TIMED_OUT`。`PAUSED` 观察结果也需重试。SDK返回成功不得直接声明SUCCEEDED；只有严格结果接口接受有效逐点证据后才记SUCCEEDED。

`POST /api/gimbal/status` 带五个身份/版本字段、`attempt_id`、`executor_session` 和 `status`。执行器先接单ACCEPTED再报告RUNNING；失败需要reason_code。一次尝试只能归一个执行器会话。重发接单不续期执行时限，RUNNING前必须接单，迟到旧attempt/旧会话不能接管新尝试。接单期限沿用配置300s，接单后的执行时限沿用60s；按墙钟计时，不随SITL物理倍率缩短。没有额外缩紧这些门槛，也不自动RTL。

观察失败、不可观测或超时后保持本机当前点；原生模式允许其余机继续，运动学模式仍全组等待。全局协调或飞控故障保持全组。恢复执行器后调用 `POST /api/gimbal/retry`，带五个身份/版本字段和旧attempt_id。任务必须运行且故障已解除；不允许为正在执行的尝试创建并行尝试。SITL重试还要求实际飞机回到原站位的既有到达容差、速度容差、新鲜姿态和名义可行指向。恢复成功时重新签发同一点，新attempt_id、新签发时间；旧证据不得拼入新观察窗口。真实适配器仍需自行重新验证云台、图像和连续稳定反馈，这个接口不是机械到位证明。

新客户端的结果必须携带attempt_id及接受该尝试的executor_session。为兼容旧演示客户端，仅第一次、尚未被状态通道认领的尝试允许旧格式结果；第二次起强制attempt_id，已认领的会话也必须匹配。此兼容模式不授权实机。`gimbal_mock_client.py` 已走接单→执行→证据回传流程，可在恢复后请求原点重试，但没有真实重连ZR10或RTSP，也没有自动清除飞控/协调故障。

网页等待命令下显示状态、尝试次数和原因；失败后显示“恢复执行器后重试原观察点”。任务暂停或仍有故障时按钮禁用。检查点记录执行状态；运动学存档恢复创建新尝试并保持暂停，原生SITL服务重启恢复仍未实现。

本次证据：`validation/scan_execution_report.json` 验证单调时钟接单/执行超时、尝试隔离、会话所有权、原点重试及证据幂等；`validation/gimbal_http_execution_lifecycle_report.json` 在原生飞控悬停中验证不可观测→原点第二次尝试→接单/执行→六个模拟源帧完成；`validation/scan_execution_ui_report.json` 验证实际网页按钮重试同一点并生成新尝试。它们均不是实际相机、YOLO、ZR10或LoRa验收。

新增生命周期后的完整科一原生回归为 `validation/sitl_scan_execution_regression_report.json`：六机从零完成1278个合成观察及故障恢复、全部落地，实际42.70min，仍超时。全程合成模式不验证每点外部执行器接单/运行状态；该范围由上述单点接口和逻辑测试覆盖，不冒充全任务真实视觉验收。

## 配置与断点

`config/scan_profile.json` 的六帧最低数量来自用户扫描/ROS规格草案，且当前两秒跨度不低于其1.5秒源帧跨度建议。两秒观察、指向误差等仍为接口假设；没有实现规格建议“稳定后观察3秒、误差≤min(0.5°，较小FOV的10%)、角速度≤0.5°/s”的完整反馈判定，不能称为满足实机观察规格。`calibrated=false` 时拒收 `REAL_OBSERVATION`；改为 true 只是配置声明，不证明实际标定完成。此项目仍统一声明 `hardware_authorized=false`，没有实机控制权限。姿态/指向及安装外参的当前边界见 `gimbal_geometry.md`。

新增断点保存观察配置版本、逐点状态和证据。修改任务或观察配置后拒绝复用旧证据。原生 SITL 仍不允许飞行中载入存档；旧版不含 `scan_protocol` 的存档不能按新版协议恢复。故障恢复目前保留活动命令，最终实机适配器必须对被中断观察重新采满有效时长，不可累加冻结画面。

下一阶段应实现 ROS 消息/服务适配器、ZR10 实际姿态及倍率反馈、视频源帧唯一标识与时间同步，并在机载侧维护完整证据，只向地面及 LoRa 发送必要摘要。当前未验证这些链路，也未实现移动目标实机跟随、抛投执行或带目标证据的整图有效覆盖统计。矩形几何覆盖不等于视觉发现率。

## 回归检查

```bash
python3 scripts/validate_scan_protocol.py
python3 scripts/validate_scan_execution.py
python3 scripts/validate_waypoint_executor.py
python3 scripts/validate_simulation.py
```
