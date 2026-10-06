# 作业区隔离与分层进出场

## 2026年10月6日用户确认后的现行口径

25个周界点围成的整块区域是科目一、科目二的合法空域；科目三树林只是不搜索，允许进出场过境。每架飞机的作业空间在水平方向不重叠，工作区之间保留20m间隔。进出场也必须留在25点周界内，使用分层高度和路径预约降低相遇风险。动态目标离开本机作业区后直接放弃，不交接、不越区跟随。

因此，原来的“全高度隔离带阻塞/必须端部通道”结论不再适用于现行任务策略。`config/airspace_policy.json` 是当前权威配置，`isolation_scope=WORK_ONLY`；默认启动不再因为进出场穿过工作区隔离带而拒绝。仍保留水平工作区、比赛周界、树林和机体包络检查。

## 查出的设计问题

2026-10-06复查默认112站方案：原先只在作业阶段要求飞机留在各自工作分区；高层INGRESS/EGRESS可以穿过中央带。航线中有2段INGRESS、3段EGRESS穿带。过去29.34分钟的完整原生搜索报告不能证明全高度禁飞要求满足，不应把分层高度不同解释为获准穿越禁区。

将当前完整带作为全高度禁飞区域，按周界内缩2m、禁区外扩2m检查：不允许树林过境时有4个连通片；即使允许树林过境仍有2个连通片，uav_1的15个站位都无法从2345起降框到达。不能用更高入场层、错时或者空间预约修复地理连通性。

## 当前默认行为

默认启动拒绝存在隔离带穿越的路线，原生服务在读取/冻结起点及发送解锁前检查。只选“允许树林过境”不能授权穿隔离带；字符串`"false"`也不会被当成授权。实际航段检查新增全高度隔离带及估计机体包络检查。

为保留任务/恢复接口联调，网页增加单独的、默认未选中的“仅条件演示，允许进出场穿越隔离带（非比赛合规）”。两项显式条件授权都不是比赛许可，更不能用于实机。接口字段为布尔值 `isolation_transit_simulation`。状态与新完整报告明确包含 `competition_airspace_compliant=false`。

新条件演示从零完成1008次合成观察、故障恢复、返场暂停恢复和六机落地，1761.3s（29.36min），采样最小间距16.870m，报告 `validation/sitl_isolation_conditional_demo_report.json`。这仅证明显式条件授权没有破坏原生执行链；它仍穿禁区，科一仍超时。最大巡航保持采样位移26.614m，仍没有制动包络证明。

## 待确认的10m端部通道候选

保留原来六个工作分区、完整20m工作隔离、112个扫描站位与九点顺序，只将全高度禁飞带在区域边缘收短，端部留10m通道；不把这段通道变成工作区。进出场/跨片转场改走周界内、禁区外扩2m后的区域内路径，通道仍由12m三维路径预约仲裁。

具体构造为原隔离带与“本区域内缩10m后的区域”求交，因此在与区域边界交会处留下通道，包含复杂房区边缘，不只是任意删掉两个矩形端点。卫星评审图见 `web/assets/isolation_corridor_review.png`；黄色仍是完整20m工作间隔，红色是候选全高度禁区，彩线是名义进出场。图片是方案图，不是浏览器截图或飞行验证。

候选完整路径已生成：`missions/fleet_plan_isolation_corridor_candidate.json`。逐段使用生产 `FlightSafety` 检查，没有使用隔离带穿越授权，名义指令及估计水平机体包络均不进入候选全高度禁区。原工作分区、站位和观察ID顺序不变。报告 `validation/isolation_corridor_candidate_report.json`，名义串行返场时间1640.205s（27.34min），不是原生实际耗时。

候选仍需要树林过境；不允许树林过境时不能覆盖所有区域。候选尚未获用户确认、没有原生飞行复验或连续物理安全证明；默认路线未被替换。10/20/30m端部留空仅有平面连通性试算，完整航段候选当前只验证10m，不据此认定10m是实机足够宽度。

网页勾选“端部通道候选（白虚线）”可叠加禁区核心和替代路线。候选使用固定方案起点；实际六机仍对应原任务。预览只读，不切换任务、不解锁。`GET /api/plan/isolation-candidate` 提供方案评审数据，不是控制接口。

采用前需确认：是否允许中央禁飞带端部收短、留下仅进出场使用的通道，以及科三树林上空是否允许过境。随后更新默认设计，按实际起点重新规划并进行双科目原生复验；不能用候选逻辑报告代替。

## 可复验命令

```bash
cd /home/zyc/zhixin_2026_ws
python3 scripts/audit_isolation_airspace.py
python3 scripts/validate_isolation_airspace.py
python3 scripts/validate_flight_safety.py
python3 scripts/plan_isolation_corridors.py
python3 scripts/render_isolation_review.py
```

服务器没有 Windows 字体时，评审图脚本会尝试 `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`，或可显式使用 `python3 scripts/render_isolation_review.py --font /absolute/path/to/chinese-font.ttc`。字体仅用于生成评审图片，不影响飞控与网页服务。

新启动且六机READY未解锁的原生服务，可运行拒绝与只读预览检查：

```bash
python3 scripts/verify_isolation_preflight.py
python3 scripts/verify_ready_service.py --report validation/ready_isolation_preflight_service_report.json
```

只用于原生条件联调的完整任务：

```bash
python3 scripts/verify_sitl.py --authorize-forest-transit --authorize-isolation-transit \
  --verify-return-recovery --timeout 600 --report validation/sitl_isolation_conditional_demo_report.json
```

原生执行期间不要启动另一组共用端口的飞控或另一个控制验证器。网页HTTP及接口检查已有证据；浏览器截图/点击验收仍未取得证据。
