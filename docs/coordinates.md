# 共享坐标与自动起点规划

## 权威输入与统一原点

新版规划读取 `config/boundary_wgs84.json` 的原始度分秒：科目一/二25个周界点、科目三8个树林点，来自用户的周界DOCX。原点固定为周界点2：纬度33.864219444444444°，经度113.70589722222222°。`scripts/geodesy.py` 用WGS84椭球完成地理坐标↔ECEF↔局部ENU，坐标描述包含 `frame_id=mission_map`、`map_origin_id`、轴向、单位和原点椭球高。

`config/site.yaml` 原有 `boundary_points_m` 是旧版近似坐标，仅供遗留生成脚本；当前 `fleet_core.py` 不再使用该表，也不再用111320线性经纬度近似。截图控制点同步改用新版周界重新拟合，仍然不是正射测绘地图。

## 六机各自EKF原点

`scripts/flight_coordinates.py` 不假定各机LOCAL_NED共用原点。每机等待自己的 `GPS_GLOBAL_ORIGIN`，再通过该原点的局部坐标基转换到共享任务ENU；发往飞控的目标由共享ENU逆变换到该机LOCAL_NED。无法取得原点时等待，不把起飞点平移量冒充EKF变换。原始LOCAL_NED和GLOBAL_POSITION_INT保留在网页API遥测供排查，全球整数经纬度分别按1e7缩放。

当前只对任务坐标与位置目标做转换，没有在此实现机体/云台姿态TF。局部NED速度范数仅用于稳定到点判据，不作为世界方向矢量输出。

## 起飞前读取实际摆放位置

1. `--mission-config` 的home控制SITL飞机最初生成位置，不是最终强制指定的导航起点。
2. 等六机收到新鲜LOCAL_NED及各自EKF原点、均未解锁时，点击启动。
3. 后端从六机实际统一ENU位置生成home，执行起降框边界、12m起点间距、分区/路线检查。
4. 起飞前重新生成航线并冻结任务；网页重新获取这一版计划。飞行时不静默更换home。
5. 发现任务开始后的EKF原点变化会保留游标并报告需核对的坐标异常；操作员应检查位置和任务版本，不把遥测恢复直接当成任务安全恢复。

这证明了SITL实际定位自动规划流程，**没有连接UM982、串口GPS2、实机RTK或LoRa**。实机控制链路仍需确认并验收。

## 高程不能偷换

附件只称71m为RTK测量的基准海拔，未证实其椭球高/MSL约定。新版保留 `altitude_reference=UNVERIFIED_RTK`，独立声明仿真假设：MSL71m、大地水准面差0m。ECEF只接受椭球高，当前仿真使用 `h=H+N`；实机必须提供核实的大地水准面差/垂直基准，不能继续使用模拟0m。

地图上Up不是任意地形的AGL；现有40m高度仍是平地任务高度。地形、楼顶、树冠和相机光心安装误差没有被这个数学转换消除，也不能以数学微米精度宣称定位精度。

## 验证方式

```bash
python3 scripts/validate_coordinates.py
python3 scripts/validate_coordinates.py --boundary-docx /实际路径/飞行区域周界范围.docx
```

WSL安装有GeographicLib `CartConvert` 时，验证器独立比较100余个场地局部点的正/逆转换；同时测试不同EKF原点对同一任务目标的一致性、极区ECEF转换和缺失原点拒绝。提供原始DOCX时逐项核对33对度分秒坐标。

报告 `validation/coordinates_report.json` 的 `reference_verified` 和 `source_docx_coordinates_verified` 分别表示独立数学参考比对和原文转录核对。没有工具或原文时明确记为false，不能用往返自洽代替独立验证。端到端飞行仍需另看 `validation/sitl_report.json`。
