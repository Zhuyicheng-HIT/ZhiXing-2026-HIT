const byId = (name) => document.getElementById(name);
const colors = ['#55b3ff','#a2d6ff','#ffa45d','#ffe080','#6ddd87','#b6ed65'];
let plan = null, state = null, view = {scale: .5, east: 0, north: 0}, drag = null;
let isolationCandidate = null;
function drawCameraWindows(){
  if(!state)return;
  let panel=document.getElementById('cameraWindows');
  if(!panel){
    panel=document.createElement('section');
    panel.id='cameraWindows';
    panel.style.cssText='margin:18px 0;padding:14px;border:1px solid #263957;border-radius:12px;background:#0b1424';
    panel.innerHTML='<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px"><strong>六机云台画面</strong><span style="font-size:12px;color:#9db0ca">Gazebo RGB 实时帧；YOLO / 实机 RTSP 可选</span></div><div id="cameraGrid" style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px"></div>';
    const fleet=byId('fleet');
    if(fleet&&fleet.parentElement)fleet.parentElement.insertBefore(panel,fleet.nextSibling);else document.body.append(panel);
  }
  const grid=byId('cameraGrid');
  const vehicles=Array.isArray(state.vehicles)?state.vehicles:(state.fleet||[]);
  for(let index=0;index<6;index++){
    let card=document.getElementById('cameraCard'+(index+1));
    if(!card){
      card=document.createElement('div');card.id='cameraCard'+(index+1);card.style.cssText='position:relative;min-height:150px;border:1px solid #294264;border-radius:8px;overflow:hidden;background:#111b2b';
      card.innerHTML='<img class="cameraLive" alt="Gazebo camera frame" style="display:none;width:100%;aspect-ratio:16/9;object-fit:cover;background:#101827"><canvas width="320" height="180" style="display:block;width:100%;aspect-ratio:16/9;background:#101827"></canvas><div class="cameraCaption" style="position:absolute;left:8px;right:8px;bottom:6px;color:#dbe8f8;font-size:12px;text-shadow:0 1px 2px #000"></div>';
      grid.append(card);
    }
    const live=card.querySelector('.cameraLive');
    live.onload=()=>{live.style.display='block';card.querySelector('canvas').style.display='none';};
    live.onerror=()=>{live.style.display='none';card.querySelector('canvas').style.display='block';};
    live.src='/api/camera/frame?vehicle_id=uav_'+(index+1)+'&t='+Date.now();
    const vehicle=vehicles[index]||{};
    const canvas=card.querySelector('canvas'),ctx=canvas.getContext('2d'),width=canvas.width,height=canvas.height;
    const gradient=ctx.createLinearGradient(0,0,0,height);gradient.addColorStop(0,'#17243a');gradient.addColorStop(1,'#304d45');ctx.fillStyle=gradient;ctx.fillRect(0,0,width,height);
    ctx.strokeStyle='#7790a9';ctx.globalAlpha=.38;for(let x=0;x<=width;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();}for(let y=0;y<=height;y+=30){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}ctx.globalAlpha=1;
    const yaw=Number(vehicle.gimbal?.azimuth_enu_deg??vehicle.gimbal_yaw_deg??0),pitch=Number(vehicle.gimbal?.elevation_deg??vehicle.gimbal_pitch_deg??-90),completed=Number(vehicle.completed_scans??vehicle.completed_segments??0);
    const cx=width/2+Math.max(-width*.32,Math.min(width*.32,yaw/60*width*.32)),cy=height/2+Math.max(-height*.25,Math.min(height*.25,(pitch+90)/35*height*.25));
    ctx.strokeStyle='#ffba66';ctx.lineWidth=2;ctx.strokeRect(cx-28,cy-20,56,40);ctx.beginPath();ctx.moveTo(cx-8,cy);ctx.lineTo(cx+8,cy);ctx.moveTo(cx,cy-8);ctx.lineTo(cx,cy+8);ctx.stroke();
    const caption=card.querySelector('.cameraCaption');caption.textContent='UAV'+(index+1)+' · 云台 '+yaw.toFixed(1)+'° / '+pitch.toFixed(1)+'° · 已完成扫描 '+completed;
  }
}
setInterval(drawCameraWindows,500);
const canvas = byId('mapCanvas'), context = canvas.getContext('2d');
const coverageToggle = document.createElement('label');
coverageToggle.innerHTML = '<input id="scanCoverage" type="checkbox" checked>各站负责扫描地块';
document.querySelector('.map-toolbar').append(coverageToggle);
byId('scanCoverage').addEventListener('change',draw);
byId('footprint').parentElement.firstChild.textContent='出发 / 房区最大扫描边长 / m';
byId('footprint').parentElement.nextElementSibling.textContent='ZR-10 16:9：出发/房区35.6×20m，草区60m高度、67.6×38m；草区整站边长取1.8倍。平地估算，未实测标定。';
const coordinateReadout=document.createElement('div');
coordinateReadout.style.cssText='position:absolute;top:12px;left:12px;padding:9px;background:#0c1428df;border-radius:8px;font-size:12px;pointer-events:none;z-index:2';
coordinateReadout.textContent='左键取点 · 右键复制经度,纬度（WGS84）';
canvas.parentElement.append(coordinateReadout);
async function selectMapCoordinate(event,copy=false){
  if(!plan)return;
  const rect=canvas.getBoundingClientRect();
  const east=(event.clientX-rect.left-view.east)/view.scale;
  const north=(view.north-event.clientY+rect.top)/view.scale;
  try{
    const result=await api('/api/map/coordinates?east='+encodeURIComponent(east)+'&north='+encodeURIComponent(north));
    const text=result.longitude_deg.toFixed(8)+','+result.latitude_deg.toFixed(8);
    coordinateReadout.textContent='WGS84 经度,纬度：'+text+' · ENU '+east.toFixed(1)+','+north.toFixed(1)+'m';
    if(copy){await navigator.clipboard.writeText(text);toast('已复制经度,纬度：'+text);}
  }catch(error){toast('取点或复制失败：'+error.message);}
}
const satellite = new Image();
satellite.src = 'assets/satellite.jpg';
satellite.onload = () => draw();
let toastTimer;
function toast(message) {byId('toast').textContent = message;byId('toast').style.display = 'block';clearTimeout(toastTimer);toastTimer = setTimeout(() => byId('toast').style.display = 'none',8000);}
async function api(path,data) {const response = await fetch(path,data ? {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)} : {});const result = await response.json();if(!response.ok) throw new Error(result.error || response.status);return result;}
async function control(action,extra={}) {try {state = await api('/api/control',{action,...extra});if(action==='start'&&state.mode==='ardupilot_internal_physics_sitl'){plan=await api('/api/plan');renderPlan();}renderState();return true;}catch(error){toast(error.message);return false;}}
function screen(point) {return [view.east+point[0]*view.scale,view.north-point[1]*view.scale];}
function geometryPath(geometry) {context.beginPath();const parts = geometry.type==='Polygon' ? [geometry.coordinates] : geometry.type==='MultiPolygon' ? geometry.coordinates : [];for(const rings of parts) for(const ring of rings) {ring.forEach((point,index)=>{const position=screen(point);if(index===0)context.moveTo(...position);else context.lineTo(...position);});context.closePath();}}
function paint(geometry,fill,stroke) {if(!geometry)return;geometryPath(geometry);context.fillStyle=fill;context.fill('evenodd');if(stroke){context.strokeStyle=stroke;context.lineWidth=1.2;context.stroke();}}
function fit() {if(!plan)return;const width=canvas.clientWidth,height=canvas.clientHeight;view.scale=Math.min((width-90)/920,(height-70)/1100);view.east=45+230*view.scale;view.north=35+620*view.scale;draw();}
function size() {const ratio=window.devicePixelRatio || 1;canvas.width=canvas.clientWidth*ratio;canvas.height=canvas.clientHeight*ratio;context.setTransform(ratio,0,0,ratio,0,0);fit();}
function draw() {
  renderFlightMode();
  context.clearRect(0,0,canvas.clientWidth,canvas.clientHeight);if(!plan)return;
  if(byId('satellite').checked&&satellite.complete&&satellite.naturalWidth&&plan.satellite){const matrix=plan.satellite.pixel_to_enu;context.save();context.transform(view.scale*matrix[0][0],-view.scale*matrix[1][0],view.scale*matrix[0][1],-view.scale*matrix[1][1],view.east+view.scale*matrix[0][2],view.north-view.scale*matrix[1][2]);context.globalAlpha=.78;context.drawImage(satellite,0,0);context.restore();}
  const sectorColors=['#428fea2c','#ffa35b2c','#82d6652c'];plan.partitions.forEach((part,index)=>{paint(part.geometry,sectorColors[index],sectorColors[index].slice(0,7));paint(part.buffer,'#fa698951','#ffb0bc');});
  paint(plan.forest,'#67716699','#b9c2ae');paint(plan.launch,'#aab5ff55','#d7deff');paint(plan.perimeter,'#ffffff03','#ffe6a6');paint(plan.coverage.uncovered,'#ff3232bb','#ff6666');
  if(byId('candidatePreview').checked&&isolationCandidate){
    context.save();context.setLineDash([7,5]);
    isolationCandidate.partitions.forEach(partition=>paint(partition.buffer,'#ffffff15','#ffffff'));
    context.strokeStyle='#ffffff';context.lineWidth=2;context.beginPath();
    isolationCandidate.vehicles.forEach(vehicle=>vehicle.segments.forEach(segment=>{
      if(['INGRESS','EGRESS','REPOSITION'].includes(segment.state)){
        context.moveTo(...screen(segment.origin));context.lineTo(...screen(segment.destination));
      }
    }));
    context.stroke();context.restore();
  }
  plan.vehicles.forEach((vehicle,index)=>{
    paint(vehicle.flight_region,colors[index]+'10',colors[index]+'66');
    if(byId('scanCoverage').checked){
      context.save();geometryPath(vehicle.search_region);context.clip('evenodd');
      const width=(vehicle.scan_configuration?.station_footprint_m||plan.footprint_m)*view.scale;
      context.fillStyle=colors[index]+'16';context.strokeStyle=colors[index]+'55';context.lineWidth=.7;
      if(vehicle.station_tasks){
        context.restore();context.save();
        vehicle.station_tasks.forEach(task=>paint(task.geometry,colors[index]+'18',colors[index]+'66'));
      }else{
        vehicle.stations.forEach(point=>{const position=screen(point);context.fillRect(position[0]-width/2,position[1]-width/2,width,width);context.strokeRect(position[0]-width/2,position[1]-width/2,width,width);});
      }
      context.restore();
    }
    if(byId('routes').checked){
      context.save();context.strokeStyle=colors[index]+'bb';context.lineWidth=1.2;
      for(const segment of vehicle.segments){
        if(!['INGRESS','EGRESS','NAVIGATE','REPOSITION'].includes(segment.state))continue;
        context.setLineDash(segment.state==='NAVIGATE'?[]:[5,4]);
        context.beginPath();context.moveTo(...screen(segment.origin));context.lineTo(...screen(segment.destination));context.stroke();
      }
      context.restore();
    }
    vehicle.stations.forEach(point=>{const position=screen(point);context.fillStyle=colors[index];context.beginPath();context.arc(...position,2.2,0,Math.PI*2);context.fill();});
    const home=screen(vehicle.home);context.strokeStyle=colors[index];context.strokeRect(home[0]-4,home[1]-4,8,8);
  });
  if(state)state.vehicles.forEach((vehicle,index)=>{
    const position=screen(vehicle.position),configuration=plan.vehicles[index].scan_configuration;
    if(vehicle.target){
      context.strokeStyle=colors[index];context.setLineDash([4,4]);context.beginPath();context.moveTo(...position);context.lineTo(...screen(vehicle.target));context.stroke();context.setLineDash([]);
      const target=screen(vehicle.target);context.strokeRect(target[0]-4,target[1]-4,8,8);
      if((vehicle.resume_state||vehicle.state)==='SCAN'&&configuration){
        const [width,height]=configuration.instantaneous_frame_m.map(value=>value*view.scale);
        context.fillStyle=colors[index]+'45';context.fillRect(target[0]-width/2,target[1]-height/2,width,height);
        context.lineWidth=2;context.strokeRect(target[0]-width/2,target[1]-height/2,width,height);
      }
    }
    context.fillStyle=colors[index];context.strokeStyle='#081224';context.lineWidth=2;context.beginPath();context.arc(...position,7,0,2*Math.PI);context.fill();context.stroke();context.font='bold 11px system-ui';context.fillStyle='#ffffff';context.fillText(vehicle.id+' / '+vehicle.position[2].toFixed(0)+'m',position[0]+10,position[1]-8);
  });
}
function metric(label,value,good=true) {const element=document.createElement('div');element.className='metric';const name=document.createElement('span');name.textContent=label;const result=document.createElement('strong');result.className=good?'good':'bad';result.textContent=value;element.append(name,result);return element;}
byId('candidatePreview').onchange=async()=>{
  try{
    if(byId('candidatePreview').checked&&!isolationCandidate)isolationCandidate=await api('/api/plan/isolation-candidate');
    byId('candidatePreviewStatus').hidden=!byId('candidatePreview').checked;
    draw();
  }catch(error){byId('candidatePreview').checked=false;toast(error.message);}
};
function renderPlan() {
  const metrics=byId('metrics');metrics.replaceChildren(metric('连续轨迹分离',plan.validation.passed?'通过（仅理想模型）':'失败',plan.validation.passed),metric('最小三维间距',plan.validation.min_3d_distance_m.toFixed(1)+'m'),metric('定点扫描航点',plan.vehicles.reduce((total,vehicle)=>total+vehicle.stations.length,0)+'个'),metric('平面矩形覆盖',(plan.coverage.ratio*100).toFixed(2)+'%',plan.coverage.ratio>.999),metric('预计完整时长',(plan.duration_s/60).toFixed(1)+'分钟',plan.within_time_budget));
  const warnings=byId('warnings');warnings.replaceChildren();const messages=[...plan.limitations];
  if(plan.isolation_airspace?.work_crossings?.length)messages.unshift('作业区存在水平隔离间隔问题，当前方案不应启动。');
  if(!plan.within_time_budget)messages.unshift('当前统一4m/s、分区蛇形扫描模型仍超过比赛时间预算；需真实识别能力与扫描协议标定，不能仅缩短动画。');
  if(plan.requires_forest_transit_permission)messages.unshift('当前进出场穿树林，需要允许周界内树林过境。');
  messages.forEach(message=>{const item=document.createElement('div');item.className='warning';item.textContent=message;warnings.append(item);});
  byId('faultVehicle').replaceChildren(...plan.vehicles.map(vehicle=>{const option=document.createElement('option');option.value=vehicle.id;option.textContent=vehicle.id+' · '+vehicle.label;return option;}));
}
function renderState() {if(!state)return;if(state.running||state.pending){byId('synthetic').checked=state.synthetic;byId('forestPermission').checked=state.forest_permission;}const pendingCommand=state.pending&&state.pending.command;byId('gimbalCommand').textContent=pendingCommand?pendingCommand.command_id+' · '+pendingCommand.command_type+'\nENU目标 '+pendingCommand.target_enu_m.map(value=>value.toFixed(1)).join(' / ')+' m\n点版本 '+pendingCommand.point_revision.slice(0,12)+' · '+pendingCommand.calibration_id+'\n'+(pendingCommand.calibration_configured?'已声明标定（仍未授权实机）':'未标定，仅模拟接口'):'当前没有外部扫描命令';byId('progress').value=state.progress;byId('clock').textContent=Math.floor(state.stamp/60).toString().padStart(2,'0')+':'+Math.floor(state.stamp%60).toString().padStart(2,'0');byId('connection').textContent=state.faults&&Object.keys(state.faults).length?'断点恢复中':state.pending?'等待云台完成事件':state.running?'仿真运行中':'仿真已暂停';byId('fleet').replaceChildren(...state.vehicles.map((vehicle,index)=>{const card=document.createElement('div');card.className='vehicle';const title=document.createElement('strong');title.style.color=colors[index];title.textContent=vehicle.id+' · '+plan.vehicles[index].label;const status=document.createElement('span');status.className='state';status.textContent=vehicle.state;const coordinates=document.createElement('div');coordinates.className='coordinates';coordinates.textContent=vehicle.position.map(value=>value.toFixed(1)).join(' / ')+' m';const scan=document.createElement('div');scan.textContent='航点 '+(vehicle.station===undefined||vehicle.station===null?'—':vehicle.station+1)+' · 已完成观察 '+vehicle.completed_scans;const gimbal=document.createElement('div');gimbal.textContent='云台 ENU方位 '+vehicle.gimbal.azimuth_enu_deg.toFixed(0)+'° / 仰角 '+vehicle.gimbal.elevation_deg.toFixed(0)+'°';card.append(title,status,coordinates,scan,gimbal);const pointing=vehicle.gimbal.pointing;if(pointing){const geometry=document.createElement('div');geometry.className='coordinates';geometry.textContent=pointing.valid?'安装系 '+pointing.azimuth_deg.toFixed(1)+'° / '+pointing.elevation_deg.toFixed(1)+'° · '+(pointing.nominal_reachable?'名义可达，SDK未核实':'精确方向超限，另见候选（非实机验收）'):'指向不可用：'+pointing.invalid_reason;card.append(geometry);if(pointing.executable){const feasible=pointing.executable;const command=document.createElement('div');command.className='coordinates';command.textContent='可执行名义指向 '+feasible.azimuth_deg.toFixed(1)+'° / '+feasible.elevation_deg.toFixed(1)+'° · 视线误差 '+feasible.pointing_error_deg.toFixed(3)+'° · '+(feasible.feasible?'满足配置误差，未验证SDK':'不可行，等待恢复');card.append(command);}}if(vehicle.scan_alignment){const alignment=vehicle.scan_alignment;const heading=document.createElement('div');heading.className='coordinates';heading.textContent='机头目标 '+alignment.heading_mission_deg.toFixed(1)+'° ENU / 误差 '+(Number.isFinite(alignment.actual_heading_error_deg)?alignment.actual_heading_error_deg.toFixed(1):'待反馈')+'° / 稳定 '+alignment.stable_s.toFixed(1)+' s';card.append(heading);}return card;}));byId('events').replaceChildren(...[...state.log].reverse().slice(0,15).map(event=>{const row=document.createElement('p');row.textContent='['+event.time_s.toFixed(1)+'s] '+event.event+' '+event.message;return row;}));draw();}
byId('startButton').onclick=()=>control('start',{forest_permission:byId('forestPermission').checked,isolation_transit_simulation:byId('isolationPermission').checked,synthetic:byId('synthetic').checked});byId('pauseButton').onclick=()=>control('pause');byId('resetButton').onclick=()=>control('reset');byId('saveButton').onclick=()=>control('checkpoint');byId('restoreButton').onclick=async()=>{if(await control('restore')){plan=await api('/api/plan');renderPlan();renderState();}};byId('speed').onchange=()=>control('speed',{value:Number(byId('speed').value)});byId('faultButton').onclick=()=>control('fault',{vehicle_id:byId('faultVehicle').value});byId('recoverButton').onclick=()=>control('recover',{vehicle_id:byId('faultVehicle').value});byId('resultButton').onclick=()=>{if(!state.pending){toast('当前没有等待完成事件的观察点');return;}control('result',{...state.pending,status:'FINISHED'});};byId('planButton').onclick=async()=>{try{const homes=JSON.parse(byId('homes').value);if(await control('replan',{homes,subject:Number(byId('subject').value),footprint:Number(byId('footprint').value)})){plan=await api('/api/plan');renderPlan();fit();}}catch(error){toast(error.message);}};
byId('fitButton').onclick=fit;byId('satellite').onchange=draw;byId('routes').onchange=draw;window.addEventListener('resize',size);
canvas.onpointerdown=event=>{if(event.button!==0)return;drag={x:event.clientX,y:event.clientY,east:view.east,north:view.north,moved:false};canvas.setPointerCapture(event.pointerId);};
canvas.onpointermove=event=>{if(drag){if(Math.hypot(event.clientX-drag.x,event.clientY-drag.y)>4)drag.moved=true;if(drag.moved){view.east=drag.east+event.clientX-drag.x;view.north=drag.north+event.clientY-drag.y;draw();}}};
canvas.onpointerup=event=>{const clicked=drag&&!drag.moved;drag=null;if(clicked)selectMapCoordinate(event);};
canvas.onpointercancel=()=>{drag=null;};
canvas.oncontextmenu=event=>{event.preventDefault();drag=null;selectMapCoordinate(event,true);};
canvas.addEventListener('wheel',event=>{event.preventDefault();const rect=canvas.getBoundingClientRect(),east=event.clientX-rect.left,north=event.clientY-rect.top;const factor=event.deltaY<0?1.1:1/1.1;view.east=east+(view.east-east)*factor;view.north=north+(view.north-north)*factor;view.scale*=factor;draw();},{passive:false});
async function init(){try{plan=await api('/api/plan');renderPlan();size();state=await api('/api/state');renderState();}catch(error){toast('后端未启动：请运行 python scripts/sim_server.py。'+error.message);byId('connection').textContent='后端未连接';}}
init();setInterval(async()=>{if(!plan)return;try{state=await api('/api/state');renderState();}catch(error){byId('connection').textContent='连接中断（不表示实机状态）';}},250);
function renderFlightMode(){
    if(!state)return;
    if(plan.coverage_planning){
        let coveragePanel=byId('coveragePlanningStatus');
        if(!coveragePanel){coveragePanel=document.createElement('div');coveragePanel.id='coveragePlanningStatus';coveragePanel.className='warning';byId('metrics').after(coveragePanel);}
        const strategy=plan.coverage_planning.strategy==='owned_scan_cells'?'直线耗时均衡分区 / 唯一地块归属 / 裁剪扫视':plan.coverage_planning.strategy==='sparse_grid_candidates'?'低密度网格候选 / 贪心择优':'旧贪心覆盖';
        const counts=plan.vehicles.map(vehicle=>vehicle.id+': '+vehicle.stations.length+'站').join(' · ');
        const observations=plan.vehicles.reduce((total,vehicle)=>total+vehicle.segments.filter(segment=>segment.state==='SCAN').length,0);
        coveragePanel.textContent='覆盖规划：'+strategy+'，站位向内偏好'+(plan.coverage_planning.preferred_station_clearance_m||0)+'m。'+counts+'；共'+observations+'条扫描段。淡色地块为各站实际负责范围，不再把重叠的整站矩形重复扫完；实线为飞机连接、虚线为进出场。视场边缘仍保留必要重叠；几何覆盖非实机识别保证。';
    }
    if(plan.flight_safety){
        const toolbar=document.querySelector('.map-toolbar span');
        toolbar.textContent='统一 ENU · 指令'+plan.vehicles[0].work_altitude_m+'m AGL · 光轴扫速4m/s · 每'+plan.coordination.entry_release_interval_s+'s放行（冲突等待）';
    }
    const pendingCommands=state.pending_commands||[state.pending].filter(Boolean);
    if(plan.observation_conditions){
        let timingPanel=byId('scanTimingStatus');
        if(!timingPanel){timingPanel=document.createElement('div');timingPanel.id='scanTimingStatus';timingPanel.className='warning';byId('metrics').after(timingPanel);}
        const conditions=plan.observation_conditions;
        const motion=plan.scan_motion_model||{};
        timingPanel.textContent='连续蛇形扫视 '+motion.maximum_ground_speed_m_s+'m/s · 端点驻留 '+conditions.minimum_observation_s+'s · 移动中至少 '+conditions.minimum_distinct_frames+'个不同源帧 · 起点快速摆位不计时 · 无端点稳定等待（非实机标定）';
    }
    if(plan.regional_scan_summary){
        let panel=byId('regionalScanStatus');
        if(!panel){panel=document.createElement('div');panel.id='regionalScanStatus';panel.className='warning';panel.style.whiteSpace='pre-line';byId('metrics').after(panel);}
        panel.textContent='ZR-10 16:9矩形视场（4m/s；平地模型，未标定）\n'+plan.regional_scan_summary.map(item=>item.vehicle_id+' '+item.zone+'：'+item.stations+'站 / '+item.sweep_segments+'条扫描段，作业高度'+item.work_altitude_m+'m，视场'+item.instantaneous_frame_m.map(value=>value.toFixed(1)).join('×')+'m / '+item.scan_zoom.toFixed(2)+'倍，最大整站'+item.station_footprint_m+'m；扫描'+(item.scan_s/60).toFixed(1)+'min；放行'+item.entry_release_s+'s / 进场等待'+item.nominal_entry_path_wait_s+'s').join('\n');
    }
    if(!Object.keys(state.faults||{}).length){
        byId('connection').textContent=!state.running?'仿真已暂停':state.concurrent_observations&&pendingCommands.length?'运行中 · '+pendingCommands.length+'架独立等待观察结果':state.running?'仿真运行中':'仿真已暂停';
    }
    let executionPanel=byId('scanExecutionStatus');
    if(!executionPanel){
        executionPanel=document.createElement('div');
        executionPanel.id='scanExecutionStatus';
        executionPanel.className='warning';
        byId('gimbalCommand').after(executionPanel);
    }
    executionPanel.replaceChildren();
    executionPanel.hidden=!pendingCommands.length;
    for(const pending of pendingCommands){
        const execution=pending.execution;
        if(!execution)continue;
        const description=document.createElement('div');
        description.textContent=pending.vehicle_id+' · '+execution.status+' · 第'+execution.execution_attempt+'次尝试 · '+(execution.reason_code||'等待执行器回传')+'（未验证实机）';
        executionPanel.append(description);
        if(execution.retry_available){
            const retry=document.createElement('button');
            retry.textContent='恢复执行器后重试原观察点';
            retry.disabled=!state.running||Object.keys(state.faults||{}).length>0;
            retry.onclick=async()=>{
                const command=pending.command;
                const identity=Object.fromEntries(['vehicle_id','command_id','mission_epoch','plan_revision','point_revision','attempt_id'].map(name=>[name,command[name]]));
                await control('retry_observation',identity);
            };
            executionPanel.append(retry);
        }
    }
    const sitl=state.mode==='ardupilot_internal_physics_sitl';
    if(sitl&&state.flight_safety_audit){
        let safetyPanel=byId('flightSafetyStatus');
        if(!safetyPanel){safetyPanel=document.createElement('div');safetyPanel.id='flightSafetyStatus';safetyPanel.className='warning';executionPanel.after(safetyPanel);}
        safetyPanel.replaceChildren();
        const audit=state.flight_safety_audit;
        const description=document.createElement('div');
        description.textContent='真实作业边界直接检查（不外扩） · 已检查 '+audit.checks+'次 / 拦截 '+audit.rejections+'次 · 名义水平包络半径 '+audit.estimated_horizontal_body_radius_m+'m（未实测）';
        safetyPanel.append(description);
        if(state.faults.flight_safety){
            const reason=document.createElement('div');reason.textContent=state.faults.flight_safety;safetyPanel.append(reason);
            const recover=document.createElement('button');recover.textContent='核对并解除边界保持';recover.onclick=()=>control('recover',{vehicle_id:'flight_safety'});safetyPanel.append(recover);
            const note=document.createElement('small');note.textContent='继续前重新检查实际位置和原航段；仍不合法会再次保持，不会自动改道或RTL。';safetyPanel.append(note);
        }
    }
    byId('modeBadge').textContent=sitl?'真实ArduPilot SITL位置反馈 · 原生内部物理':'运动学预演 · 非飞控反馈';
    if(sitl&&state.fcu_mission_elapsed_s!==undefined){
        byId('modeBadge').textContent+=' · 飞控任务 '+(state.fcu_mission_elapsed_s/60).toFixed(1)+'分钟 · 当前最小间距 '+state.actual_min_separation_m.toFixed(1)+'m';
    }
    if(sitl&&state.scan_alignment_audit){
        const audit=state.scan_alignment_audit;
        byId('modeBadge').textContent+=' · 保序航向规划 '+(audit.station_heading_plans||0)+'次 / 复用 '+(audit.station_heading_reuses||0)+'点';
        byId('modeBadge').textContent+=' · 入站预转向 '+(audit.approach_heading_uses||0)+'站';
    }
    if(sitl&&state.reservation_audit){
        const audit=state.reservation_audit;
        byId('modeBadge').textContent+=' · 最大并行入场 '+audit.maximum_parallel_entries+'架 / 航段预约等待 '+audit.blocked_checks+'次';
    }
    const entryDescription=state.entry_policy==='layered_reserved'?'分层入场并检查航段空间预约，转场与返场保持独占':'进出场通道独占';
    document.querySelector('footer').textContent=sitl?'飞机位置来自六个独立SITL；实际到点且稳定才推进状态机，'+entryDescription+'。进度不是物理时间。云台/目标仍为合成；不是Gazebo动力学或实机验证。':'运动学任务预演：不是ArduPilot飞控闭环。科二抛投待确认。使用说明见工作空间 README.md。';
    ['speed','subject','footprint','homes','planButton','resetButton','restoreButton'].forEach(id=>{byId(id).disabled=sitl;});
    if(sitl){
        if(state.hold_recovery_audit){
            let holdPanel=byId('holdRecoveryStatus');
            if(!holdPanel){holdPanel=document.createElement('div');holdPanel.id='holdRecoveryStatus';holdPanel.className='warning';byId('metrics').after(holdPanel);}
            const audit=state.hold_recovery_audit;
            holdPanel.textContent=(state.running&&state.latched_group_hold_enu_m?'恢复中：等待GUIDED机实际回到保持点（误差<0.8m、速度<0.6m/s），再预约原航段。':'保持恢复：到位低速后重新预约，不跳站。')+' 历史采样最大保持漂移 '+audit.maximum_observed_hold_drift_m.toFixed(1)+'m；保持命令不等于瞬时刹停，尚无实机制动包络。';
        }
        const speed=byId('speed');
        if(![...speed.options].some(option=>Number(option.value)===state.speed))speed.add(new Option(state.speed+'× SITL物理',String(state.speed)));
        speed.value=String(state.speed);
        byId('subject').value=String(plan.subject);
        byId('footprint').value=plan.footprint_m;
        byId('homes').value=JSON.stringify(plan.vehicles.map(vehicle=>vehicle.home));
        byId('clock').textContent='航点名义进度 '+(state.progress*100).toFixed(1)+'% · 飞控 '+(state.fcu_mission_elapsed_s/60).toFixed(1)+'分钟 · 独占转场 '+(state.transit_owner||'空闲')+' · 入场 '+((state.active_entries||[]).join(',')||'无')+' · 返场 '+((state.active_returns||[]).join(',')||'无')+' · 返场策略 '+(state.return_policy==='layered_reserved'?'分层预约':'独占');
        const cards=byId('fleet').querySelectorAll('.vehicle');
        state.vehicles.forEach((vehicle,index)=>{
            if(!cards[index]||!vehicle.telemetry)return;
            let row=cards[index].querySelector('.flight-feedback');
            if(!row){row=document.createElement('div');row.className='flight-feedback';cards[index].append(row);}
            row.textContent='FCU '+vehicle.telemetry.mode+' · '+(vehicle.telemetry.armed?'已解锁':'未解锁')+' · 定位更新 '+(vehicle.telemetry.age_s===null?'未收到':vehicle.telemetry.age_s.toFixed(2)+'s');
            const reservationWait=state.reservation_waits?.[vehicle.id];
            if(reservationWait){const waitRow=document.createElement('div');waitRow.className='phase-feedback';waitRow.textContent='等待航段预约：与 '+reservationWait.vehicle_id+' 剩余航段距离 '+reservationWait.distance_m.toFixed(1)+'m，保留原游标';cards[index].append(waitRow);}
            if(vehicle.scan_alignment){
                const headingRow=document.createElement('div');
                headingRow.className='phase-feedback';
                headingRow.textContent='观察原序不变 · '+(vehicle.scan_alignment.policy==='REUSE_ORDERED_STATION_HEADING'?'复用站位航向':'前瞻连续观察点')+' · 连续前缀 '+vehicle.scan_alignment.ordered_prefix_count+'点';
                cards[index].append(headingRow);
            }
            const phases=state.phase_elapsed_s?.[vehicle.id];
            if(phases){
                const total=(names)=>names.reduce((sum,name)=>sum+(phases[name]||0),0)/60;
                const phaseRow=document.createElement('div');
                phaseRow.className='phase-feedback';
  phaseRow.textContent='阶段累计：飞行 '+total(['CLIMB','INGRESS','DESCEND_TO_WORK','NAVIGATE','REPOSITION_CLIMB','REPOSITION','RETURN_CLIMB','EGRESS','RETURN_DESCEND','LAND']).toFixed(1)+'分 · 观察 '+total(['STABILIZE','SCAN','CONFIRM_STATIC','TRACK_MOVING','WAIT_RESULT']).toFixed(1)+'分 · 搜索同步 '+total(['SEARCH_SYNC']).toFixed(1)+'分 · 航向/视线等待 '+total(['SCAN_ALIGN','SCAN_WAIT_POINTING','SCAN_WAIT_ATTITUDE']).toFixed(1)+'分 · 排队 '+total(['WAIT_RELEASE','WAIT_RETURN_SLOT','WAIT_TRANSIT_SLOT','WAIT_PATH_RESERVATION']).toFixed(1)+'分 · 恢复刹停 '+total(['WAIT_HOLD_STABLE']).toFixed(1)+'分';
                cards[index].append(phaseRow);
            }
        });
        if(state.preparing)byId('connection').textContent='等待六机解锁 / 起飞反馈';
    }
}
