(() => {
  const panel = document.createElement('section');
  panel.id = 'missionEditor';
  panel.innerHTML = `<h2>地面站航线编辑</h2>
    <small>编辑的是规划航点建议，扫描站位保持独立。修改进出场、转场或返场点后，保存并重新规划，系统再进行时间、边界和间距检查。</small>
    <label>飞机<select id="missionVehicle"></select></label>
    <div class="buttons"><button id="missionLoad">读取当前规划航点</button><button id="missionSave" class="primary">保存航点草稿</button><button id="missionApply">按正式区域重新规划</button></div>
    <label>航点列表<select id="missionList" size="7"></select></label>
    <div class="buttons"><button id="missionAddBefore">前插</button><button id="missionAddAfter">后插</button><button id="missionDelete">删除</button></div>
    <div class="buttons"><button id="missionUp">上移</button><button id="missionDown">下移</button><button id="missionReset">恢复原规划</button></div>
    <label>当前点 ENU / m<input id="missionPoint" type="text" placeholder="east,north,altitude"></label>
    <button id="missionUpdate">更新当前点坐标</button>
    <div id="missionStatus" role="status" class="warning">未读取航线</div>`;
  document.querySelector('aside').prepend(panel);
  const style = document.createElement('style');
  style.textContent = '#missionEditor input[type=text]{width:100%;background:#0b1323;border:1px solid #34425e;border-radius:6px;color:#e4edfc;padding:9px}#missionEditor select[size]{height:155px}#missionEditor .active{border-color:#56e6da;background:#19635c}';
  document.head.append(style);
  let data = null, selectedVehicle = '', selectedPoint = -1, dragging = null, initialized = false;
  const status = message => { byId('missionStatus').textContent = message; };
  const route = () => data?.routes.find(item => item.vehicle_id === selectedVehicle);
  const pointText = point => point.map(value => Number(value).toFixed(1)).join(',');
  function refresh() {
    const current = route();
    byId('missionVehicle').replaceChildren(...(data?.routes || []).map(item => { const option = document.createElement('option'); option.value = item.vehicle_id; option.textContent = item.vehicle_id + ' · ' + item.label; option.selected = item.vehicle_id === selectedVehicle; return option; }));
    byId('missionList').replaceChildren(...(current?.points || []).map((item, index) => { const option = document.createElement('option'); option.value = index; option.textContent = `${index + 1}. ${item.state || 'MANUAL'} · ${pointText(item.point)}`; option.selected = index === selectedPoint; return option; }));
    const selected = current?.points[selectedPoint];
    byId('missionPoint').value = selected ? pointText(selected.point) : '';
    draw();
  }
  function enu(event) { const rect = canvas.getBoundingClientRect(); return [(event.clientX - rect.left - view.east) / view.scale, (view.north - event.clientY + rect.top) / view.scale]; }
  function mapPoint(point) { return screen(point); }
  function nearest(event) {
    const current = route(); if (!current) return -1;
    const rect = canvas.getBoundingClientRect(), pixel = [event.clientX - rect.left, event.clientY - rect.top];
    let best = -1, distance = 14;
    current.points.forEach((item, index) => { const position = mapPoint(item.point); const value = Math.hypot(position[0] - pixel[0], position[1] - pixel[1]); if (value < distance) { best = index; distance = value; } });
    return best;
  }
  window.missionEditor = {
    click(event) {
      if (!data) return false;
      const index = nearest(event);
      if (index < 0) return false;
      selectedPoint = index; refresh(); status(`已选中 ${selectedVehicle} 航点 ${index + 1}，可拖动或调整顺序。`); return true;
    }
  };
  async function load() {
    data = await api('/api/mission-waypoints');
    selectedVehicle = data.routes[0]?.vehicle_id || '';
    selectedPoint = -1; refresh(); status('已读取当前规划航点；点击地图上的点可选中。');
  }
  byId('missionVehicle').onchange = event => { selectedVehicle = event.target.value; selectedPoint = -1; refresh(); };
  byId('missionList').onchange = event => { selectedPoint = Number(event.target.value); refresh(); };
  byId('missionLoad').onclick = () => load().catch(error => toast('读取航点失败：' + error.message));
  function mutate(callback) { const current = route(); if (!current || selectedPoint < 0) { toast('请先选择一个航点'); return; } callback(current.points); refresh(); status('航线已修改，点击“保存航点草稿”保存'); }
  byId('missionDelete').onclick = () => mutate(points => { points.splice(selectedPoint, 1); selectedPoint = Math.min(selectedPoint, points.length - 1); });
  byId('missionUp').onclick = () => mutate(points => { if (selectedPoint > 0) { [points[selectedPoint - 1], points[selectedPoint]] = [points[selectedPoint], points[selectedPoint - 1]]; selectedPoint--; } });
  byId('missionDown').onclick = () => mutate(points => { if (selectedPoint < points.length - 1) { [points[selectedPoint + 1], points[selectedPoint]] = [points[selectedPoint], points[selectedPoint + 1]]; selectedPoint++; } });
  function insert(after) { const current = route(); if (!current) return; const reference = current.points[selectedPoint]?.point || [...current.home, 0]; const point = [Number(reference[0]) + (after ? 10 : -10), Number(reference[1]), Number(reference[2] || plan.vehicles.find(item => item.id === selectedVehicle)?.entry_altitude_m || 40.5)]; const item = {id: `manual:${selectedVehicle}:${Date.now()}`, state: 'MANUAL', point, editable: true, generated: false}; const index = selectedPoint < 0 ? current.points.length : selectedPoint + (after ? 1 : 0); current.points.splice(index, 0, item); selectedPoint = index; refresh(); status('已插入航点，拖动到准确位置后保存'); }
  byId('missionAddBefore').onclick = () => insert(false);
  byId('missionAddAfter').onclick = () => insert(true);
  byId('missionUpdate').onclick = () => mutate(points => { const values = byId('missionPoint').value.split(',').map(Number); if (values.length !== 3 || values.some(value => !Number.isFinite(value))) throw new Error('请输入 east,north,altitude'); points[selectedPoint].point = values; });
  byId('missionReset').onclick = async () => { try { data = await api('/api/mission-waypoints?reset=1'); selectedVehicle = data.routes[0]?.vehicle_id || ''; selectedPoint = -1; refresh(); status('已恢复原始规划航点，尚未保存'); } catch (error) { toast('恢复原规划失败：' + error.message); } };
  byId('missionSave').onclick = async () => { try { if (!data) await load(); data = await api('/api/mission-waypoints', data); status('航点草稿已保存；应用前仍需重新规划和重新校验。'); } catch (error) { toast('保存航点失败：' + error.message); } };
  byId('missionApply').onclick = async () => { try { await control('replan', {use_manual_draft: false}); await load(); status('已按正式场地区域配置重新规划；航点编辑草稿未直接执行。'); } catch (error) { toast('重新规划失败：' + error.message); } };
  canvas.addEventListener('pointerdown', event => { if (!data || event.button !== 0) return; const index = nearest(event); if (index < 0) return; selectedPoint = index; dragging = {vehicle: selectedVehicle, point: index}; drag = null; canvas.setPointerCapture(event.pointerId); event.stopImmediatePropagation(); }, true);
  canvas.addEventListener('pointermove', event => { if (!dragging) return; const current = route(); const position = enu(event); current.points[dragging.point].point[0] = position[0]; current.points[dragging.point].point[1] = position[1]; refresh(); event.stopImmediatePropagation(); }, true);
  function finishDrag(event) { if (!dragging) return; dragging = null; status('航点已拖动，点击保存航点草稿'); event.stopImmediatePropagation(); }
  canvas.addEventListener('pointerup', finishDrag, true); canvas.addEventListener('pointercancel', finishDrag, true);
  const originalDraw = draw;
  draw = function() { originalDraw(); if (!data) return; context.save(); data.routes.forEach(item => { const points = item.points.map(point => screen(point.point)); if (!points.length) return; context.beginPath(); points.forEach((point, index) => index ? context.lineTo(...point) : context.moveTo(...point)); context.strokeStyle = item.vehicle_id === selectedVehicle ? '#ffffff' : '#9aabc0'; context.lineWidth = item.vehicle_id === selectedVehicle ? 2.5 : 1; context.setLineDash([5, 4]); context.stroke(); context.setLineDash([]); points.forEach((point, index) => { context.beginPath(); context.arc(...point, item.vehicle_id === selectedVehicle && index === selectedPoint ? 8 : 4, 0, Math.PI * 2); context.fillStyle = item.vehicle_id === selectedVehicle ? '#ffffff' : '#9aabc0'; context.fill(); context.fillStyle = '#e9f1ff'; context.font = '11px sans-serif'; context.fillText(`${item.vehicle_id.replace('uav_', 'U')}-${index + 1}`, point[0] + 8, point[1] - 6); }); }); context.restore(); };
  const wait = setInterval(() => { if (!plan || initialized) return; initialized = true; clearInterval(wait); load().catch(error => status('航点读取失败：' + error.message)); }, 150);
})();
