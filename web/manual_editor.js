(() => {
  const kinds = {search_area: '搜索区', exclude_area: '禁搜区', waypoints: '航点序列'};
  const panel = document.createElement('section');
  panel.id = 'manualEditor';
  panel.innerHTML = `<h2>手动绘制 / 优化意见</h2>
    <small>仅作为后续优化输入，不改变当前执行任务。区域边界不是飞机必须飞过的路线。</small>
    <label>归属飞机<select id="draftOwner"><option value="unassigned">未指定 / 由优化分配</option>${Array.from({length: 6}, (_, index) => `<option value="uav_${index + 1}">UAV${index + 1}</option>`).join('')}</select></label>
    <label>名称<input id="draftLabel" type="text" maxlength="120" placeholder="如：房区需加密搜索"></label>
    <label>优化备注<textarea id="draftNotes" maxlength="2000" rows="2" placeholder="说明必须经过、希望避开、调整区域等"></textarea></label>
    <div class="buttons"><button data-draft-kind="search_area">画搜索区</button><button data-draft-kind="exclude_area">画禁搜区</button><button data-draft-kind="waypoints">画航点</button></div>
    <div class="buttons"><button id="draftFinish">完成绘制</button><button id="draftUndo">撤销末点</button><button id="draftCancel">取消绘制</button></div>
    <small>左键逐点添加；区域至少3点；航点按点击顺序编号。右键撤销末点，Enter完成，Esc取消。拖空白处平移，滚轮缩放。选中已完成对象后，可拖动顶点。</small>
    <label>已绘制对象<select id="draftList" size="4"></select></label>
    <div class="buttons"><button id="draftUpdate">更新名称 / 归属 / 备注</button><button id="draftDelete">删除选中</button></div>
    <div class="buttons"><button id="draftSave" class="primary">保存到工作空间</button><button id="draftLoad">读取上次保存</button><button id="draftExport">导出JSON</button><button id="draftImport">导入JSON</button></div>
    <input id="draftFile" type="file" accept=".json,application/json" hidden>
    <div id="draftStatus" role="status" class="warning">等待地图加载</div>`;
  document.querySelector('aside').prepend(panel);
  const style = document.createElement('style');
  style.textContent = '#manualEditor input[type=text]{width:100%;background:#0b1323;border:1px solid #34425e;border-radius:6px;color:#e4edfc;padding:9px}#manualEditor select[size]{height:100px}#manualEditor button{font-size:12px}#manualEditor .active{border-color:#56e6da;background:#19635c}#draftStatus{overflow-wrap:anywhere;white-space:pre-line}';
  document.head.append(style);
  let features = [], current = null, selected = -1, vertexDrag = null, loadedOrigin = null;
  let initialized = false, dirty = false, cachedExport = null;
  const status = message => { byId('draftStatus').textContent = message; };
  const key = () => 'zhixin.manualDraft.' + loadedOrigin;
  const payload = () => ({schema: 'zhixin/manual-draft/v1', map_origin_id: loadedOrigin, features});
  function persist() {
    dirty = true;
    cachedExport = null;
    try { localStorage.setItem(key(), JSON.stringify(payload())); }
    catch (error) { status('浏览器暂存失败，请及时保存或导出：' + error.message); }
  }
  function metadata() {
    return {vehicle_id: byId('draftOwner').value, label: byId('draftLabel').value.trim(), notes: byId('draftNotes').value.trim()};
  }
  function refresh() {
    byId('draftList').replaceChildren(...features.map((feature, index) => {
      const option = document.createElement('option');
      option.value = index;
      option.textContent = `${index + 1}. ${feature.label || kinds[feature.kind]} · ${feature.vehicle_id} · ${feature.points_enu_m.length}点`;
      option.selected = index === selected;
      return option;
    }));
    panel.querySelectorAll('[data-draft-kind]').forEach(button => button.classList.toggle('active', button.dataset.draftKind === current?.kind));
    canvas.style.cursor = current ? 'crosshair' : 'grab';
    draw();
  }
  function finish() {
    if (!current) return;
    if (current.points_enu_m.length < (current.kind === 'waypoints' ? 1 : 3)) { toast('区域至少3点，航点至少1点'); return; }
    features.push({...current, ...metadata()});
    selected = features.length - 1;
    current = null;
    persist(); refresh(); status('绘制已完成，尚未保存到工作空间。可继续绘制或保存。');
  }
  function undo() {
    if (current) { current.points_enu_m.pop(); refresh(); status('当前绘制：' + current.points_enu_m.length + '点'); }
  }
  function eventPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return [(event.clientX - rect.left - view.east) / view.scale, (view.north - event.clientY + rect.top) / view.scale];
  }
  window.manualEditor = {
    click(event) {
      if (!current || !plan) return false;
      current.points_enu_m.push(eventPoint(event));
      refresh(); status(kinds[current.kind] + '：已添加' + current.points_enu_m.length + '点，完成后点击“完成绘制”。');
      return true;
    },
    rightClick() { if (!current) return false; undo(); return true; }
  };
  panel.querySelectorAll('[data-draft-kind]').forEach(button => button.onclick = () => {
    if (!plan || loadedOrigin !== plan.map_frame.map_origin_id) { toast('地图尚未加载或原点已变化，请刷新'); return; }
    if (current) { toast('请先完成或取消当前绘制'); return; }
    current = {kind: button.dataset.draftKind, ...metadata(), points_enu_m: []};
    selected = -1; refresh(); status('正在绘制' + kinds[current.kind] + '，左键添加顶点。');
  });
  byId('draftFinish').onclick = finish;
  byId('draftUndo').onclick = undo;
  byId('draftCancel').onclick = () => { current = null; refresh(); status('已取消当前绘制'); };
  byId('draftList').onchange = () => {
    selected = Number(byId('draftList').value);
    const feature = features[selected];
    if (!feature) return;
    byId('draftOwner').value = feature.vehicle_id;
    byId('draftLabel').value = feature.label;
    byId('draftNotes').value = feature.notes;
    draw();
  };
  byId('draftUpdate').onclick = () => {
    if (!features[selected]) return;
    Object.assign(features[selected], metadata()); persist(); refresh(); status('属性已更新，尚未保存到工作空间');
  };
  byId('draftDelete').onclick = () => {
    if (!features[selected]) return;
    features.splice(selected, 1); selected = -1; persist(); refresh(); status('已删除选中对象，尚未保存到工作空间');
  };
  async function save() {
    if (!plan || !initialized) throw new Error('地图尚未加载');
    if (current) throw new Error('请先完成或取消当前绘制');
    const result = await api('/api/manual-draft', payload());
    cachedExport = result.draft;
    features = result.draft.features;
    dirty = false;
    localStorage.setItem(key(), JSON.stringify(result.draft));
    refresh(); status('已保存：' + result.saved_path + '\n' + (result.draft.warnings.join('\n') || '草稿仅供后续优化，不影响当前执行航线。'));
    return result.draft;
  }
  byId('draftSave').onclick = async () => { try { await save(); } catch (error) { toast('保存失败：' + error.message); } };
  function accept(data) {
    if (!plan || data.schema !== 'zhixin/manual-draft/v1' || data.map_origin_id !== loadedOrigin || !Array.isArray(data.features)) throw new Error('文件格式或地图原点不一致');
    if (data.features.length > 200) throw new Error('对象数量超过200');
    for (const feature of data.features) {
      if (!kinds[feature.kind] || !['unassigned', ...plan.vehicles.map(vehicle => vehicle.id)].includes(feature.vehicle_id) || typeof feature.label !== 'string' || typeof feature.notes !== 'string' || !Array.isArray(feature.points_enu_m) || feature.points_enu_m.length < (feature.kind === 'waypoints' ? 1 : 3) || feature.points_enu_m.length > 1000 || feature.points_enu_m.some(point => !Array.isArray(point) || point.length !== 2 || point.some(value => !Number.isFinite(value) || Math.abs(value) > 10000))) throw new Error('文件包含无效绘制对象');
    }
    features = data.features; current = null; selected = -1; refresh();
  }
  byId('draftLoad').onclick = async () => {
    if ((dirty || current) && !confirm('读取上次保存会替换当前未保存草稿，是否继续？')) return;
    try { const data = await api('/api/manual-draft'); accept(data); cachedExport = data; dirty = false; localStorage.setItem(key(), JSON.stringify(data)); status('已读取工作空间上次保存，共' + features.length + '条'); }
    catch (error) { toast('读取失败：' + error.message); }
  };
  byId('draftExport').onclick = async () => {
    try {
      const data = dirty || !cachedExport || current ? await save() : cachedExport;
      const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], {type: 'application/json'}));
      const anchor = document.createElement('a'); anchor.href = url; anchor.download = 'zhixin_manual_' + (data.filename || 'draft.json'); anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) { toast('导出失败：' + error.message); }
  };
  byId('draftImport').onclick = () => byId('draftFile').click();
  byId('draftFile').onchange = async event => {
    const file = event.target.files[0];
    if (!file) return;
    if ((dirty || current) && !confirm('导入会替换当前未保存草稿，是否继续？')) { event.target.value = ''; return; }
    try { if (file.size > 1048576) throw new Error('文件不能大于1MB'); accept(JSON.parse(await file.text())); persist(); status('已导入，点击保存到工作空间进行几何检查'); }
    catch (error) { toast('导入失败：' + error.message); }
    event.target.value = '';
  };
  canvas.addEventListener('pointerdown', event => {
    if (event.button !== 0 || current || !features[selected]) return;
    const rect = canvas.getBoundingClientRect();
    const pixel = [event.clientX - rect.left, event.clientY - rect.top];
    const index = features[selected].points_enu_m.findIndex(point => { const position = screen(point); return Math.hypot(position[0] - pixel[0], position[1] - pixel[1]) < 10; });
    if (index < 0) return;
    vertexDrag = {feature: selected, index}; drag = null; canvas.setPointerCapture(event.pointerId); event.stopImmediatePropagation();
  }, true);
  canvas.addEventListener('pointermove', event => {
    if (!vertexDrag) return;
    features[vertexDrag.feature].points_enu_m[vertexDrag.index] = eventPoint(event); draw(); event.stopImmediatePropagation();
  }, true);
  function endVertex(event) {
    if (!vertexDrag) return;
    vertexDrag = null; persist(); refresh(); status('顶点已调整，尚未保存到工作空间'); event.stopImmediatePropagation();
  }
  canvas.addEventListener('pointerup', endVertex, true);
  canvas.addEventListener('pointercancel', endVertex, true);
  window.addEventListener('keydown', event => {
    if (!current || ['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target.tagName)) return;
    if (event.key === 'Enter') { event.preventDefault(); finish(); }
    if (event.key === 'Escape') byId('draftCancel').click();
    if (event.key === 'Backspace') { event.preventDefault(); undo(); }
  });
  window.addEventListener('beforeunload', event => { if (dirty || current) { event.preventDefault(); event.returnValue = ''; } });
  const originalDraw = draw;
  draw = function() {
    originalDraw();
    if (!plan) return;
    context.save();
    [...features, ...(current ? [current] : [])].forEach((feature, index) => {
      const points = feature.points_enu_m.map(screen);
      if (!points.length) return;
      const color = feature.kind === 'search_area' ? '#49f1e3' : feature.kind === 'exclude_area' ? '#ff618d' : '#fff070';
      context.beginPath(); points.forEach((point, pointIndex) => pointIndex ? context.lineTo(...point) : context.moveTo(...point));
      if (feature.kind !== 'waypoints' && points.length >= 3) { context.closePath(); context.fillStyle = color + '35'; context.fill(); }
      context.strokeStyle = color; context.lineWidth = index === selected ? 3 : 2; context.setLineDash(feature === current ? [6, 4] : []); context.stroke(); context.setLineDash([]);
      points.forEach((point, pointIndex) => { context.beginPath(); context.arc(...point, index === selected ? 6 : 4, 0, Math.PI * 2); context.fillStyle = color; context.fill(); context.font = '12px sans-serif'; context.fillText(String(pointIndex + 1), point[0] + 8, point[1] - 6); });
      context.fillStyle = color; context.fillText(feature.label || kinds[feature.kind], points[0][0] + 8, points[0][1] + 16);
    });
    context.restore();
  };
  const wait = setInterval(async () => {
    if (!plan || initialized) return;
    initialized = true; clearInterval(wait); loadedOrigin = plan.map_frame.map_origin_id;
    try {
      const data = await api('/api/manual-draft');
      if (data.features?.length) { accept(data); cachedExport = data; localStorage.setItem(key(), JSON.stringify(data)); status('已载入' + features.length + '条草稿，可开始绘制'); }
      else { localStorage.removeItem(key()); accept(data); cachedExport = data; status('当前没有手绘叠加，可开始绘制'); }
    } catch (error) { status('草稿读取失败，可重新绘制：' + error.message); }
  }, 100);
})();
