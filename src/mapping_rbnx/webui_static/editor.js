// SPDX-License-Identifier: MulanPSL-2.0
const $ = id => document.getElementById(id);
const cv = $('map');
const ctx = cv.getContext('2d', {willReadFrequently: true});
const vp = $('viewport');
const mapId = new URLSearchParams(location.search).get('map_id');
const api = name => '/api/editor/' + name + '?map_id=' + encodeURIComponent(mapId || '');
const status = text => { $('status').textContent = text; };
let meta, tool = 'wall', zoom = 1, tx = 0, ty = 0;
let drag = null, space = false, undo = [], redo = [], dirty = false, busy = true;

function render() {
  cv.style.transform = `translate(${tx}px,${ty}px) scale(${zoom})`;
}
function fit() {
  if (!cv.width || !cv.height) return;
  zoom = Math.max(.01, Math.min((vp.clientWidth - 48) / cv.width, (vp.clientHeight - 48) / cv.height));
  tx = (vp.clientWidth - cv.width * zoom) / 2;
  ty = (vp.clientHeight - cv.height * zoom) / 2;
  render();
}
function point(e) {
  const r = cv.getBoundingClientRect();
  return {x: (e.clientX - r.left) / zoom, y: (e.clientY - r.top) / zoom};
}
function world(p) {
  const a = meta.origin[2], x = p.x * meta.resolution, y = (cv.height - p.y) * meta.resolution;
  return {x: meta.origin[0] + Math.cos(a) * x - Math.sin(a) * y,
          y: meta.origin[1] + Math.sin(a) * x + Math.cos(a) * y};
}
const snapshot = () => ctx.getImageData(0, 0, cv.width, cv.height);
function pushUndo() {
  undo.push(snapshot());
  // At most 64 MiB of history, including maps with millions of cells.
  const count = Math.max(1, Math.min(30, Math.floor(64 * 1024 * 1024 / (cv.width * cv.height * 4))));
  if (undo.length > count) undo.shift();
  redo = [];
  dirty = true;
}
function color() {
  const v = meta.palette[tool === 'free' ? 'free' : tool === 'unknown' ? 'unknown' : 'wall'];
  return `rgb(${v},${v},${v})`;
}
function dot(p) {
  const radius = Math.max(.5, Number($('width').value) / meta.resolution / 2);
  ctx.fillStyle = color();
  for (let y = Math.max(0, Math.floor(p.y-radius)); y < Math.min(cv.height, Math.ceil(p.y+radius)); y++) {
    for (let x = Math.max(0, Math.floor(p.x-radius)); x < Math.min(cv.width, Math.ceil(p.x+radius)); x++) {
      if ((x+.5-p.x)**2 + (y+.5-p.y)**2 <= radius**2) ctx.fillRect(x,y,1,1);
    }
  }
}
function stroke(a,b) {
  const steps = Math.max(1, Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)*2));
  for (let i=0; i<=steps; i++) dot({x:a.x+(b.x-a.x)*i/steps,y:a.y+(b.y-a.y)*i/steps});
}
function shape(a,b) {
  if (tool === 'rect') {
    ctx.fillStyle = color();
    ctx.fillRect(Math.floor(Math.min(a.x,b.x)),Math.floor(Math.min(a.y,b.y)),
                 Math.max(1,Math.ceil(Math.abs(b.x-a.x))),Math.max(1,Math.ceil(Math.abs(b.y-a.y))));
  } else stroke(a,b);
}
function exports() {
  for (const kind of ['pgm','yaml']) {
    $('export-'+kind).href = api('export.'+kind)+'&revision='+encodeURIComponent(meta.version);
  }
}
async function load() {
  busy = true;
  try {
    const response = await fetch(api('meta'), {cache: 'no-store'});
    const next = await response.json();
    if (!response.ok) throw Error(next.detail || '无法载入地图');
    const img = new Image();
    await new Promise((resolve,reject) => {
      img.onload=resolve; img.onerror=()=>reject(Error('地图图像载入失败'));
      img.src=api('map.png')+'&revision='+encodeURIComponent(next.version)+'&t='+Date.now();
    });
    meta=next;
    $('save').textContent=meta.auto_apply?'保存并应用栅格':'保存编辑版本（导出使用）';
    $('usage').textContent=meta.auto_apply?
      '此图载入 slam_toolbox 后，编辑栅格用于导航和下一次初始定位。持续定位使用原位姿图；保存不会启动机器人或重新定位。':
      '此后端暂未接入自动应用。可编辑并导出栅格供其他工具使用；原图和建图数据保留。';
    for (const key of ['wall','free','unknown']) {
      const v=meta.palette[key]; $('legend-'+key).style.background=`rgb(${v},${v},${v})`;
    }
    cv.width=meta.width; cv.height=meta.height;
    ctx.drawImage(img,0,0); undo=[]; redo=[]; dirty=false;
    $('mapname').textContent=`${meta.map_id} · ${Number(meta.resolution).toFixed(2)} m / 像素`;
    $('version').textContent='当前版本：'+meta.version;
    fit(); exports(); status('选择工具开始编辑；原图和位姿图保持不变');
  } finally { busy=false; }
}
$('tools').onclick=e=>{
  const b=e.target.closest('button[data-tool]');
  if (!b || drag) return;
  tool=b.dataset.tool;
  for (const x of $('tools').children) x.classList.toggle('active',x===b);
  cv.style.cursor=tool==='pan'?'grab':'crosshair';
};
$('width').oninput=()=>{ $('widthlabel').textContent=Number($('width').value).toFixed(2)+' m'; };
vp.onpointerdown=e=>{
  if (busy || !meta || (e.button!==0 && e.button!==1)) return;
  const p=point(e);
  if (tool==='pan' || space || e.button===1) drag={pan:true,x:e.clientX,y:e.clientY,tx,ty};
  else {
    if (p.x<0 || p.y<0 || p.x>=cv.width || p.y>=cv.height) return;
    pushUndo(); drag={start:p,last:p,base:snapshot()};
    if (!['line','rect'].includes(tool)) dot(p);
    status('有未保存修改');
  }
  vp.setPointerCapture(e.pointerId); e.preventDefault();
};
vp.onpointermove=e=>{
  if (!meta) return;
  const p=point(e), w=world(p);
  $('coords').textContent=p.x>=0&&p.y>=0&&p.x<cv.width&&p.y<cv.height?`x ${w.x.toFixed(2)} m · y ${w.y.toFixed(2)} m`:'';
  if (!drag) return;
  if (drag.pan) { tx=drag.tx+e.clientX-drag.x; ty=drag.ty+e.clientY-drag.y; render(); }
  else {
    if (['line','rect'].includes(tool)) { ctx.putImageData(drag.base,0,0); shape(drag.start,p); }
    else stroke(drag.last,p);
    drag.last=p; status('有未保存修改');
  }
};
vp.onpointerup=e=>{
  if (drag&&!drag.pan&&['line','rect'].includes(tool)) { ctx.putImageData(drag.base,0,0); shape(drag.start,point(e)); }
  drag=null;
};
vp.onpointercancel=()=>{ drag=null; };
vp.addEventListener('wheel',e=>{
  e.preventDefault();
  const r=vp.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
  const next=Math.max(.01,Math.min(20,zoom*Math.exp(-e.deltaY*.001)));
  tx=x-(x-tx)*next/zoom;ty=y-(y-ty)*next/zoom;zoom=next;render();
},{passive:false});
$('fit').onclick=fit;
$('undo').onclick=()=>{
  if (!busy&&!drag&&undo.length) { redo.push(snapshot());ctx.putImageData(undo.pop(),0,0);dirty=true;status('已撤销；保存后应用'); }
};
$('redo').onclick=()=>{
  if (!busy&&!drag&&redo.length) { undo.push(snapshot());ctx.putImageData(redo.pop(),0,0);dirty=true;status('已重做；保存后应用'); }
};
$('revert').onclick=()=>{ if (!busy&&!drag) load().catch(e=>status(e.message)); };
async function post(name,body) {
  const response=await fetch(api(name),{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const value=await response.json();
  if (!response.ok || !value.ok) throw Error(value.detail || '保存失败');
  return value;
}
$('save').onclick=async()=>{
  if (busy || drag || !meta) return;
  busy=true; $('save').disabled=true;
  try {
    const value=await post('save',{png:cv.toDataURL('image/png'),version:meta.version,base_sha256:meta.base_sha256});
    meta.version=value.version;dirty=false;exports();
    $('version').textContent='当前版本：'+value.version;
    status(`已保存${meta.auto_apply?"并应用":"，可导出"} · 修改 ${value.changed_cells} 个栅格 · 原图与位姿图保留`);
  } catch(e) { status(e.message); }
  finally { busy=false; $('save').disabled=false; }
};
$('restore').onclick=async()=>{
  if (busy || drag || !meta) return;
  busy=true;
  try {
    await post('restore',{version:meta.version,base_sha256:meta.base_sha256});
    await load(); status('已恢复原始栅格；历史编辑版本保留');
  } catch(e) { status(e.message); }
  finally { busy=false; }
};
document.addEventListener('keydown',e=>{
  if (e.code==='Space') { space=true;e.preventDefault(); }
  if ((e.ctrlKey||e.metaKey)&&e.key==='z') { e.preventDefault();$(e.shiftKey?'redo':'undo').click(); }
});
document.addEventListener('keyup',e=>{ if(e.code==='Space') space=false; });
window.addEventListener('blur',()=>{space=false;drag=null;});
window.addEventListener('beforeunload',e=>{ if(dirty){e.preventDefault();e.returnValue='';} });
window.addEventListener('resize',fit);
load().catch(e=>status('载入失败：'+e.message));
