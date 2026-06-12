# ─────────────────────────────────────────────
#  param_server.py  —  Flask 参数实时调节服务器
# ─────────────────────────────────────────────

import logging
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

_R_CONST  = Rotation.from_euler('y',  np.pi / 2).as_matrix()
_R_Z_POS  = Rotation.from_euler('z',  np.pi / 2).as_matrix()
_R_Z_NEG  = Rotation.from_euler('z', -np.pi / 2).as_matrix()


def _wait_rotation(state, target_rot: np.ndarray,
                   timeout: float = 6.0, tol: float = 0.08) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if state.rotation_error_magnitude(target_rot) < tol:
            return True
        time.sleep(0.05)
    return False


def _wait_height(state, target_z: float,
                 timeout: float = 10.0, tol: float = 0.005) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        with state._lock:
            cur_z = state._smooth_position[2]
        if abs(cur_z - target_z) < tol:
            return True
        time.sleep(0.05)
    return False


_HTML = """\
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{{ arm_name }} 参数调节</title>
<style>
:root{--accent:#007AFF;--bg:#F5F5F7;--card:#FFF;--text:#1D1D1F;--muted:#86868B;--border:#D2D2D7}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;padding:28px 20px;max-width:1020px;margin:0 auto}
h1{font-size:1.5rem;font-weight:700;margin-bottom:4px}
.sub{color:var(--muted);font-size:.88rem;margin-bottom:24px}
.layout{display:grid;grid-template-columns:1fr 1fr;gap:16px;align-items:start}
@media(max-width:680px){.layout{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:16px}
.card{background:var(--card);border-radius:16px;padding:20px 20px 4px;box-shadow:0 1px 4px rgba(0,0,0,.08)}
.sec{font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:16px}
.param{margin-bottom:20px}
.prow{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:6px}
.plabel{font-weight:500;font-size:.9rem}
.pval{font-family:"SF Mono","Fira Code",monospace;font-size:1rem;color:var(--accent);font-weight:600;min-width:60px;text-align:right}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:4px;border-radius:2px;
  background:linear-gradient(to right,var(--accent) 0%,var(--accent) var(--p,0%),var(--border) var(--p,0%),var(--border) 100%);
  outline:none;cursor:pointer;margin-bottom:4px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;
  background:var(--accent);cursor:pointer;box-shadow:0 1px 6px rgba(0,122,255,.35)}
.rlabels{display:flex;justify-content:space-between;color:var(--muted);font-size:.74rem}
.detail{color:var(--muted);font-size:.74rem;margin-top:3px;font-family:monospace}
.footer{display:flex;gap:12px;align-items:center;margin-top:16px}
.reset{flex:1;padding:13px;background:#F5F5F7;border:1.5px solid var(--border);
  border-radius:10px;font-size:.93rem;font-weight:500;cursor:pointer;color:var(--text);transition:background .1s}
.reset:hover{background:#E8E8EA}
#st{color:var(--muted);font-size:.8rem;min-height:18px}
.mode-group{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:4px}
.mode-opt{border:2px solid var(--border);border-radius:12px;padding:10px 6px;cursor:pointer;
  text-align:center;transition:border-color .15s,background .15s;user-select:none}
.mode-opt:hover{border-color:var(--accent)}
.mode-opt.active{border-color:var(--accent);background:rgba(0,122,255,.08)}
.mode-title{font-weight:600;font-size:.85rem}
.mode-desc{color:var(--muted);font-size:.70rem;margin-top:3px;line-height:1.3}
.action-btn{display:block;width:100%;padding:13px 14px;background:var(--card);
  border:2px solid var(--border);border-radius:12px;cursor:pointer;text-align:left;
  transition:border-color .15s,background .15s;margin-bottom:10px}
.action-btn:last-child{margin-bottom:0}
.action-btn:hover:not(:disabled){border-color:var(--accent);background:rgba(0,122,255,.06)}
.action-btn:disabled{opacity:.5;cursor:not-allowed}
.action-btn .an{display:block;font-weight:600;font-size:.92rem;color:var(--text)}
.action-btn .ad{display:block;font-size:.73rem;color:var(--muted);margin-top:3px}
</style>
</head>
<body>
<h1>{{ arm_name }}</h1>
<p class="sub">参数实时调节 · 拖动滑块即时生效</p>

<div class="layout">

<!-- ── 左栏：所有滑块 ── -->
<div class="col">

  <div class="card">
    <div class="sec">缩放参数</div>
    <div class="param">
      <div class="prow">
        <span class="plabel">位移缩放 <code>translation_m</code></span>
        <span class="pval" id="v-tr">{{ "%.3f"|format(tr_def) }}</span>
      </div>
      <input type="range" id="s-tr" min="0.01" max="5.0" step="0.01" value="{{ tr_def }}">
      <div class="rlabels"><span>0.01</span><span>5.0</span></div>
    </div>
    <div class="param">
      <div class="prow">
        <span class="plabel">旋转缩放 <code>rotation_rad</code></span>
        <span class="pval" id="v-ro">{{ "%.3f"|format(ro_def) }}</span>
      </div>
      <input type="range" id="s-ro" min="0.01" max="5.0" step="0.01" value="{{ ro_def }}">
      <div class="rlabels"><span>0.01</span><span>5.0</span></div>
    </div>
    <div class="param">
      <div class="prow">
        <span class="plabel">夹爪速度 <code>gripper_vel</code></span>
        <span class="pval" id="v-gv">{{ "%.1f"|format(gv_def) }} rad/s</span>
      </div>
      <input type="range" id="s-gv" min="0.1" max="10.0" step="0.1" value="{{ gv_def }}">
      <div class="rlabels"><span>0.1</span><span>10.0 rad/s</span></div>
    </div>
    <div class="param">
      <div class="prow">
        <span class="plabel">精细平动 <code>fine_scale</code></span>
        <span class="pval" id="v-fs">{{ "%.3f"|format(fs_def) }} m/s</span>
      </div>
      <input type="range" id="s-fs" min="0.01" max="0.5" step="0.01" value="{{ fs_def }}">
      <div class="rlabels"><span>0.01</span><span>0.5 m/s</span></div>
    </div>
    <div class="param">
      <div class="prow">
        <span class="plabel">精细旋转 <code>fine_rotation_scale</code></span>
        <span class="pval" id="v-fr">{{ "%.2f"|format(fr_def) }} rad/s</span>
      </div>
      <input type="range" id="s-fr" min="0.05" max="3.0" step="0.05" value="{{ fr_def }}">
      <div class="rlabels"><span>0.05</span><span>3.0 rad/s</span></div>
    </div>
  </div>

  <div class="card">
    <div class="sec">Smooth Target PD</div>
    <div class="param">
      <div class="prow">
        <span class="plabel">跟踪增益 <code>tracking_gain_hz</code></span>
        <span class="pval" id="v-tg">{{ "%.1f"|format(tg_def) }} Hz</span>
      </div>
      <input type="range" id="s-tg" min="1" max="200" step="1" value="{{ tg_def }}">
      <div class="rlabels"><span>1 Hz</span><span>200 Hz</span></div>
      <div class="detail">时间常数 ≈ {{ "%.3f"|format(1.0/tg_def) }} s</div>
    </div>
    <div class="param">
      <div class="prow">
        <span class="plabel">阻尼比 <code>damping_ratio</code></span>
        <span class="pval" id="v-dr">{{ "%.2f"|format(dr_def) }}</span>
      </div>
      <input type="range" id="s-dr" min="0.1" max="2.0" step="0.05" value="{{ dr_def }}">
      <div class="rlabels"><span>0.1（欠阻尼）</span><span>2.0（过阻尼）</span></div>
      <div class="detail">1.0 = 临界阻尼</div>
    </div>
  </div>

</div><!-- /左栏 -->

<!-- ── 右栏：模式 / 动作 / 位姿 ── -->
<div class="col">

  <div class="card">
    <div class="sec">运动模式</div>
    <div class="mode-group">
      <div class="mode-opt active" id="opt-0" onclick="setMode(0)">
        <div class="mode-title">自由</div>
        <div class="mode-desc">平动 + 全向旋转</div>
      </div>
      <div class="mode-opt" id="opt-1" onclick="setMode(1)">
        <div class="mode-title">约束旋转</div>
        <div class="mode-desc">Ry90° 基准<br>仅 z 轴旋转</div>
      </div>
      <div class="mode-opt" id="opt-2" onclick="setMode(2)">
        <div class="mode-title">仅平动</div>
        <div class="mode-desc">Ry90° 基准<br>无旋转</div>
      </div>
      <div class="mode-opt" id="opt-3" onclick="setMode(3)">
        <div class="mode-title">Z旋转</div>
        <div class="mode-desc">无初始旋转<br>仅 z 轴旋转</div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="sec">操纵杆精细控制</div>
    <div class="mode-group" style="grid-template-columns:1fr 1fr 1fr">
      <div class="mode-opt active" id="fopt-0" onclick="setFineMode(0)">
        <div class="mode-title">XY 平动</div>
        <div class="mode-desc">世界坐标系<br>x/y 平移</div>
      </div>
      <div class="mode-opt" id="fopt-1" onclick="setFineMode(1)">
        <div class="mode-title">Z 旋转</div>
        <div class="mode-desc">世界坐标系<br>绕 z 轴</div>
      </div>
      <div class="mode-opt" id="fopt-2" onclick="setFineMode(2)">
        <div class="mode-title">EEF 平动</div>
        <div class="mode-desc">末端坐标系<br>x/y 平移</div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="sec">预制动作</div>
    <button class="action-btn" id="btn-screw" onclick="runAction('screw_cap','btn-screw')">
      <span class="an">扭瓶盖 ×3</span>
      <span class="ad">对齐 Ry90° → 合爪 → z+90° → 松爪 → z−90°，重复三次</span>
    </button>
  </div>

  <div class="card">
    <div class="sec">已保存位姿 / 高度</div>
    <div style="display:flex;gap:8px;margin-bottom:8px">
      <input type="text" id="pose-name" placeholder="输入名称"
        style="flex:1;padding:9px 10px;border:1.5px solid var(--border);border-radius:8px;
               font-size:.88rem;outline:none;color:var(--text);background:var(--bg)">
      <button onclick="savePose()"
        style="padding:9px 12px;background:var(--accent);color:#fff;border:none;
               border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer;white-space:nowrap">
        保存位姿
      </button>
      <button onclick="saveHeight()"
        style="padding:9px 12px;background:#34C759;color:#fff;border:none;
               border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer;white-space:nowrap">
        保存高度
      </button>
    </div>
    <div id="pose-list"></div>
  </div>

</div><!-- /右栏 -->
</div><!-- /layout -->

<div class="footer">
  <button class="reset" onclick="resetAll()">↩ 恢复默认值</button>
  <div id="st"></div>
</div>

<script>
const DEF = { tr:{{ tr_def }}, ro:{{ ro_def }}, tg:{{ tg_def }}, dr:{{ dr_def }}, fs:{{ fs_def }}, fr:{{ fr_def }}, gv:{{ gv_def }} };
const IDS  = ['tr','ro','tg','dr','fs','fr','gv'];
const KEYS = { tr:'translation_m', ro:'rotation_rad', tg:'tracking_gain_hz', dr:'damping_ratio', fs:'fine_scale', fr:'fine_rotation_scale', gv:'gripper_vel' };
const UNIT = { tr:'', ro:'', tg:' Hz', dr:'', fs:' m/s', fr:' rad/s', gv:' rad/s' };
const FMT  = (v, id) => id==='tg' ? v.toFixed(1) : (id==='dr'||id==='fr') ? v.toFixed(2) : id==='gv' ? v.toFixed(1) : v.toFixed(3);

function track(el){
  const p=(el.value-el.min)/(el.max-el.min)*100;
  el.style.setProperty('--p',p+'%');
}
function setStatus(msg){ document.getElementById('st').textContent=msg; }

async function send(payload){
  try{
    await fetch('/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    setStatus('已更新 '+new Date().toLocaleTimeString());
  }catch(e){ setStatus('失败: '+e.message); }
}

IDS.forEach(id=>{
  const el=document.getElementById('s-'+id);
  track(el);
  el.addEventListener('input',()=>{
    const v=parseFloat(el.value);
    track(el);
    document.getElementById('v-'+id).textContent=FMT(v,id)+UNIT[id];
    send({[KEYS[id]]:v});
  });
});

function updateModeUI(n){
  [0,1,2,3].forEach(i=>document.getElementById('opt-'+i).classList.toggle('active',i===n));
}
function setMode(n){
  updateModeUI(n);
  send({motion_mode:n});
}

function updateFineModeUI(n){
  [0,1,2].forEach(i=>document.getElementById('fopt-'+i).classList.toggle('active',i===n));
}
function setFineMode(n){
  updateFineModeUI(n);
  send({fine_mode:n});
}

fetch('/params').then(r=>r.json()).then(d=>{
  if(d.motion_mode!==undefined) updateModeUI(d.motion_mode);
  if(d.fine_mode!==undefined) updateFineModeUI(d.fine_mode);
  ['tr','ro','tg','dr','fs','fr'].forEach(id=>{
    const key=KEYS[id], el=document.getElementById('s-'+id);
    if(d[key]!==undefined){ el.value=d[key]; track(el);
      document.getElementById('v-'+id).textContent=FMT(d[key],id)+UNIT[id]; }
  });
}).catch(()=>{});

function resetAll(){
  IDS.forEach(id=>{
    const el=document.getElementById('s-'+id);
    el.value=DEF[id]; track(el);
    document.getElementById('v-'+id).textContent=FMT(DEF[id],id)+UNIT[id];
  });
  send({translation_m:DEF.tr,rotation_rad:DEF.ro,tracking_gain_hz:DEF.tg,damping_ratio:DEF.dr,fine_scale:DEF.fs,fine_rotation_scale:DEF.fr,gripper_vel:DEF.gv});
}

let _actionPoll = null;
async function runAction(name, btnId){
  const btn = document.getElementById(btnId);
  const origName = btn.querySelector('.an').textContent;
  btn.disabled = true;
  btn.querySelector('.an').textContent = '执行中…';
  setStatus('动作开始');
  try {
    const r = await fetch('/actions/'+name, {method:'POST'});
    const d = await r.json();
    if(!d.ok){ setStatus('启动失败: '+(d.error||'')); resetActionBtn(btn,origName); return; }
    _actionPoll = setInterval(async ()=>{
      try {
        const s = await (await fetch('/action_status')).json();
        if(!s.running){
          clearInterval(_actionPoll); _actionPoll=null;
          resetActionBtn(btn,origName);
          setStatus('动作完成 '+new Date().toLocaleTimeString());
        }
      } catch { clearInterval(_actionPoll); _actionPoll=null; resetActionBtn(btn,origName); }
    }, 400);
  } catch(e){ setStatus('请求失败: '+e.message); resetActionBtn(btn,origName); }
}
function resetActionBtn(btn,name){ btn.disabled=false; btn.querySelector('.an').textContent=name; }

function _esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');}

function renderPoses(poses){
  const el=document.getElementById('pose-list');
  const entries=Object.entries(poses||{});
  if(!entries.length){
    el.innerHTML='<div style="color:var(--muted);font-size:.85rem;text-align:center;padding:8px 0">暂无保存的位姿 / 高度</div>';
    return;
  }
  el.innerHTML=entries.map(([name,val])=>{
    const isHeight = val && val.type==='height';
    const tag = isHeight
      ? `<span style="color:#34C759;font-size:.72rem;font-weight:600;margin-left:6px">高度 ${val.z.toFixed(3)}m</span>`
      : `<span style="color:var(--muted);font-size:.72rem;margin-left:6px">位姿</span>`;
    const gotoFn = isHeight ? 'gotoHeightBtn(this)' : 'gotoPoseBtn(this)';
    const btnColor = isHeight ? '#34C759' : 'var(--accent)';
    return `
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:8px">
      <div style="flex:1;padding:10px 12px;border:1.5px solid var(--border);border-radius:8px;
                  font-size:.9rem;font-weight:500;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">
        ${_esc(name)}${tag}</div>
      <button data-name="${_esc(name)}" onclick="${gotoFn}"
        style="padding:10px 14px;background:${btnColor};color:#fff;border:none;
               border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer;white-space:nowrap">前往</button>
      <button data-name="${_esc(name)}" onclick="deletePoseBtn(this)"
        style="padding:10px 14px;background:var(--card);color:#FF3B30;border:1.5px solid #FF3B30;
               border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer">删除</button>
    </div>`;
  }).join('');
}

async function loadPoses(){
  try{ const d=await(await fetch('/poses')).json(); renderPoses(d.poses||{}); }catch{}
}

async function savePose(){
  const name=document.getElementById('pose-name').value.trim();
  if(!name){setStatus('请输入名称');return;}
  try{
    const r=await fetch('/poses',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
    const d=await r.json();
    if(d.ok){document.getElementById('pose-name').value='';loadPoses();setStatus('已保存位姿: '+name);}
    else setStatus('保存失败: '+(d.error||''));
  }catch(e){setStatus('请求失败: '+e.message);}
}

async function saveHeight(){
  const name=document.getElementById('pose-name').value.trim();
  if(!name){setStatus('请输入名称');return;}
  try{
    const r=await fetch('/heights',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
    const d=await r.json();
    if(d.ok){document.getElementById('pose-name').value='';loadPoses();setStatus('已保存高度: '+name);}
    else setStatus('保存失败: '+(d.error||''));
  }catch(e){setStatus('请求失败: '+e.message);}
}

async function deletePoseBtn(btn){
  const name=btn.dataset.name;
  try{ await fetch('/poses/'+encodeURIComponent(name),{method:'DELETE'}); loadPoses(); }
  catch(e){ setStatus('删除失败: '+e.message); }
}

async function _gotoAction(btn, url, label){
  btn.disabled=true;
  setStatus('前往: '+label);
  try{
    const r=await fetch(url,{method:'POST'});
    const d=await r.json();
    if(!d.ok){setStatus('启动失败: '+(d.error||''));btn.disabled=false;return;}
    const poll=setInterval(async()=>{
      try{
        const s=await(await fetch('/action_status')).json();
        if(!s.running){clearInterval(poll);btn.disabled=false;setStatus('已到达: '+label+' '+new Date().toLocaleTimeString());}
      }catch{clearInterval(poll);btn.disabled=false;}
    },300);
  }catch(e){setStatus('请求失败: '+e.message);btn.disabled=false;}
}

async function gotoPoseBtn(btn){
  _gotoAction(btn,'/actions/goto_pose/'+encodeURIComponent(btn.dataset.name),btn.dataset.name);
}

async function gotoHeightBtn(btn){
  _gotoAction(btn,'/actions/goto_height/'+encodeURIComponent(btn.dataset.name),btn.dataset.name);
}

loadPoses();
</script>
</body>
</html>
"""


def start(state, robot, port: int, arm_name: str, poses_file: str = ""):
    """在 daemon 线程中启动参数调节服务器。"""
    try:
        from flask import Flask, request, jsonify, render_template_string
    except ImportError:
        print("[ParamServer] flask 未安装，跳过参数调节服务器 (pip install flask)")
        return

    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    import json
    import os

    from panthera_config import TRANSLATION_SCALE, ROTATION_SCALE, TRACKING_GAIN_HZ, DAMPING_RATIO, FINE_SCALE, FINE_ROTATION_SCALE

    poses = {}
    if poses_file and os.path.exists(poses_file):
        try:
            with open(poses_file) as f:
                raw = json.load(f)
            # 向后兼容：旧格式值为 list（关节角）→ 转为 typed dict
            poses = {
                k: ({"type": "joint_pose", "joint_pos": v} if isinstance(v, list) else v)
                for k, v in raw.items()
            }
        except Exception as e:
            print(f"[ParamServer] 加载位姿文件失败: {e}")

    def _persist_poses():
        if not poses_file:
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(poses_file)), exist_ok=True)
            with open(poses_file, "w") as f:
                json.dump(poses, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[ParamServer] 保存位姿文件失败: {e}")

    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(
            _HTML,
            arm_name=arm_name,
            tr_def=TRANSLATION_SCALE,
            ro_def=ROTATION_SCALE,
            tg_def=TRACKING_GAIN_HZ,
            dr_def=DAMPING_RATIO,
            fs_def=FINE_SCALE,
            fr_def=FINE_ROTATION_SCALE,
            gv_def=state.gripper_vel,
        )

    @app.route("/params", methods=["GET"])
    def get_params():
        return jsonify({
            "translation_m":       state.translation_scale,
            "rotation_rad":        state.rotation_scale,
            "tracking_gain_hz":    state.tracking_gain_hz,
            "damping_ratio":       state.damping_ratio,
            "motion_mode":         state.motion_mode,
            "fine_mode":           state.fine_mode,
            "fine_scale":          state.fine_scale,
            "fine_rotation_scale": state.fine_rotation_scale,
            "gripper_vel":         state.gripper_vel,
        })

    @app.route("/params", methods=["POST"])
    def update_params():
        data = request.get_json(force=True) or {}
        if "translation_m" in data:
            state.translation_scale = float(data["translation_m"])
        if "rotation_rad" in data:
            state.rotation_scale = float(data["rotation_rad"])
        if "tracking_gain_hz" in data:
            state.tracking_gain_hz = float(data["tracking_gain_hz"])
        if "damping_ratio" in data:
            state.damping_ratio = float(data["damping_ratio"])
        if "motion_mode" in data:
            state.set_motion_mode(int(data["motion_mode"]))
        if "fine_mode" in data:
            with state._lock:
                state.fine_mode = int(data["fine_mode"])
        if "fine_scale" in data:
            with state._lock:
                state.fine_scale = float(data["fine_scale"])
        if "fine_rotation_scale" in data:
            with state._lock:
                state.fine_rotation_scale = float(data["fine_rotation_scale"])
        if "gripper_vel" in data:
            with state._lock:
                state.gripper_vel = float(data["gripper_vel"])
        return jsonify({"ok": True})

    action_running = [False]

    def _run_screw_cap_thread():
        try:
            state.action_locked = True
            with state._lock:
                state.target_rotation = _R_CONST.copy()
            _wait_rotation(state, _R_CONST, timeout=6.0)
            for _ in range(3):
                state.gripper_cmd = 0.0
                time.sleep(1.5)
                target = _R_Z_POS @ _R_CONST
                with state._lock:
                    state.target_rotation = target.copy()
                _wait_rotation(state, target)
                state.gripper_cmd = 2.0
                time.sleep(3.0)
                with state._lock:
                    state.target_rotation = _R_CONST.copy()
                _wait_rotation(state, _R_CONST)
        finally:
            state.action_locked = False
            action_running[0] = False

    @app.route("/actions/screw_cap", methods=["POST"])
    def action_screw_cap():
        if action_running[0]:
            return jsonify({"ok": False, "error": "already running"}), 409
        action_running[0] = True
        threading.Thread(target=_run_screw_cap_thread, daemon=True,
                         name="ScrewCapAction").start()
        return jsonify({"ok": True})

    @app.route("/action_status", methods=["GET"])
    def action_status():
        return jsonify({"running": action_running[0]})

    @app.route("/poses", methods=["GET"])
    def get_poses():
        return jsonify({"poses": poses})

    @app.route("/poses", methods=["POST"])
    def save_pose():
        data = request.get_json(force=True) or {}
        name = data.get("name", "").strip()
        if not name:
            return jsonify({"ok": False, "error": "name required"}), 400
        joint_pos = list(state.last_valid_joint_pos)
        poses[name] = {"type": "joint_pose", "joint_pos": joint_pos}
        _persist_poses()
        print(f"[ParamServer] 保存位姿 '{name}': {[f'{v:.3f}' for v in joint_pos]}")
        return jsonify({"ok": True})

    @app.route("/heights", methods=["POST"])
    def save_height():
        data = request.get_json(force=True) or {}
        name = data.get("name", "").strip()
        if not name:
            return jsonify({"ok": False, "error": "name required"}), 400
        with state._lock:
            z = float(state.fk_position[2])
        poses[name] = {"type": "height", "z": z}
        _persist_poses()
        print(f"[ParamServer] 保存高度 '{name}': z={z:.4f} m")
        return jsonify({"ok": True})

    @app.route("/poses/<name>", methods=["DELETE"])
    def delete_pose(name):
        poses.pop(name, None)
        _persist_poses()
        return jsonify({"ok": True})

    def _run_goto_pose_thread(joint_pos):
        try:
            state.request_goto_joint_pos(joint_pos)
            t0 = time.time()
            while state.action_locked and time.time() - t0 < 30.0:
                time.sleep(0.1)
        finally:
            action_running[0] = False

    @app.route("/actions/goto_pose/<name>", methods=["POST"])
    def action_goto_pose(name):
        if action_running[0]:
            return jsonify({"ok": False, "error": "already running"}), 409
        if name not in poses:
            return jsonify({"ok": False, "error": "pose not found"}), 404
        entry = poses[name]
        if isinstance(entry, dict):
            if entry.get("type") != "joint_pose":
                return jsonify({"ok": False, "error": "not a joint pose"}), 400
            joint_pos = entry["joint_pos"]
        else:
            joint_pos = entry  # 旧格式兼容
        action_running[0] = True
        threading.Thread(target=_run_goto_pose_thread, args=(list(joint_pos),),
                         daemon=True, name="GotoPoseAction").start()
        return jsonify({"ok": True})

    def _run_goto_height_thread(z: float):
        try:
            if not state.request_goto_height_z(z):
                print("[ParamServer] goto_height: 机械臂未校准，跳过")
                return
            _wait_height(state, z, timeout=10.0)
        finally:
            state.action_locked = False
            action_running[0] = False

    @app.route("/actions/goto_height/<name>", methods=["POST"])
    def action_goto_height(name):
        if action_running[0]:
            return jsonify({"ok": False, "error": "already running"}), 409
        if name not in poses:
            return jsonify({"ok": False, "error": "height not found"}), 404
        entry = poses[name]
        if not isinstance(entry, dict) or entry.get("type") != "height":
            return jsonify({"ok": False, "error": "not a height entry"}), 400
        action_running[0] = True
        threading.Thread(target=_run_goto_height_thread, args=(entry["z"],),
                         daemon=True, name="GotoHeightAction").start()
        return jsonify({"ok": True})

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False),
        daemon=True,
        name="ParamServer",
    ).start()
    print(f"[ParamServer] 参数调节页面: http://<server-ip>:{port}")
