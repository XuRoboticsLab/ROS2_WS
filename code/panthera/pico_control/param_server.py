# ─────────────────────────────────────────────
#  param_server.py  —  Flask 参数实时调节服务器
#
#  GET  /        → 调节页面
#  GET  /params  → 当前参数 JSON
#  POST /params  → 更新参数
# ─────────────────────────────────────────────

import logging
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

# 预制动作用到的常量旋转
_R_CONST  = Rotation.from_euler('y',  np.pi / 2).as_matrix()  # Ry(90°) — gripper 竖直向下
_R_Z_POS  = Rotation.from_euler('z',  np.pi / 2).as_matrix()  # Rz(+90°)
_R_Z_NEG  = Rotation.from_euler('z', -np.pi / 2).as_matrix()  # Rz(-90°)


def _wait_rotation(state, target_rot: np.ndarray,
                   timeout: float = 6.0, tol: float = 0.08) -> bool:
    """阻塞直到 smooth_rotation 与 target_rot 误差 < tol rad，或超时返回 False。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if state.rotation_error_magnitude(target_rot) < tol:
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
body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;padding:28px 16px;max-width:520px;margin:0 auto}
h1{font-size:1.5rem;font-weight:700;margin-bottom:4px}
.sub{color:var(--muted);font-size:.88rem;margin-bottom:24px}
.card{background:var(--card);border-radius:16px;padding:20px 20px 4px;margin-bottom:16px;box-shadow:0 1px 4px rgba(0,0,0,.08)}
.sec{font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:16px}
.param{margin-bottom:20px}
.prow{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:6px}
.plabel{font-weight:500;font-size:.93rem}
.pval{font-family:"SF Mono","Fira Code",monospace;font-size:1.05rem;color:var(--accent);font-weight:600;min-width:64px;text-align:right}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:4px;border-radius:2px;
  background:linear-gradient(to right,var(--accent) 0%,var(--accent) var(--p,0%),var(--border) var(--p,0%),var(--border) 100%);
  outline:none;cursor:pointer;margin-bottom:4px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;
  background:var(--accent);cursor:pointer;box-shadow:0 1px 6px rgba(0,122,255,.35)}
.rlabels{display:flex;justify-content:space-between;color:var(--muted);font-size:.76rem}
.detail{color:var(--muted);font-size:.75rem;margin-top:3px;font-family:monospace}
.reset{display:block;width:100%;padding:13px;background:#F5F5F7;border:1.5px solid var(--border);
  border-radius:10px;font-size:.93rem;font-weight:500;cursor:pointer;color:var(--text);transition:background .1s}
.reset:hover{background:#E8E8EA}
#st{text-align:center;color:var(--muted);font-size:.8rem;margin-top:12px;min-height:18px}
.mode-group{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.mode-opt{border:2px solid var(--border);border-radius:12px;padding:12px 8px;cursor:pointer;
  text-align:center;transition:border-color .15s,background .15s;user-select:none}
.mode-opt:hover{border-color:var(--accent)}
.mode-opt.active{border-color:var(--accent);background:rgba(0,122,255,.08)}
.mode-title{font-weight:600;font-size:.88rem}
.mode-desc{color:var(--muted);font-size:.72rem;margin-top:4px;line-height:1.3}
.action-btn{display:block;width:100%;padding:14px 16px;background:var(--card);
  border:2px solid var(--border);border-radius:12px;cursor:pointer;text-align:left;
  transition:border-color .15s,background .15s;margin-bottom:10px}
.action-btn:last-child{margin-bottom:0}
.action-btn:hover:not(:disabled){border-color:var(--accent);background:rgba(0,122,255,.06)}
.action-btn:disabled{opacity:.5;cursor:not-allowed}
.action-btn .an{display:block;font-weight:600;font-size:.95rem;color:var(--text)}
.action-btn .ad{display:block;font-size:.75rem;color:var(--muted);margin-top:3px}
</style>
</head>
<body>
<h1>{{ arm_name }}</h1>
<p class="sub">参数实时调节 · 拖动滑块即时生效</p>

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
  </div>
</div>

<div class="card">
  <div class="sec">运动缩放</div>
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
</div>

<div class="card">
  <div class="sec">Smooth Target PD 参数</div>
  <div class="param">
    <div class="prow">
      <span class="plabel">跟踪增益 <code>tracking_gain_hz</code></span>
      <span class="pval" id="v-tg">{{ "%.1f"|format(tg_def) }} Hz</span>
    </div>
    <input type="range" id="s-tg" min="1" max="200" step="1" value="{{ tg_def }}">
    <div class="rlabels"><span>1 Hz</span><span>200 Hz</span></div>
    <div class="detail">时间常数 ≈ {{ "%.3f"|format(1.0/tg_def) }} s（越大跟踪越快）</div>
  </div>
  <div class="param">
    <div class="prow">
      <span class="plabel">阻尼比 <code>damping_ratio</code></span>
      <span class="pval" id="v-dr">{{ "%.2f"|format(dr_def) }}</span>
    </div>
    <input type="range" id="s-dr" min="0.1" max="2.0" step="0.05" value="{{ dr_def }}">
    <div class="rlabels"><span>0.1（欠阻尼）</span><span>2.0（过阻尼）</span></div>
    <div class="detail">1.0 = 临界阻尼；&lt;1 有超调；&gt;1 无超调但慢</div>
  </div>
</div>

<div class="card">
  <div class="sec">操纵杆精细控制</div>
  <div class="mode-group" style="grid-template-columns:1fr 1fr 1fr;margin-bottom:16px">
    <div class="mode-opt active" id="fopt-0" onclick="setFineMode(0)">
      <div class="mode-title">XY 平动</div>
      <div class="mode-desc">世界坐标系<br>x/y 方向平移</div>
    </div>
    <div class="mode-opt" id="fopt-1" onclick="setFineMode(1)">
      <div class="mode-title">Z 旋转</div>
      <div class="mode-desc">世界坐标系<br>绕 z 轴旋转</div>
    </div>
    <div class="mode-opt" id="fopt-2" onclick="setFineMode(2)">
      <div class="mode-title">EEF 平动</div>
      <div class="mode-desc">末端坐标系<br>x/y 方向平移</div>
    </div>
  </div>
  <div class="param">
    <div class="prow">
      <span class="plabel">平动速度 <code>fine_scale</code></span>
      <span class="pval" id="v-fs">{{ "%.3f"|format(fs_def) }} m/s</span>
    </div>
    <input type="range" id="s-fs" min="0.01" max="0.5" step="0.01" value="{{ fs_def }}">
    <div class="rlabels"><span>0.01</span><span>0.5 m/s</span></div>
  </div>
  <div class="param">
    <div class="prow">
      <span class="plabel">旋转速度 <code>fine_rotation_scale</code></span>
      <span class="pval" id="v-fr">{{ "%.2f"|format(fr_def) }} rad/s</span>
    </div>
    <input type="range" id="s-fr" min="0.05" max="3.0" step="0.05" value="{{ fr_def }}">
    <div class="rlabels"><span>0.05</span><span>3.0 rad/s</span></div>
  </div>
</div>

<div class="card">
  <div class="sec">预制动作</div>
  <button class="action-btn" id="btn-screw" onclick="runAction('screw_cap','btn-screw')">
    <span class="an">扭瓶盖 ×3</span>
    <span class="ad">先对齐 Ry90° → 合爪 → z+90° → 松爪 → z−90°，重复三次</span>
  </button>
</div>

<button class="reset" onclick="resetAll()">↩ 恢复默认值</button>
<div id="st"></div>

<script>
const DEF = { tr:{{ tr_def }}, ro:{{ ro_def }}, tg:{{ tg_def }}, dr:{{ dr_def }}, fs:{{ fs_def }}, fr:{{ fr_def }} };
const IDS  = ['tr','ro','tg','dr','fs','fr'];
const KEYS = { tr:'translation_m', ro:'rotation_rad', tg:'tracking_gain_hz', dr:'damping_ratio', fs:'fine_scale', fr:'fine_rotation_scale' };
const UNIT = { tr:'', ro:'', tg:' Hz', dr:'', fs:' m/s', fr:' rad/s' };
const FMT  = (v, id) => id==='tg' ? v.toFixed(1) : (id==='dr'||id==='fr') ? v.toFixed(2) : v.toFixed(3);

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
  [0,1,2].forEach(i=>document.getElementById('opt-'+i).classList.toggle('active',i===n));
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

// 页面加载时同步当前参数
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
  send({translation_m:DEF.tr,rotation_rad:DEF.ro,tracking_gain_hz:DEF.tg,damping_ratio:DEF.dr,fine_scale:DEF.fs,fine_rotation_scale:DEF.fr});
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
</script>
</body>
</html>
"""


def start(state, port: int, arm_name: str):
    """在 daemon 线程中启动参数调节服务器。"""
    try:
        from flask import Flask, request, jsonify, render_template_string
    except ImportError:
        print("[ParamServer] flask 未安装，跳过参数调节服务器 (pip install flask)")
        return

    # 压制 werkzeug 的请求日志，避免干扰主程序输出
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    from config import TRANSLATION_SCALE, ROTATION_SCALE, TRACKING_GAIN_HZ, DAMPING_RATIO, FINE_SCALE, FINE_ROTATION_SCALE

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

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False),
        daemon=True,
        name="ParamServer",
    ).start()
    print(f"[ParamServer] 参数调节页面: http://<server-ip>:{port}")
