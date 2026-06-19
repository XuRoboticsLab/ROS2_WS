# ─────────────────────────────────────────────
#  param_server.py  —  Flask 参数实时调节服务器（Z1 MoveL 简化版）
# ─────────────────────────────────────────────

import logging
import threading

from z1_config import TRANSLATION_SCALE, ROTATION_SCALE, FINE_SCALE

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
body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
     padding:28px 20px;max-width:520px;margin:0 auto}
h1{font-size:1.5rem;font-weight:700;margin-bottom:4px}
.sub{color:var(--muted);font-size:.88rem;margin-bottom:24px}
.card{background:var(--card);border-radius:16px;padding:20px 20px 4px;box-shadow:0 1px 4px rgba(0,0,0,.08);margin-bottom:16px}
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
.footer{display:flex;gap:12px;align-items:center;margin-top:8px}
.reset-btn{flex:1;padding:13px;background:#F5F5F7;border:1.5px solid var(--border);
  border-radius:10px;font-size:.93rem;font-weight:500;cursor:pointer;color:var(--text)}
.reset-btn:hover{background:#E8E8EA}
#st{color:var(--muted);font-size:.8rem;min-height:18px}
</style>
</head>
<body>
<h1>{{ arm_name }}</h1>
<p class="sub">参数实时调节 · 拖动滑块即时生效</p>

<div class="card">
  <div class="sec">Pico → 机械臂缩放</div>
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
  <div class="sec">摇杆精细控制</div>
  <div class="param">
    <div class="prow">
      <span class="plabel">精细平动 <code>fine_scale</code></span>
      <span class="pval" id="v-fs">{{ "%.3f"|format(fs_def) }} m/s</span>
    </div>
    <input type="range" id="s-fs" min="0.005" max="0.3" step="0.005" value="{{ fs_def }}">
    <div class="rlabels"><span>0.005</span><span>0.3 m/s</span></div>
  </div>
</div>

<div class="footer">
  <button class="reset-btn" onclick="resetAll()">↩ 恢复默认值</button>
  <div id="st"></div>
</div>

<script>
const DEF  = {tr:{{ tr_def }},ro:{{ ro_def }},fs:{{ fs_def }}};
const IDS  = ['tr','ro','fs'];
const KEYS = {tr:'translation_m',ro:'rotation_rad',fs:'fine_scale'};
const UNIT = {tr:'',ro:'',fs:' m/s'};
const FMT  = v => v.toFixed(3);

function track(el){
  const p=(el.value-el.min)/(el.max-el.min)*100;
  el.style.setProperty('--p',p+'%');
}
function setStatus(msg){document.getElementById('st').textContent=msg;}

async function send(payload){
  try{
    await fetch('/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    setStatus('已更新 '+new Date().toLocaleTimeString());
  }catch(e){setStatus('失败: '+e.message);}
}

IDS.forEach(id=>{
  const el=document.getElementById('s-'+id);
  track(el);
  el.addEventListener('input',()=>{
    const v=parseFloat(el.value);
    track(el);
    document.getElementById('v-'+id).textContent=FMT(v)+UNIT[id];
    send({[KEYS[id]]:v});
  });
});

fetch('/params').then(r=>r.json()).then(d=>{
  IDS.forEach(id=>{
    const key=KEYS[id],el=document.getElementById('s-'+id);
    if(d[key]!==undefined){el.value=d[key];track(el);
      document.getElementById('v-'+id).textContent=FMT(d[key])+UNIT[id];}
  });
}).catch(()=>{});

function resetAll(){
  IDS.forEach(id=>{
    const el=document.getElementById('s-'+id);
    el.value=DEF[id];track(el);
    document.getElementById('v-'+id).textContent=FMT(DEF[id])+UNIT[id];
  });
  send({translation_m:DEF.tr,rotation_rad:DEF.ro,fine_scale:DEF.fs});
}
</script>
</body>
</html>
"""


def start(state, port: int, arm_name: str):
    try:
        from flask import Flask, request, jsonify, render_template_string
    except ImportError:
        print("[ParamServer] flask 未安装，跳过 (pip install flask)")
        return

    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(
            _HTML,
            arm_name=arm_name,
            tr_def=TRANSLATION_SCALE,
            ro_def=ROTATION_SCALE,
            fs_def=FINE_SCALE,
        )

    @app.route("/params", methods=["GET"])
    def get_params():
        return jsonify({
            "translation_m": state.translation_scale,
            "rotation_rad":  state.rotation_scale,
            "fine_scale":    state.fine_scale,
        })

    @app.route("/params", methods=["POST"])
    def update_params():
        data = request.get_json(force=True) or {}
        if "translation_m" in data: state.translation_scale = float(data["translation_m"])
        if "rotation_rad"  in data: state.rotation_scale    = float(data["rotation_rad"])
        if "fine_scale"    in data: state.fine_scale        = float(data["fine_scale"])
        return jsonify({"ok": True})

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False),
        daemon=True, name="ParamServer",
    ).start()
    print(f"[ParamServer] 参数调节页面: http://<server-ip>:{port}")
