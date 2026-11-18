# Embedded web dashboard — served at GET / by HttpService.
#
# One self-contained page (vanilla JS, no external assets — the robot may
# be on an isolated network). Talks only to the JSON API in api.py:
#   /telemetry /config /logs /crash /cmd /ota/upload /stream /shot
# The auth token for POST endpoints is kept in localStorage and sent as
# X-Auth-Token. Kept as a Python string so OTA bundles carry it and the
# compile-check covers the file.

from .httpd import send_response

PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>gridcam</title>
<style>
 :root{--bg:#14171c;--panel:#1d222a;--line:#2c333e;--fg:#d6dde8;--dim:#7d8899;
       --acc:#4fa3ff;--ok:#39c07a;--warn:#e0a63c;--err:#e05c5c}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.45 system-ui,sans-serif}
 header{display:flex;align-items:center;gap:12px;padding:10px 16px;
        background:var(--panel);border-bottom:1px solid var(--line)}
 header h1{font-size:16px;margin:0}
 #dot{width:10px;height:10px;border-radius:50%;background:var(--err)}
 #dot.ok{background:var(--ok)}
 header .sp{flex:1}
 input,button,textarea,select{background:#12151a;color:var(--fg);
   border:1px solid var(--line);border-radius:4px;padding:5px 9px;font:inherit}
 button{cursor:pointer}
 button:hover{border-color:var(--acc)}
 button.danger{border-color:var(--err);color:var(--err)}
 main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px;
      max-width:1200px;margin:auto}
 @media(max-width:900px){main{grid-template-columns:1fr}}
 section{background:var(--panel);border:1px solid var(--line);
         border-radius:8px;padding:12px}
 section h2{margin:0 0 8px;font-size:13px;text-transform:uppercase;
            letter-spacing:.08em;color:var(--dim)}
 .kv{display:grid;grid-template-columns:auto 1fr;gap:2px 14px;font-size:13px}
 .kv b{color:var(--dim);font-weight:normal}
 pre,textarea{font:12px/1.4 ui-monospace,monospace;white-space:pre;
   overflow:auto;background:#12151a;border:1px solid var(--line);
   border-radius:4px;padding:8px;margin:0}
 pre{max-height:260px}
 textarea{width:100%;height:220px;resize:vertical}
 .row{display:flex;gap:8px;margin-top:8px;flex-wrap:wrap;align-items:center}
 #stream{width:100%;background:#000;border-radius:4px;min-height:120px}
 .live{position:relative}
 #qbadge{position:absolute;top:8px;right:8px;padding:4px 10px;border-radius:6px;
   font:600 15px/1.2 system-ui,sans-serif;color:#0b0b0b;display:none}
 #qbadge small{display:block;font-weight:400;font-size:11px}
 .q-excellent{background:#39c07a}.q-good{background:#9fd36a}
 .q-warning{background:#e0a63c}.q-bad{background:#e05c5c}
 .full{grid-column:1/-1}
 a{color:var(--acc)}
 #msg{color:var(--warn);font-size:12px}
 #cfgmsg{font-size:12px;margin:8px 0 0;white-space:pre-wrap}
 #cfgmsg.ok{color:var(--ok)} #cfgmsg.err{color:var(--err)}
 .seg{display:inline-flex}
 .seg button{border-radius:0}
 .seg button+button{margin-left:-1px}
 .seg button:first-child{border-radius:4px 0 0 4px}
 .seg button:last-child{border-radius:0 4px 4px 0}
 .seg button.on{background:var(--acc);border-color:var(--acc);color:#0b0b0b}
 #tilemsg{font-size:12px;color:var(--warn)}
</style></head><body>
<header><div id="dot"></div><h1>gridcam</h1>
 <span id="ver" style="color:var(--dim)"></span>
 <span class="seg" title="Floor tile. Switch only when the robot is stopped.">
  <button id="tile1" onclick="setTile(1)">Tile 1</button><button
   id="tile2" onclick="setTile(2)">Tile 2</button></span>
 <span id="tilemsg" role="status"></span><span class="sp"></span>
 <span id="msg"></span>
 <input id="token" placeholder="auth token" size="12" title="X-Auth-Token">
</header>
<main>
<section><h2>Status</h2><div class="kv" id="status"></div>
 <div class="row">
  <button onclick="cmd('restart')" class="danger">Restart</button>
  <button onclick="if(confirm('Reboot into safe mode?'))cmd('safe_mode')"
    class="danger">Safe mode</button>
  <button onclick="refresh()">Refresh</button>
 </div>
 <div class="row">
  <label><input type="checkbox" id="ledon" onchange="setLed(this.checked)">
   heartbeat LED</label>
  <button onclick="cmd('identify')">Identify (blink blue)</button>
 </div></section>
<section><h2>Live view</h2>
 <div class="live"><img id="stream" alt="(stream stopped)">
  <div id="qbadge" title="frame quality, short average (updates every 3 s)"></div></div>
 <div class="row">
  <button onclick="startStream()">Start stream</button>
  <button onclick="stopStream()">Stop</button>
  <button onclick="shot()">Snapshot</button>
  <span style="color:var(--dim);font-size:12px">streaming lowers fps</span>
 </div></section>
<section><h2>Config (overlay is persisted)</h2>
 <textarea id="cfg" spellcheck="false"></textarea>
 <div class="row">
  <button onclick="loadCfg()">Reload</button>
  <button onclick="saveCfg(false)">Apply</button>
  <button onclick="saveCfg(true)">Apply + persist</button>
  <button class="danger" onclick="resetCfg()">Set defaults</button>
 </div>
 <div id="cfgmsg" role="status"></div></section>
<section><h2>Software update</h2>
 <div class="kv" id="otainfo"></div>
 <div class="row"><input type="file" id="gpk" accept=".gpk">
  <button onclick="upload()">Upload &amp; install</button></div>
 <p style="color:var(--dim);font-size:12px;margin:8px 0 0">
  Board reboots into a 60&nbsp;s trial; a crash auto-reverts to the
  previous version. Build bundles with <code>host/make_bundle.py</code>.</p>
 </section>
<section><h2>AprilTag</h2>
 <div class="row" style="margin:0 0 8px">
  <button onclick="atDetect()">Detect now</button>
  <label>image <select id="atpre" onchange="atPre()">
   <option value="none">as captured (default)</option>
   <option value="enhance">enhanced: contrast + sharpen</option>
  </select></label>
  <span id="atinfo" style="color:var(--dim);font-size:12px"></span>
 </div>
 <pre id="attags" style="max-height:90px">(no detection yet)</pre>
 <img id="atimg" style="width:100%;border-radius:4px;margin-top:8px;display:none">
</section>
<section><h2>Status LED (on-board)</h2>
 <div class="row">
  <label>colour <select id="sledcolor">
   <option>white</option><option>red</option><option>green</option>
   <option>blue</option><option>yellow</option><option>cyan</option>
   <option>magenta</option></select></label>
 </div>
 <div class="row">
  <button onclick="sled('on')">Constant</button>
  <button onclick="sled('slow')">Slow blink</button>
  <button onclick="sled('fast')">Fast blink</button>
  <button onclick="sled('off')">Off</button>
  <button onclick="sled('auto')">Auto (health)</button>
 </div>
 <p style="color:var(--dim);font-size:12px;margin:8px 0 0">
  Takes the on-board RGB LED away from the health heartbeat.
  "Auto" gives it back — green blink healthy, solid red unhealthy.</p>
</section>
<section><h2>Camera lights</h2>
 <div class="row">
  <label>brightness <input type="range" id="clbright" min="0" max="255"
    value="128" style="vertical-align:middle"
    oninput="$('clval').textContent=this.value"
    onchange="clSet(this.value)"></label>
  <span id="clval">128</span>
 </div>
 <div class="row">
  <button onclick="clSet(255)">Full</button>
  <button onclick="clSet(128)">Half</button>
  <button onclick="clCmd({action:'off'})">Off</button>
 </div>
 <p style="color:var(--dim);font-size:12px;margin:8px 0 0">
  The six white LEDs on the STM32 main board. White only, so there is no
  colour to pick — brightness is one shared level for all six. The camera
  owns the setting and sends it over the UART, so a change applies at once.</p>
</section>
<section class="full"><h2>Logs</h2>
 <div class="row" style="margin:0 0 8px">
  <button onclick="loadLogs()">Tail</button>
  <select id="logfile"></select>
  <button onclick="loadLogFile()">Open file</button>
  <label><input type="checkbox" id="follow"> follow</label>
 </div><pre id="logs"></pre></section>
<section class="full"><h2>Crash reports</h2>
 <div class="row" style="margin:0 0 8px"><select id="crashsel"></select>
  <button onclick="loadCrash()">Open</button></div>
 <pre id="crash">(none)</pre></section>
</main>
<script>
const $=id=>document.getElementById(id);
const tok=()=>({'X-Auth-Token':$('token').value});
$('token').value=localStorage.token||'';
$('token').onchange=()=>localStorage.token=$('token').value;
const say=t=>{$('msg').textContent=t;setTimeout(()=>$('msg').textContent='',5000)};
const esc=x=>x==null?'-':String(x).replace(/[&<>"]/g,c=>'&#'+c.charCodeAt(0)+';');

// Colour badge over the live view: the short-average quality and its level.
// 85+ excellent, 60-84 good, 40-59 warning, under 40 bad (quality.LEVELS).
function qBadge(q){
 const b=$('qbadge');
 if(!q||q.q_avg==null){b.style.display='none';return}
 const a=Math.round(q.q_avg),
   l=a>=85?'excellent':a>=60?'good':a>=40?'warning':'bad';
 b.className='q-'+l;b.style.display='block';
 b.innerHTML='Q '+a+' '+l+(q.why?'<small>'+q.why_text+'</small>':'');
}
async function refresh(){
 try{
  const t=await (await fetch('/telemetry')).json();
  $('dot').className='ok';
  $('ver').textContent='v'+t.version+(t.build&&t.build!='dev'?' ('+t.build+')':'');
  const s=t.services||{},v=s.vision||{},w=s.wifi||{},u=s.uart||{},
        m=t.mem||{},fl=t.flash||{},lg=s.logsvc||{},rs=t.reset||{};
  const kb=x=>((x||0)/1024).toFixed(0),mb=x=>((x||0)/1048576).toFixed(2);
  $('status').innerHTML=
   `<b>uptime</b><span>${(t.uptime_s||0).toFixed(0)} s</span>`+
   `<b>healthy</b><span>${t.healthy} (stable: ${t.stable})</span>`+
   // Why the board last started. A power loss writes nothing, so without
   // this the page cannot tell one apart from a commanded reset -- which is
   // exactly the ambiguity that cost a diagnosis on 2026-09-10.
   `<b>last restart</b><span>${rs.verdict||'unknown'}` +
     `${rs.boots!=null?' — boot '+rs.boots:''}` +
     `${rs.crashes?' <span style="color:var(--err)">crashes '+rs.crashes+'</span>':''}` +
     `${rs.cause?' ['+rs.cause+']':''}</span>`+
   `<b>fps</b><span>${(v.fps||0).toFixed(1)} (frames ${v.frames||0})</span>`+
   `<b>camera</b><span>${v.no_scene?
     '<span style="color:var(--err)">NO SCENE — covered or dark? ('+
     (v.reject_pct||0).toFixed(0)+'% rejected)</span>':
     (v.reject_pct>50?
      '<span style="color:var(--warn)">degraded ('+
      v.reject_pct.toFixed(0)+'% rejected)</span>':
      'ok ('+(v.reject_pct||0).toFixed(0)+'% rejected)')}</span>`+
   tileRows(v)+
   `<b>grid</b><span>${v.tier||'none'}${v.x_mm!=null?' X '+v.x_mm.toFixed(1):''}${v.y_mm!=null?' Y '+v.y_mm.toFixed(1):''}${v.theta_deg!=null?' T '+v.theta_deg.toFixed(2):''} (${v.x_rejected||0} rejected)</span>`+
   // Frame quality (vision/quality.py): q now, its short average, the three
   // parts, and the reason in words when it is low.
   (qBadge(v.quality),'')+
   (v.quality?(()=>{const q=v.quality,p=x=>x==null?'-':(100*x).toFixed(0),
     col=q.q_avg==null?'':q.q_avg<40?'var(--err)':q.q_avg<60?'var(--warn)':'';
     return `<b>quality</b><span${col?' style="color:'+col+'"':''}>`+
     `${q.q==null?'-':q.q} (avg ${q.q_avg==null?'-':q.q_avg.toFixed(0)}; `+
     `picture ${p(q.img)} line ${p(q.line)} recognition ${p(q.rec)})`+
     `${q.why?' — '+q.why_text:''}${q.learning?' [learning]':''}`+
     ` — low ${q.low_pct}% of frames</span>`})():'')+
   `<b>cpu</b><span>${(v.cpu_pct||0).toFixed(0)}% (work ${(v.busy_ms||0).toFixed(0)} ms/frame)</span>`+
   `<b>ram</b><span>${kb(m.alloc)} / ${kb(m.total)} kB used, ${kb(m.free)} kB free</span>`+
   `<b>flash</b><span>${mb(fl.used)} / ${mb(fl.total)} MB used (log writes ${kb(lg.bytes_written)} kB)</span>`+
   `<b>wifi</b><span>${w.connected?(w.ssid?w.ssid+' ':'')+w.ip+' (rssi '+(w.rssi??'?')+')':'down'}</span>`+
   `<b>uart</b><span>sent ${u.sent||0}, dropped ${u.dropped||0}, badcrc ${u.bad_crc||0}</span>`;
  $('otainfo').innerHTML=`<b>version</b><span>${t.version}</span>`+
   `<b>stable</b><span>${t.stable}</span>`;
  showTile(v);
 }catch(e){$('dot').className='';}
}
// Floor tile. The buttons show the tile the camera READS (telemetry), not
// the last click. The switch is its own /cmd that writes only vision.tile,
// never the whole textarea (that would also touch camera.*).
// tilePend: a switch the camera holds (from here, the STM32 or another page);
// a click on the active tile then cancels it.
let tileWant=0,tileSeen=0,tilePend=0;
function tileSay(t){$('tilemsg').textContent=t}
function tileRows(v){
 if(v.tile==null)return '';
 const p=v.tile_pending,e=v.tile_err,d=v.tile2;
 let r=`<b>tile</b><span>${v.tile}${v.tile===2?' (UNIT16 holes)':' (tape)'}`+
  `${p?' — switching to '+esc(p):''}${v.junction_code?' — node '+esc(v.junction_code):''}`+
  `${e?' <span style="color:var(--err)">error: '+esc(e)+'</span>':''}</span>`;
 if(v.tile===2&&d)r+=`<b>tile 2 read</b><span>mode ${esc(d.mode)} lines ${esc(d.lines)}`+
  ` pres ${esc(d.pres)} cdist ${esc(d.cdist)} xnode ${esc(d.xnode)}`+
  `${d.uc!=null?' uc '+esc(d.uc):''}</span>`;
 return r;
}
function showTile(v){
 const t=v.tile;
 $('tile1').className=t===1?'on':'';$('tile2').className=t===2?'on':'';
 if(t==null)return;
 tilePend=v.tile_pending||0;
 if(tileSeen&&t!==tileSeen)loadCfg();     // switched (here, or by the STM32)
 tileSeen=t;
 if(tileWant&&t===tileWant&&!v.tile_pending){tileWant=0;tileSay('')}
}
async function setTile(n){
 if(n===tileSeen&&!tileWant&&!tilePend){say('already on Tile '+n);return}
 let r,j;
 try{r=await fetch('/cmd',{method:'POST',headers:tok(),
   body:JSON.stringify({cmd:'tile',tile:n})});j=await r.json()}
 catch(e){tileSay('Tile '+n+': no answer from the camera');return}
 if(!r.ok||!j.ok){tileWant=0;
  tileSay('Tile '+n+' refused: '+((j&&(j.reason||j.error))||('HTTP '+r.status)));return}
 tileWant=n;
 tileSay((n===tileSeen?'staying on Tile '+n:'switching...')+(j.reason?' ('+j.reason+')':'')+
   (j.persisted?'':' (NOT saved to flash)'));
 loadCfg();                               // the textarea must not hold the old tile
 setTimeout(refresh,700);
}
async function cmd(c){
 const r=await fetch('/cmd',{method:'POST',headers:tok(),
   body:JSON.stringify({cmd:c})});
 say((await r.json()).error||c+' sent');
}
const BLANK='data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
function startStream(){$('stream').src=BLANK;
 setTimeout(()=>$('stream').src='/stream?'+Date.now(),100)}
function stopStream(){$('stream').src=BLANK}
async function shot(){$('stream').src='/shot?'+Date.now()}
async function loadCfg(){
 const c=await (await fetch('/config')).json();
 $('cfg').value=JSON.stringify(c,null,1);
 $('cfgmsg').textContent='';
 $('ledon').checked=!!(c.led&&c.led.enabled);
 $('atpre').value=(c.apriltag&&c.apriltag.preprocess==='enhance')?'enhance':'none';
}
async function atPre(){
 const v=$('atpre').value;
 const r=await fetch('/config',{method:'POST',headers:tok(),
   body:JSON.stringify({patch:{apriltag:{preprocess:v}},persist:true})});
 say((await r.json()).ok?'AprilTag image: '+v+' (saved, all routes)':'failed');
}
async function setLed(on){
 await fetch('/config',{method:'POST',headers:tok(),
   body:JSON.stringify({patch:{led:{enabled:on}},persist:true})});
 say('heartbeat LED '+(on?'on':'off'));
}
function cfgSay(ok,t){const m=$('cfgmsg');m.className=ok?'ok':'err';m.textContent=t}
async function resetCfg(){
 if(!confirm('Set EVERY setting back to its default and save it? '+
   'The WiFi login, the web token and the tile are kept.'))return;
 let r,j;
 try{r=await fetch('/config',{method:'POST',headers:tok(),
   body:JSON.stringify({reset:true})});j=await r.json()}
 catch(e){cfgSay(false,'NOT reset: no answer from the camera ('+e.message+')');return}
 if(!r.ok||!j.ok){cfgSay(false,'NOT reset: '+((j&&j.error)||('HTTP '+r.status)));return}
 await loadCfg();
 cfgSay(j.persisted,j.persisted?'Defaults set and saved (WiFi, token and tile kept)':
   'Defaults set until the next restart, but SAVING TO FLASH FAILED (see Logs)');
}
async function saveCfg(persist){
 // Stays on screen until the next Apply or Reload: an error that vanishes
 // after 5 s is easy to miss, and nothing was changed on the camera.
 let patch;
 try{patch=JSON.parse($('cfg').value)}
 catch(e){cfgSay(false,'NOT applied, nothing changed: the text is not valid JSON. '+e.message);return}
 if(patch===null||typeof patch!=='object'||Array.isArray(patch)){
  cfgSay(false,'NOT applied, nothing changed: the config must be a JSON object { ... }');return}
 // vision.tile is the Tile buttons' (POST /cmd tile): text loaded before
 // a switch by the STM32 or another page would put the old tile back
 if(patch.vision&&typeof patch.vision==='object'){
  delete patch.vision.tile;
  if(!Object.keys(patch.vision).length)delete patch.vision}
 let r,j;
 try{r=await fetch('/config',{method:'POST',headers:tok(),
   body:JSON.stringify({patch:patch,persist:persist})});j=await r.json()}
 catch(e){cfgSay(false,'NOT applied: no answer from the camera ('+e.message+')');return}
 if(!r.ok||!j.ok){
  const why=(j&&j.errors)?j.errors.join(' | '):((j&&j.error)||('HTTP '+r.status));
  cfgSay(false,'NOT applied, nothing changed: '+why);return}
 if(persist&&!j.persisted){
  cfgSay(false,'Applied until the next restart, but SAVING TO FLASH FAILED (see Logs)');return}
 cfgSay(true,persist?'Applied and saved to flash':'Applied (until the next restart; use Apply + persist to keep it)');
}
async function loadLogs(){
 $('logs').textContent=await (await fetch('/logs?tail=100')).text();
 $('logs').scrollTop=1e9;
 const f=await (await fetch('/logs')).json();
 $('logfile').innerHTML=(f.files||[]).map(x=>
   `<option>${x.name}</option>`).join('');
}
async function loadLogFile(){
 const n=$('logfile').value;if(!n)return;
 $('logs').textContent=await (await fetch('/logs/'+n)).text();
}
async function loadCrashList(){
 const c=await (await fetch('/crash')).json();
 $('crashsel').innerHTML=(c.reports||[]).map(x=>
   `<option>${x}</option>`).join('');
}
async function loadCrash(){
 const n=$('crashsel').value;if(!n)return;
 $('crash').textContent=await (await fetch('/crash/'+n)).text();
}
async function upload(){
 const f=$('gpk').files[0];if(!f){say('choose a .gpk file');return}
 say('uploading '+f.name+' ...');
 const r=await fetch('/ota/upload',{method:'POST',headers:tok(),body:f});
 const j=await r.json();
 say(j.error?('rejected: '+j.error):'installing — board reboots for trial');
}
async function atDetect(){
 $('atinfo').textContent='detecting...';
 try{
  const r=await(await fetch('/apriltag/detect',{method:'POST',headers:tok(),
    body:'{}'})).json();
  if(r.error){$('atinfo').textContent='error: '+r.error;return}
  $('atinfo').textContent=`${r.tags.length} tag(s) in ${r.elapsed_ms} ms, image: ${r.preprocess||'?'}`;
  $('attags').textContent=r.tags.length?
   r.tags.map(t=>`ID ${t.id}  angle ${t.rotation_deg.toFixed(0)}°  `+
     `margin ${t.decision_margin.toFixed(1)}  at (${t.cx},${t.cy})`).join('\\n')
   :'(no tags in view)';
  $('atimg').style.display='block';
  $('atimg').src='/apriltag/image?'+Date.now();
 }catch(e){$('atinfo').textContent='failed: '+e}
}
async function sled(mode){
 const b={cmd:'led',mode:mode};
 if(mode!=='auto')b.color=$('sledcolor').value;
 const r=await(await fetch('/cmd',{method:'POST',headers:tok(),
   body:JSON.stringify(b)})).json();
 say(r.error?('led: '+r.error):('status LED: '+(mode==='auto'?'auto (health)':
   mode+' '+b.color)));
}
async function clCmd(p){
 const r=await(await fetch('/cmd',{method:'POST',headers:tok(),
   body:JSON.stringify({cmd:'camled',...p})})).json();
 say(r.error?('camera lights: '+r.error):('camera lights: level '+r.level));
 if(!r.error&&r.brightness!==undefined){
  $('clbright').value=r.brightness; $('clval').textContent=r.brightness;
 }
}
function clSet(v){
 const n=parseInt(v,10);
 clCmd({enabled:n>0,brightness:n});
}
setInterval(refresh,3000);
setInterval(()=>{if($('follow').checked)loadLogs()},3000);
refresh();loadCfg();loadLogs();loadCrashList();
</script></body></html>"""


def register(server, app):
    async def index(req):
        await send_response(req.writer, content_type='text/html', body=PAGE)

    server.add_route('GET', '/', index)
