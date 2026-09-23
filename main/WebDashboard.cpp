#include "WebDashboard.h"

#include <WebServer.h>
#include <WiFi.h>

namespace {
WebServer server(80);
bool serverStarted = false;
bool startRequested = false;
bool stopRequested = false;
bool resetRequested = false;
String stateJson = "{}";

constexpr int LOG_CAPACITY = 80;
String logLines[LOG_CAPACITY];
uint32_t logSequence = 0;

const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Micromouse Flood Fill</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}
body{margin:0;background:#0b1220;color:#e5eefc}
header{padding:14px 18px;background:#111c30;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
h1{font-size:19px;margin:0}.pill{background:#20304d;padding:6px 10px;border-radius:14px}
main{display:grid;grid-template-columns:minmax(330px,680px) minmax(300px,1fr);gap:14px;padding:14px}
.panel{background:#111c30;border:1px solid #263956;border-radius:10px;padding:12px}
canvas{width:100%;aspect-ratio:1;background:#09101d}
button,input{font:inherit;border-radius:6px;border:1px solid #4b658b;padding:8px;background:#172944;color:#fff}
button{cursor:pointer}.start{background:#146c43}.stop{background:#9e2f38}
#console{height:430px;overflow:auto;white-space:pre-wrap;background:#050a12;padding:10px;border-radius:6px;font:12px ui-monospace,monospace}
form{display:flex;gap:7px;margin-top:8px}input{flex:1}
.legend{font-size:13px;color:#aebed7;margin-top:8px}
@media(max-width:850px){main{grid-template-columns:1fr}#console{height:270px}}
</style>
</head>
<body>
<header>
 <h1>Micromouse flood-fill exploration</h1>
 <span class="pill" id="pose">Pose: --</span>
 <span class="pill" id="run">Connecting...</span>
 <button class="start" onclick="command('start')">Start</button>
 <button class="stop" onclick="command('d')">Stop</button>
 <button onclick="command('reset')">Reset map</button>
</header>
<main>
 <section class="panel">
  <canvas id="maze" width="768" height="768"></canvas>
  <div class="legend">Cyan: known wall · blue: visited · number: flood distance · yellow arrow: robot</div>
 </section>
 <section class="panel">
  <h2 style="margin-top:0;font-size:17px">Wi-Fi serial console</h2>
  <div id="console"></div>
  <form onsubmit="commandLine(event)">
   <input id="command" autocomplete="off" placeholder="s / start / d / stop / reset">
   <button>Send</button>
  </form>
 </section>
</main>
<script>
const canvas=document.getElementById('maze'),ctx=canvas.getContext('2d');
const output=document.getElementById('console');let after=0;
const arrows=['↑','→','↓','←'];
async function command(value){
 await fetch('/api/command',{method:'POST',headers:{'Content-Type':'text/plain'},body:value});
}
function commandLine(event){
 event.preventDefault();const input=document.getElementById('command');
 if(input.value.trim())command(input.value.trim());input.value='';
}
function draw(s){
 const n=s.size||16,w=canvas.width/n;
 ctx.fillStyle='#09101d';ctx.fillRect(0,0,canvas.width,canvas.height);
 for(let y=0;y<n;y++)for(let x=0;x<n;x++){
  const i=y*n+x,px=x*w,py=(n-1-y)*w,known=s.known[i],walls=s.walls[i];
  if(s.visited[i]){ctx.fillStyle='#14365d';ctx.fillRect(px+2,py+2,w-4,w-4)}
  ctx.fillStyle='#8fa8ca';ctx.font=(w*.28)+'px system-ui';ctx.textAlign='center';ctx.textBaseline='middle';
  const d=s.distance[i];if(d<65535)ctx.fillText(d,px+w/2,py+w/2);
  const edges=[[1,px,py,px+w,py],[2,px+w,py,px+w,py+w],[4,px,py+w,px+w,py+w],[8,px,py,px,py+w]];
  for(const e of edges)if(known&e[0]){
   ctx.strokeStyle=(walls&e[0])?'#45d4ff':'#263956';ctx.lineWidth=(walls&e[0])?4:1;
   ctx.beginPath();ctx.moveTo(e[1],e[2]);ctx.lineTo(e[3],e[4]);ctx.stroke();
  }
 }
 const px=s.x*w+w/2,py=(n-1-s.y)*w+w/2;
 ctx.fillStyle='#ffd34d';ctx.font=(w*.65)+'px system-ui';ctx.fillText(arrows[s.heading],px,py);
 document.getElementById('pose').textContent=`Pose: (${s.x}, ${s.y}) ${s.headingName}`;
 document.getElementById('run').textContent=s.status;
}
async function update(){
 try{
  const state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());draw(state);
  const logs=await fetch('/api/logs?after='+after,{cache:'no-store'}).then(r=>r.json());
  for(const line of logs.lines){output.textContent+=line.text+'\n';after=line.seq}
  if(logs.lines.length){output.scrollTop=output.scrollHeight}
 }catch(e){document.getElementById('run').textContent='Dashboard disconnected'}
}
setInterval(update,400);update();
</script>
</body>
</html>
)HTML";

String jsonString(const char *text) {
  String result;
  result += '"';
  if (text) {
    while (*text) {
      const char c = *text++;
      if (c == '"' || c == '\\') { result += '\\'; result += c; }
      else if (c == '\n') result += "\\n";
      else if (c == '\r') result += "\\r";
      else if ((uint8_t)c >= 0x20) result += c;
    }
  }
  result += '"';
  return result;
}

void handleCommand() {
  String command = server.arg("plain");
  command.trim();
  command.toLowerCase();
  if (command == "s" || command == "start") startRequested = true;
  else if (command == "d" || command == "stop") stopRequested = true;
  else if (command == "r" || command == "reset") {
    resetRequested = true;
    stopRequested = true;
  }
  else {
    server.send(400, "text/plain", "Use s/start, d/stop, or reset");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleLogs() {
  const uint32_t after = (uint32_t)server.arg("after").toInt();
  uint32_t first = after + 1;
  const uint32_t oldest = logSequence > LOG_CAPACITY
      ? logSequence - LOG_CAPACITY + 1 : 1;
  if (first < oldest) first = oldest;

  String response;
  response.reserve(2048);
  response = "{\"lines\":[";
  bool comma = false;
  for (uint32_t sequence = first; sequence <= logSequence; ++sequence) {
    if (comma) response += ',';
    comma = true;
    response += "{\"seq\":";
    response += sequence;
    response += ",\"text\":";
    response += jsonString(logLines[(sequence - 1) % LOG_CAPACITY].c_str());
    response += '}';
  }
  response += "]}";
  server.send(200, "application/json", response);
}
}  // namespace

void dashboardBegin() {
  if (serverStarted) return;
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", HTTP_GET,
            []() { server.send(200, "application/json", stateJson); });
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/command", HTTP_POST, handleCommand);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  serverStarted = true;
}

void dashboardLoop() {
  if (serverStarted) server.handleClient();
}

bool dashboardTakeStart() {
  const bool requested = startRequested;
  startRequested = false;
  return requested;
}

bool dashboardTakeStop() {
  const bool requested = stopRequested;
  stopRequested = false;
  return requested;
}

bool dashboardTakeReset() {
  const bool requested = resetRequested;
  resetRequested = false;
  return requested;
}

void dashboardAppendLog(const char *line) {
  ++logSequence;
  logLines[(logSequence - 1) % LOG_CAPACITY] = line ? line : "";
}

void dashboardUpdateMap(const MazeMap &maze, int robotX, int robotY,
                        Direction heading, bool running, const char *status) {
  String json;
  json.reserve(7000);
  json = "{\"size\":16,\"x\":";
  json += robotX;
  json += ",\"y\":";
  json += robotY;
  json += ",\"heading\":";
  json += (uint8_t)heading;
  json += ",\"headingName\":";
  json += jsonString(MazeMap::directionName(heading));
  json += ",\"running\":";
  json += running ? "true" : "false";
  json += ",\"status\":";
  json += jsonString(status);

  const char *names[] = {"walls", "known", "visited", "distance"};
  for (int field = 0; field < 4; ++field) {
    json += ",\"";
    json += names[field];
    json += "\":[";
    bool comma = false;
    for (int y = 0; y < MAZE_SIZE; ++y) {
      for (int x = 0; x < MAZE_SIZE; ++x) {
        if (comma) json += ',';
        comma = true;
        const MazeCell &cell = maze.cell(x, y);
        if (field == 0) json += cell.walls;
        else if (field == 1) json += cell.known;
        else if (field == 2) json += cell.visited ? 1 : 0;
        else json += cell.distance;
      }
    }
    json += ']';
  }
  json += '}';
  stateJson = json;
}
