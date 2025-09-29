#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QMC5883LCompass.h>
#include <math.h>
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 26
#define LORA_FREQ 433E6
#define GPS_RX 34
#define GPS_TX 12
#define OLED_ADDR 0x3C
#define OLED_W 128
#define OLED_H 64
#define BUZZER_PIN 25
#define BUZZER_CH 0
#define SOS_BTN_PIN 13
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
HardwareSerial GPSSerial(1);
const char *AP_SSID = "TBEAM";
const char *AP_PASS = "";
WebServer server(80);
TinyGPSPlus gps;
Preferences prefs;
QMC5883LCompass qmc;
#define USE_QMC_CALIB 1
const float MX_OFF = 416.500f;
const float MY_OFF = 152.500f;
const float MZ_OFF = 293.000f;
const float MX_SCL = 0.943335f;
const float MY_SCL = 1.001467f;
const float MZ_SCL = 1.062252f;
struct MsgItem{ String text; bool isLoc; bool out; uint32_t id; };
static const int MAX_MSG=10;
MsgItem ringBuf[MAX_MSG];
int ringCount=0;
uint32_t lastId=0;
String lastShown="";
String lastCoord="";
uint32_t lastCoordMillis=0;
bool lockActive=false;
double targetLat=0, targetLon=0;
String targetCoordStr="";
double curLat=0, curLon=0;
float headingDeg=0;
bool  mag_ok=false;
uint32_t lastCompassMillis=0;
uint32_t lastMsgMillis=0;
bool shortBeepOn=false;
uint32_t shortBeepUntil=0;
uint32_t sosBuzzUntil=0;
bool btnPrev=HIGH;
uint32_t btnDebounceAt=0;
bool display_ok=false;
bool lora_ok=false;
uint32_t lastLoRaActivity=0;
bool     showingHeading=false;
uint32_t showHeadingUntil=0;
enum RadioProfile { RP_DEFAULT, RP_RANGE };
volatile RadioProfile radioProf = RP_DEFAULT;

void setLoRaDefault(){
  LoRa.setSpreadingFactor(10);            
  LoRa.setSignalBandwidth(250E3);          
  LoRa.setCodingRate4(5);                  
  LoRa.setPreambleLength(12);
  LoRa.enableCrc();}

void setLoRaRange(){
  LoRa.setSpreadingFactor(12);            
  LoRa.setSignalBandwidth(62500);        
  LoRa.setCodingRate4(8);                  
  LoRa.setPreambleLength(16);
  LoRa.enableCrc();}

void applyProfile(RadioProfile p){
  if(p==RP_DEFAULT) setLoRaDefault(); else setLoRaRange();
  radioProf = p;}
void oledPower(bool on){
  if(!display_ok) return;
  display.ssd1306_command(on? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);}
void wifiOn(){
  WiFi.mode(WIFI_AP);
  if(strlen(AP_PASS)==0) WiFi.softAP(AP_SSID);
  else WiFi.softAP(AP_SSID, AP_PASS);}
void wifiOff(){
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);}
// ===================== WEB UI =====================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>LoRa Chat + GPS + Kompas</title>
<style>
:root{--bg1:#0b1220;--bg2:#0c1730;--card:rgba(18,26,43,.78);--ring:#25345a;--text:#eaf0ff;--mut:#9fb2d9;--accent:#5ab1ff;--good:#2bd66a;--warn:#ffd64d;--bad:#ff4d4d}
*{box-sizing:border-box}html,body{height:100%}body{margin:0;font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;color:var(--text);background:linear-gradient(180deg,var(--bg1),var(--bg2)) fixed}
header{position:sticky;top:0;background:linear-gradient(90deg,#0e1a33,#0b254a);border-bottom:1px solid #132246;padding:10px 12px;z-index:9}
.hrow{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
h1{margin:0;font-size:16px}
.chip{padding:6px 10px;border-radius:999px;border:1px solid var(--ring);background:rgba(15,20,35,.6);font-size:12px;color:var(--mut)}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;border:2px solid #0003;box-shadow:0 0 0 3px #0002 inset;vertical-align:middle}
.green{background:var(--good)}.yellow{background:var(--warn)}.red{background:var(--bad)}
main{padding:12px;max-width:920px;margin:0 auto;display:grid;gap:12px}
.card{backdrop-filter:blur(12px);background:var(--card);border:1px solid var(--ring);border-radius:16px;padding:14px}
.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.spread{display:flex;justify-content:space-between;align-items:center;gap:10px}
button{border:none;padding:12px 14px;border-radius:12px;cursor:pointer;font-weight:700;letter-spacing:.2px}
button:active{transform:translateY(1px)}
.accent{background:var(--accent);color:#001428}
.ghost{background:rgba(13,21,38,.8);color:#fff;border:1px solid var(--ring)}
.btnGreen{background:var(--good);color:#001428}
.btnRed{background:var(--bad);color:#001428}
.small{font-size:12px;color:var(--mut)}
#locVal{font-family:ui-monospace,Consolas,monospace;background:rgba(10,16,30,.85);border:1px dashed #2b3a63;padding:10px 12px;border-radius:12px;min-width:240px}
input[type=text]{flex:1;min-width:180px;padding:12px;border-radius:12px;border:1px solid var(--ring);background:rgba(13,21,38,.85);color:var(--text);outline:none}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media (max-width:720px){.cols{grid-template-columns:1fr}}
.colHead{font-weight:700;font-size:12px;color:var(--mut);margin:2px 0 6px}
.list{display:flex;flex-direction:column;gap:8px;max-height:48vh;overflow:auto;padding-right:2px}
.bubble{padding:11px 13px;border-radius:14px;max-width:95%;background:rgba(13,21,38,.9);border:1px solid var(--ring);word-break:break-word}
.out{align-self:flex-end;background:linear-gradient(180deg,#0f2a56,#153c77);border-color:#2b4b82}
.loc{border-style:dashed}
.footer{display:flex;align-items:center;gap:10px;justify-content:space-between;margin-top:8px}
.badge{border:1px solid var(--ring);border-radius:999px;padding:6px 10px;color:var(--mut);font-size:12px}
</style></head>
<body>
<header>
  <div class="hrow">
    <h1>LoRa Chat + GPS + Kompas</h1>
    <span class="chip">AP: TBEAM_CHAT</span>
    <span class="chip small"><span id="stESP" class="dot red"></span> ESP32</span>
    <span class="chip small"><span id="stOLED" class="dot red"></span> OLED</span>
    <span class="chip small"><span id="stLoRa" class="dot red"></span> LoRa</span>
    <span class="chip small"><span id="stGPS" class="dot red"></span> GPS</span>
    <span class="chip small"><span id="stCMP" class="dot red"></span> Kompas</span>
    <span class="chip small"><span id="stMEM" class="dot red"></span> MEM</span>
  </div>
</header>
<main>
<section class="card">
  <div class="spread">
    <div class="row">
      <button id="btnMe" class="accent">Lokasi Saya</button>
      <span id="gpsDot" class="dot red" title="GPS status"></span>
      <span class="small">Merah=belum fix • Hijau=sudah fix</span>
    </div>
    <div class="row">
      <button id="btnSendLoc" class="ghost">Kirim Lokasi via LoRa</button>
      <button id="btnSOS" class="btnRed">SOS</button>
      <button id="btnArah" class="ghost">Arah</button>
    </div>
  </div>
  <div class="row" style="margin-top:10px"><div id="locVal">-</div></div>
  <div class="row" style="margin-top:12px">
    <div class="chip" id="hdgChip"><span id="hdgVal">--°</span> <span id="arrow" style="display:none;margin-left:6px">▲</span></div>
    <input id="targetCoord" type="text" placeholder="Koordinat teman (lat,lon)" />
    <button id="btnLock" class="btnGreen">Lock</button>
  </div>
</section>
<section class="card">
  <div class="spread">
    <div class="small">Ruang Chat • Maks 25 karakter • 10 pesan terbaru disimpan</div>
    <div class="chip small" id="msgCount">0/10</div>
  </div>
  <div class="cols">
    <div>
      <div class="colHead">Masuk</div>
      <div id="inList" class="list"></div>
    </div>
    <div>
      <div class="colHead">Keluar</div>
      <div id="outList" class="list"></div>
    </div>
  </div>
  <div class="footer">
    <input id="msg" type="text" maxlength="25" placeholder="Tulis pesan (≤25 karakter)"/>
    <button id="btnSend" class="accent">Kirim</button>
  </div>
</section>
<section class="card">
  <div class="spread">
    <div class="row"><span class="badge">Mode Long (MAX) — Jarak maks</span></div>
  </div>
  <div class="row">
    <input id="msgLong" type="text" maxlength="40" placeholder="Pesan MAX (≤40 karakter)"/>
    <button id="btnSendLong" class="btnRed">Kirim MAX</button>
    <span id="longHint" class="small"></span>
  </div>
</section>
</main>
<script>
const el=(id)=>document.getElementById(id)
const gpsDot=el('gpsDot'), hdgVal=el('hdgVal'), arrow=el('arrow'), btnLock=el('btnLock'), targetCoord=el('targetCoord')
function setDot(n,st){n.className='dot '+(st==='g'?'green':st==='y'?'yellow':'red')}
function renderMessages(arr){
  const inList=el('inList'), outList=el('outList')
  inList.innerHTML=''; outList.innerHTML=''
  const ins=arr.filter(m=>!m.out), outs=arr.filter(m=>m.out)
  ins.forEach(m=>{const d=document.createElement('div');d.className='bubble'+(m.isLoc?' loc':'');d.textContent=m.text;inList.appendChild(d)})
  outs.forEach(m=>{const d=document.createElement('div');d.className='bubble out'+(m.isLoc?' loc':'');d.textContent=m.text;outList.appendChild(d)})
  el('msgCount').textContent=(arr.length||0)+'/10'
  for(let i=arr.length-1;i>=0;i--){ if(arr[i].isLoc && !arr[i].out){ const t=arr[i].text.replace(/^LOC:\s*/,'').trim(); targetCoord.value=t; break; } }
}
async function loadMessages(){ try{ const j=await (await fetch('/messages')).json(); renderMessages(j) }catch(e){} }
async function fetchMe(){ try{ const j=await (await fetch('/me')).json(); if(j.fix&&j.coord){ el('locVal').textContent=j.coord; setDot(gpsDot,'g'); } else { el('locVal').textContent='Belum ada GPS fix'; setDot(gpsDot,'r'); } }catch(e){ el('locVal').textContent='Gagal ambil lokasi'; setDot(gpsDot,'r'); } }
async function loadHealth(){ try{ const j=await (await fetch('/health')).json(); setDot(el('stOLED'), j.oled); setDot(el('stLoRa'), j.lora); setDot(el('stGPS'), j.gps); setDot(el('stCMP'), j.cmp); setDot(el('stMEM'), j.mem); setDot(el('stESP'),'g'); }catch(e){ setDot(el('stESP'),'y') } }

el('btnMe').onclick=fetchMe
el('btnSendLoc').onclick=async()=>{ await fetch('/sendLoc',{method:'POST'}); await loadMessages() }
el('btnSOS').onclick=async()=>{ await fetch('/sos',{method:'POST'}); await loadMessages() }
el('btnSend').onclick=async()=>{ const t=el('msg').value.trim(); if(!t) return; if(t.length>25){ alert('Maks 25 karakter'); return } const p=new URLSearchParams(); p.set('text',t); await fetch('/send',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()}); el('msg').value=''; await loadMessages() }

el('btnArah').onclick=async()=>{
  try{
    const h=await (await fetch('/heading')).json();
    if(h.ok){
      const deg=Math.round(h.deg);
      hdgVal.textContent=deg+'°';
      arrow.style.display='inline-block';
      arrow.style.transform='rotate('+deg+'deg)';
      await fetch('/showHeading?ms=10000',{method:'POST'});
    }else{ alert(h.msg||'Kompas belum siap'); }
  }catch(e){ alert('Gagal membaca kompas'); }
}
el('btnSendLong').onclick=async()=>{
  const t=el('msgLong').value.trim();
  if(!t){ alert('Pesan kosong'); return }
  if(t.length>40){ alert('Maks 40 karakter'); return }
  const p=new URLSearchParams(); p.set('text',t);
  el('longHint').textContent='Mengirim MAX... Wi-Fi mungkin putus sementara';
  try{ await fetch('/maxSend',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()}); }catch(e){}
  setTimeout(()=>{ el('longHint').textContent=''; }, 6000);
}

setInterval(()=>{ loadMessages(); loadHealth() }, 1000)
loadMessages(); loadHealth()
</script>
</body></html>
)HTML";
// ===================== Helpers =====================
static inline double toRad(double d){ return d*DEG_TO_RAD; }
double bearingTo(double lat1,double lon1,double lat2,double lon2){
  double y = sin(toRad(lon2-lon1))*cos(toRad(lat2));
  double x = cos(toRad(lat1))*sin(toRad(lat2)) - sin(toRad(lat1))*cos(toRad(lat2))*cos(toRad(lon2-lon1));
  double b = atan2(y,x) * 180.0 / PI; if(b<0) b+=360.0; return b;
}
String fmtCoord(double v){char buf[24];dtostrf(v,0,6,buf);String s=String(buf);s.trim();return s;}
String makeCoordString(double lat,double lon){String s=fmtCoord(lat);s+=",";s+=fmtCoord(lon);return s;}
String jsonEscape(String s){s.replace("\\","\\\\");s.replace("\"","\\\"");return s;}
// ===================== OLED =====================
void oledTextCenter(const String &line1,const String &line2=""){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(line1,0,0,&x1,&y1,&w,&h);
  int cx=(OLED_W - (int)w)/2; int cy= (OLED_H/2) - 8;
  display.setCursor(cx, cy); display.println(line1);
  if(line2.length()){
    display.setTextSize(1);
    display.getTextBounds(line2,0,0,&x1,&y1,&w,&h);
    cx=(OLED_W - (int)w)/2; cy= cy+18;
    display.setCursor(cx, cy); display.println(line2);}
  display.display();}
void oledShowMessage(const String &msg){
  if(msg==lastShown) return;
  lastShown=msg;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int start=0; int y=0;
  while(start<(int)msg.length() && y<OLED_H-8){
    String row=msg.substring(start,start+21);
    display.setCursor(0,y);
    display.println(row);
    y+=12;
    start+=21;}
  display.display();}
void oledDrawArrow(){ display.fillTriangle(OLED_W/2, 8, OLED_W/2-8, 24, OLED_W/2+8, 24, SSD1306_WHITE); }
void oledShowHeading(float deg){
  display.clearDisplay();
  oledDrawArrow();
  String s = String((int)round(deg)) + "°";
  display.setTextSize(2); display.setTextColor(SSD1306_WHITE);
  int16_t x1,y1; uint16_t w,h; display.getTextBounds(s,0,0,&x1,&y1,&w,&h);
  int cx=(OLED_W - (int)w)/2;
  display.setCursor(cx, OLED_H/2+10); display.println(s);
  display.display();
}
// ===================== Persistence =====================
void persistRing(){
  prefs.begin("chat",false);
  prefs.putUInt("cnt",ringCount);
  prefs.putUInt("lastId",lastId);
  for(int i=0;i<ringCount;i++){
    String km="msg"+String(i);
    String kt="typ"+String(i);
    String ko="out"+String(i);
    prefs.putString(km.c_str(),ringBuf[i].text);
    prefs.putBool(kt.c_str(),ringBuf[i].isLoc);
    prefs.putBool(ko.c_str(),ringBuf[i].out);
  }
  prefs.putBool("lock", lockActive);
  prefs.putString("tcoord", targetCoordStr);
  prefs.end();
}
void restoreFromNVS(){
  prefs.begin("chat",false);
  ringCount=prefs.getUInt("cnt",0);
  lastId=prefs.getUInt("lastId",0);
  if(ringCount>MAX_MSG) ringCount=MAX_MSG;
  for(int i=0;i<ringCount;i++){
    String km="msg"+String(i);
    String kt="typ"+String(i);
    String ko="out"+String(i);
    ringBuf[i].text=prefs.getString(km.c_str(),"");
    ringBuf[i].isLoc=prefs.getBool(kt.c_str(),false);
    ringBuf[i].out=prefs.getBool(ko.c_str(),false);
    ringBuf[i].id=++lastId;}
  lockActive=prefs.getBool("lock", false);
  targetCoordStr=prefs.getString("tcoord", "");
  prefs.end();
  if(targetCoordStr.length()){
    int c = targetCoordStr.indexOf(',');
    if(c>0){ targetLat = targetCoordStr.substring(0,c).toDouble(); targetLon = targetCoordStr.substring(c+1).toDouble(); }}
  if(ringCount>0) oledShowMessage(ringBuf[ringCount-1].text); else oledTextCenter("T-Beam","Ready");}
// ===================== Messaging (normal) =====================
void pushMessage(const String &text,bool isLoc,bool fromMe){
  lastId++;
  MsgItem m; m.text=text; m.isLoc=isLoc; m.out=fromMe; m.id=lastId;
  if(ringCount<MAX_MSG) ringBuf[ringCount++]=m;
  else { for(int i=1;i<MAX_MSG;i++) ringBuf[i-1]=ringBuf[i]; ringBuf[MAX_MSG-1]=m; }
  if(!showingHeading) oledShowMessage(text);
  lastMsgMillis=millis();
  persistRing();}
void sendLoRaNow(const String &payload){
  LoRa.beginPacket(); LoRa.print(payload); LoRa.endPacket();
  lastLoRaActivity=millis();}
void sendLoRaDefault(const String &payload){
  RadioProfile prev = radioProf;
  applyProfile(RP_DEFAULT);
  sendLoRaNow(payload);
  applyProfile(prev);}
void shortBeep(uint32_t ms){ shortBeepOn=true; shortBeepUntil=millis()+ms; }
void startSosBeep(uint32_t ms){
  uint32_t now=millis();
  if(now < sosBuzzUntil) sosBuzzUntil += ms; else sosBuzzUntil = now + ms;}
String coordNow(bool &ok){
  ok=false;
  String coord="";
  if(gps.location.isValid() && gps.location.age()<2000){
    ok=true; coord=makeCoordString(gps.location.lat(), gps.location.lng());
  } else if(lastCoord.length() && (millis()-lastCoordMillis)<10000){
    ok=true; coord=lastCoord;
  }
  return coord;
}
void handleIncomingPayload(const String &p){
  if(p.startsWith("MSG:")){
    pushMessage(p.substring(4), false, false); shortBeep(300);
  }
  else if(p.startsWith("LOC:")){
    String t = p.substring(4); t.trim();
    pushMessage("LOC: " + t, true, false);
    targetCoordStr=t; int c=t.indexOf(','); if(c>0){ targetLat=t.substring(0,c).toDouble(); targetLon=t.substring(c+1).toDouble(); }
    shortBeep(300);
  }
  else if(p.startsWith("SOS")){
    pushMessage(p, true, false); startSosBeep(30000);
  }
  lastLoRaActivity=millis();
}
// ===================== GPS & QMC =====================
void readGPS(){
  while(GPSSerial.available()){
    char c = GPSSerial.read();
    gps.encode(c);
    if(gps.location.isUpdated()){
      lastCoord = makeCoordString(gps.location.lat(), gps.location.lng());
      lastCoordMillis = millis();
      curLat = gps.location.lat();
      curLon = gps.location.lng();
    }
  }
}
bool sampleHeadingOnce(float &outDeg, int nsamp=6, int gap_ms=8){
  if (millis() - lastLoRaActivity < 15) delay(15);
  qmc.init(); qmc.setMode(1,2,0,3); // Continuous, 50Hz, ±2G, OSR512
  long sx=0, sy=0;
  for(int i=0;i<nsamp;i++){
    qmc.read(); int x=qmc.getX(), y=qmc.getY();
  #if USE_QMC_CALIB
    float cx=(x - MX_OFF) * MX_SCL;
    float cy=(y - MY_OFF) * MY_SCL;
    sx += (long)round(cx); sy += (long)round(cy);
  #else
    sx += x; sy += y;
  #endif
    delay(gap_ms);
  }
  float hx = (float)sx / nsamp, hy = (float)sy / nsamp;
  float mag2 = hx*hx + hy*hy; if (mag2 < 1e2f) return false;
  float deg = atan2f(hy, hx) * 180.0f / PI; if (deg < 0) deg += 360.0f;
  outDeg = deg; headingDeg = deg; lastCompassMillis = millis(); mag_ok = true; return true;
}
// ===================== HTTP Handlers (normal) =====================
void handleRoot(){ server.setContentLength(strlen_P(INDEX_HTML)); server.send_P(200,"text/html",INDEX_HTML); }
void handleMe(){
  bool fix=false; String coord="";
  if(gps.location.isValid() && gps.location.age()<2000){ fix=true; coord=makeCoordString(gps.location.lat(),gps.location.lng()); lastCoord=coord; lastCoordMillis=millis(); }
  else if(lastCoord.length() && (millis()-lastCoordMillis)<10000){ fix=true; coord=lastCoord; }
  String out="{\"fix\":"; out += (fix?"true":"false"); if(fix) out += ",\"coord\":\""+coord+"\""; out += "}";
  server.send(200,"application/json",out);
}
void handleSend(){
  String text; if(server.hasArg("text")) text=server.arg("text"); text.trim();
  if(text.length()==0){ server.send(400,"application/json","{\"ok\":false,\"msg\":\"Pesan kosong\"}"); return; }
  if(text.length()>25){ server.send(400,"application/json","{\"ok\":false,\"msg\":\"Maks 25 karakter\"}"); return; }
  String payload="MSG:"+text; sendLoRaDefault(payload); pushMessage(text,false,true);
  server.send(200,"application/json","{\"ok\":true}");
}
void handleSendLoc(){
  bool ok=false; String c=coordNow(ok);
  if(!ok){ server.send(200,"application/json","{\"ok\":false,\"msg\":\"GPS belum fix\"}"); return; }
  String payload="LOC:"+c; sendLoRaDefault(payload); pushMessage("LOC: "+c,true,true);
  server.send(200,"application/json","{\"ok\":true}");
}
void handleMessages(){
  String out="[";
  for(int i=0;i<ringCount;i++){
    if(i) out+=",";
    out+="{\"text\":\""+jsonEscape(ringBuf[i].text)+"\",\"isLoc\":"+(ringBuf[i].isLoc?"true":"false")+",\"out\":"+(ringBuf[i].out?"true":"false")+"}";
  }
  out+="]";
  server.send(200,"application/json",out);
}
void handleHeading(){
  float deg=0; bool ok = sampleHeadingOnce(deg);
  if(!ok){ server.send(200,"application/json","{\"ok\":false,\"msg\":\"Kompas belum siap\"}"); return; }
  String out = "{\"ok\":true,\"deg\":"+String(deg,1)+"}";
  server.send(200,"application/json",out);
}
void handleShowHeading(){
  int ms = 10000; if (server.hasArg("ms")) { int v = server.arg("ms").toInt(); if (v>=1000 && v<=60000) ms=v; }
  float deg=0; bool ok = sampleHeadingOnce(deg);
  if(!ok){ server.send(200,"application/json","{\"ok\":false,\"msg\":\"Kompas belum siap\"}"); return; }
  oledShowHeading(deg); showingHeading = true; showHeadingUntil = millis() + (uint32_t)ms;
  server.send(200,"application/json","{\"ok\":true}");
}
void handleLock(){
  String state = server.hasArg("state")? server.arg("state") : "0";
  if(state=="1"){
    String t = server.hasArg("target")? server.arg("target") : targetCoordStr; t.trim();
    int c=t.indexOf(','); if(c<=0){ server.send(400,"application/json","{\"ok\":false,\"msg\":\"Target tidak valid\"}"); return; }
    targetLat = t.substring(0,c).toDouble(); targetLon = t.substring(c+1).toDouble(); targetCoordStr = t; lockActive = true;
  } else lockActive = false;
  persistRing(); server.send(200,"application/json","{\"ok\":true}");
}

void handleSOS(){
  bool ok=false; String c=coordNow(ok);
  if(!ok){ server.send(200,"application/json","{\"ok\":false,\"msg\":\"GPS belum fix\"}"); return; }
  String payload="SOS : "+c; sendLoRaDefault(payload); pushMessage(payload, true, true);
  server.send(200,"application/json","{\"ok\":true}");
}

void handleHealth(){
  char s_oled = display_ok ? 'g' : 'r';
  uint32_t now=millis();
  char s_lora = lora_ok ? ((now-lastLoRaActivity<10000)?'g':(now-lastLoRaActivity<60000?'y':'r')) : 'r';
  char s_gps='r'; if(gps.location.isValid()){ unsigned long age=gps.location.age(); if(age<2000) s_gps='g'; else if(age<10000) s_gps='y'; else s_gps='r'; }
  char s_cmp = (now - lastCompassMillis < 15000) ? 'g' : 'y';
  uint32_t freeh=ESP.getFreeHeap(); char s_mem = freeh>100000?'g':(freeh>50000?'y':'r');
  String out="{"; out+="\"oled\":\""; out+=s_oled; out+="\"";
  out+=",\"lora\":\""; out+=s_lora; out+="\""; out+=",\"gps\":\""; out+=s_gps; out+="\"";
  out+=",\"cmp\":\""; out+=s_cmp; out+="\""; out+=",\"mem\":\""; out+=s_mem; out+="\""; out+="}";
  server.send(200,"application/json",out);
}

// ===================== MODE LONG (MAX) =====================
static const int  MAX_LONG_LEN       = 40;     
static const int  MAX_REQ_WINDOW_MS  = 3000;   
static const int  MAX_RANGE_INTERVAL = 900;    
static const int  MAX_PREP_DELAY     = 150;    
static const int  MAX_RESTORE_DELAY  = 150;    
bool     maxBusy=false;
String   maxBuf="";
uint32_t maxId=0;
uint8_t  maxTxCount=0;
uint32_t maxLastTx=0;
enum MaxStage { MAX_IDLE, MAX_PREP, MAX_REQ_TX, MAX_MSG_TX, MAX_RESTORE };
MaxStage maxStage = MAX_IDLE;
uint32_t maxListenUntil=0;
uint32_t rxExpectId=0;
uint32_t lastMaxSeenId=0;
void startMaxSend(const String &text){
  if(maxBusy) return;
  maxBuf = text; maxId = (uint32_t) (millis() ^ ESP.getEfuseMac());
  maxTxCount = 0; maxLastTx = 0;
  maxBusy = true; maxStage = MAX_PREP;
}
void maxLoop(){
  if(!maxBusy) return;
  switch(maxStage){
    case MAX_PREP:
      oledPower(false); wifiOff();
      delay(MAX_PREP_DELAY);
      applyProfile(RP_DEFAULT); 
      maxStage = MAX_REQ_TX;
      break;

    case MAX_REQ_TX: {
      String req = "MAX_REQ:" + String(maxId) + ":" + String(MAX_REQ_WINDOW_MS);
      sendLoRaNow(req);
      applyProfile(RP_RANGE);
      maxTxCount = 0; maxLastTx = 0;
      maxStage = MAX_MSG_TX;
      break;
    }
    case MAX_MSG_TX:
      if(maxTxCount < 2){
        if(maxTxCount==0 || (millis() - maxLastTx >= (uint32_t)MAX_RANGE_INTERVAL)){
          String pl = "MAX_MSG:" + String(maxId) + ":" + maxBuf;
          sendLoRaNow(pl);
          maxTxCount++; maxLastTx = millis();
        }
      }else{
        delay(MAX_RESTORE_DELAY);
        applyProfile(RP_DEFAULT);
        wifiOn(); oledPower(true);
        pushMessage("[MAX] " + maxBuf, false, true);
        maxStage = MAX_RESTORE;
      }
      break;

    case MAX_RESTORE:
      maxBusy=false; maxStage=MAX_IDLE;
      break;

    default: break;
  }
}
void handleMaxReq(const String &p){ 
  int c1 = p.indexOf(':',7); if(c1<0) return;
  int c2 = p.indexOf(':',c1+1); if(c2<0) return;
  uint32_t id = (uint32_t) strtoul(p.substring(8,c1).c_str(), nullptr, 10);
  int win = p.substring(c1+1).toInt(); if(win < 1000 || win > 8000) win = MAX_REQ_WINDOW_MS;
  rxExpectId = id;
  applyProfile(RP_RANGE);
  maxListenUntil = millis() + (uint32_t)win;
}
void handleMaxMsg(const String &p){ 
  int c1 = p.indexOf(':',8); if(c1<0) return;
  uint32_t id = (uint32_t) strtoul(p.substring(8,c1).c_str(), nullptr, 10);
  String text = p.substring(c1+1);
  if(id == lastMaxSeenId) return;
  lastMaxSeenId = id;

  pushMessage("[MAX] " + text, false, false);
  shortBeep(300);
}
// ===================== HTTP Mode Long =====================
void handleMaxSend(){
  String text; if(server.hasArg("text")) text=server.arg("text"); text.trim();
  if(text.length()==0){ server.send(400,"application/json","{\"ok\":false,\"msg\":\"Pesan kosong\"}"); return; }
  if(text.length()>MAX_LONG_LEN){ server.send(400,"application/json","{\"ok\":false,\"msg\":\"Maks 40 karakter\"}"); return; }

  startMaxSend(text);
  server.send(200,"application/json","{\"ok\":true}");
}
// ===================== Setup & Loop =====================
void setup(){
  Serial.begin(115200); delay(150);
  Wire.begin(21,22);
  display_ok = display.begin(SSD1306_SWITCHCAPVCC,OLED_ADDR);
  if(display_ok){ display.clearDisplay(); display.display(); display.setRotation(0); oledTextCenter("Booting",""); }
  GPSSerial.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_SS);
  LoRa.setPins(LORA_SS,LORA_RST,LORA_DIO0);
  lora_ok = LoRa.begin(LORA_FREQ);
  if(lora_ok){ LoRa.setTxPower(17); setLoRaDefault(); }
  ledcSetup(BUZZER_CH, 2000, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  ledcWrite(BUZZER_CH, 0);
  pinMode(SOS_BTN_PIN, INPUT_PULLUP);
  qmc.init(); qmc.setMode(1,2,0,3);
  wifiOn();
  server.on("/", handleRoot);
  server.on("/me", HTTP_GET, handleMe);
  server.on("/send", HTTP_POST, handleSend);
  server.on("/sendLoc", HTTP_POST, handleSendLoc);
  server.on("/messages", HTTP_GET, handleMessages);
  server.on("/heading", HTTP_GET, handleHeading);
  server.on("/showHeading", HTTP_POST, handleShowHeading);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/lock", HTTP_POST, handleLock);
  server.on("/sos", HTTP_POST, handleSOS);
  server.on("/maxSend", HTTP_POST, handleMaxSend); 
  server.begin();
  restoreFromNVS();
  lastMsgMillis = millis();
  lastLoRaActivity = millis();
}

void loop(){
  server.handleClient();
  readGPS();
  if (showingHeading && millis() > showHeadingUntil){
    showingHeading = false;
    if (ringCount>0) oledShowMessage(ringBuf[ringCount-1].text);
    else oledTextCenter("T-Beam","Ready");
  }
  int pSize=LoRa.parsePacket();
  if(pSize){
    String payload=""; while(LoRa.available()) payload+=(char)LoRa.read();
    payload.trim();
    if(payload.startsWith("MAX_REQ:"))      handleMaxReq(payload);
    else if(payload.startsWith("MAX_MSG:")) handleMaxMsg(payload);
    else if(payload.length())               handleIncomingPayload(payload);
  }
  if(maxListenUntil && (int32_t)(millis() - maxListenUntil) > 0){
    applyProfile(RP_DEFAULT);
    maxListenUntil = 0;
  }
  bool btn = digitalRead(SOS_BTN_PIN);
  if(btn!=btnPrev && millis()-btnDebounceAt>50){
    btnDebounceAt=millis(); if(btn==LOW) handleSOS(); btnPrev=btn;
  }
  uint32_t now=millis();
  bool sosActive = now < sosBuzzUntil;
  bool shortActive = shortBeepOn && now < shortBeepUntil;
  if(sosActive || shortActive) ledcWrite(BUZZER_CH,128);
  else { ledcWrite(BUZZER_CH,0); shortBeepOn=false; }
  maxLoop();
}
