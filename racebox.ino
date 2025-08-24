#include <TFT_eSPI.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>
#include <algorithm>
#include <stdarg.h>  // buat va_list


// ================== PIN & CONFIG ==================
#define GPS_RX 27
#define GPS_TX 22
#define GPS_BAUD 115200
#define BTN_RESET 0

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

// SD - VSPI
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK  18
#ifndef SD_CS
#define SD_CS   5
#endif

// ================== UI ZONES ==================
struct Zone { int x, y, w, h; };
const Zone ZONE_STATUS_BAR   = {0, 0, 320, 18};
const Zone ZONE_GPS_INDICATOR= {270, 2, 45, 14};
const Zone ZONE_SPEED_MAIN   = {20, 25, 180, 65};
const Zone ZONE_SPEED_UNIT   = {205, 70, 50, 20};
const Zone ZONE_TIMER        = {220, 25, 90, 40}; // sedikit lebih tinggi utk IP
const Zone ZONE_STATUS       = {220, 65, 90, 20};
const Zone ZONE_PROGRESS     = {20, 95, 280, 8};
const Zone ZONE_TABLE        = {10, 115, 300, 115};

#define COLOR_BG       TFT_BLACK
#define COLOR_PRIMARY  TFT_CYAN
#define COLOR_SECONDARY TFT_YELLOW
#define COLOR_SUCCESS  TFT_GREEN
#define COLOR_WARNING  TFT_ORANGE
#define COLOR_ERROR    TFT_RED
#define COLOR_TEXT     TFT_WHITE
#define COLOR_DIM      TFT_DARKGREY
#define COLOR_ACCENT   TFT_MAGENTA

// ====== THRESHOLDS ======
#define MIN_START_SPEED        3.0f   // km/h
#define MIN_COUNTING_SPEED     3.0f   // km/h
#define GPS_ACCURACY_THRESHOLD 2.0f   // meter
#define STOP_SPEED_THRESHOLD   2.0f   // km/h

// ================== GLOBALS ==================
HardwareSerial GPS(2);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprStatusBar = TFT_eSprite(&tft);
TFT_eSprite sprSpeedMain  = TFT_eSprite(&tft);
TFT_eSprite sprTimer      = TFT_eSprite(&tft);
TFT_eSprite sprTable      = TFT_eSprite(&tft);

WebServer server(80);
bool wifiConnected = false;
String wifiIpStr = "--";
bool sdReady = false;

// RMC date/time
char rmc_date[7] = ""; // ddmmyy
char rmc_time[7] = ""; // hhmmss

// GPS state
String hdop = "-";
int satellites = 0;
float speedKmh = 0.0f, filteredSpeed = 0.0f, totalDistance = 0.0f;
float lastLat = 0.0f, lastLon = 0.0f, lastValidLat = 0.0f, lastValidLon = 0.0f;
bool firstFix = true;

float kalmanP=1.0f, kalmanQ=0.3f, kalmanR=1.2f, kalmanX=0.0f, kalmanK=0.0f;
const int STOP_CONFIRM_SAMPLE = 2;
static int stopCounter = 0;

// Milestones
const float milestoneDist[]   = {18.288, 60.96, 100, 200, 402.336, 500, 804.672, 1000, 1609.347};
const char* milestoneLabels[] = {"60ft","200ft","100m","200m","1/4mi","500m","1/2mi","1km","1mi"};
const int N_MILESTONE = sizeof(milestoneDist)/sizeof(milestoneDist[0]);
struct TimeTrap { float time; float speed; bool achieved; bool isTarget; };
TimeTrap milestones[N_MILESTONE];
float time_0_100=0, dist_0_100=0; bool achieved_0_100=false;
float maxSpeed = 0.0f;

// Sat info
int gpsFixQuality = 0;
#define MAX_SAT 16
struct SatInfo { uint8_t prn; uint8_t snr; bool used; };
SatInfo satList[MAX_SAT];
uint8_t nSatInView=0, nSatInUse=0;
char gpsLat[24] = "-";
char gpsLon[24] = "-";
char gpsAlt[16] = "-";
char gpsFixType[8] = "-";
char gpsMode[8] = "-";

// last cache (boot screen + status)
float last_hdop = -99;
int last_nSatInUse=-1, last_nSatInView=-1, last_gpsFixQuality=-1;
char last_gpsMode[8]="", last_gpsFixType[8]="", last_gpsLat[24]="", last_gpsLon[24]="", last_gpsAlt[16]="";
String last_wifi_ip=""; bool last_wifi_conn=false; bool last_sd_ready=false; int last_race_count=-1;

// FSM
enum SystemState { STATE_NOT_READY, STATE_READY, STATE_RUNNING, STATE_FINISHED, STATE_FAILED };
SystemState currentState = STATE_NOT_READY;
unsigned long startTime = 0; float startDistance = 0;
unsigned long lastButtonPress=0; const unsigned long BUTTON_DEBOUNCE=400;
bool needsFullRedraw=true; unsigned long lastUIUpdate=0; const unsigned long UI_UPDATE_INTERVAL=50;
unsigned long failedTime=0;

// ================== LOGGING (SD ring buffer, 15Hz) ==================
File runFile;
bool loggingActive = false;
char tmpPath[64] = {0};
uint32_t lastFlushPos = 0;

#ifndef LOG_RING_CAP
#define LOG_RING_CAP 128      // ~8.5s @15Hz per batch
#endif

struct LogRow {
  uint32_t ts_ms;
  float elapsed_s, dist_m, v_raw, v_filt, lat, lon, hdop;
  uint16_t sats;
  uint8_t  type; // 0=RMC,1=GGA,2=GSA,3=GSV
};
LogRow logRing[LOG_RING_CAP];
uint16_t logHead = 0;

// Known WiFi
struct KnownNet { const char* ssid; const char* pass; };
KnownNet knownList[] = {
  {"Wicaksu", "wicaksuu"},
  {"Rumah",   "passwordrumah"},
  {"Kantor",  "passwordkantor"},
};
const int N_KNOWN = sizeof(knownList)/sizeof(knownList[0]);

// === GPS Hz monitor ===
float gpsHz = 0.0f;
uint32_t _hzWinStart = 0;
uint16_t _hzGGAcount = 0;

// ================== FWD DECL ==================
void drawBootScreenWelcome();
void drawStaticElements();
void updateDisplay();
void drawStatusBar();
void drawGPSStatus();
void drawSpeedDisplay();
void drawTimerDisplay();
void drawMilestoneTable();
void resetSystem();
void startRun();
void finishRun();
void updateMilestones();
bool allMilestonesAchieved();
void readGPS();
void parseRMC(const String& sentence);
void parseGGA(const String& sentence);
void parseGSA(const String& sentence);
void parseGSV(const String& sentence);
void parseVTG(const String& sentence);
void splitString(const String& str, char delimiter, String output[], int maxParts);
float convertToDecimal(float raw, const String& direction);
float haversine(float lat1, float lon1, float lat2, float lon2);
void updateTimerLogic();
void updateSpeedFilter();
void registerWebRoutes();
int  countRaceFiles();
void handleIndex();
void handleRaceDetail();
void handleApiRace();
void handleDownload();
void handleDelete();
void handleResetRun();
void handleDeleteAll();
void handleApiRacePage();


/************  AUTO BAUD + 20 Hz SETUP (u-blox M10)  ************/

// ===== On-screen logger untuk GPS setup =====

// >>> Tambahkan ini:
TFT_eSprite sprGpsSetup = TFT_eSprite(&tft);

static const int GPSLOG_MAX = 12;
static String gpsLogLines[GPSLOG_MAX];
static int gpsLogCount = 0;

static void gpsSetupRender() {
  sprGpsSetup.fillSprite(COLOR_BG);
  // Title
  sprGpsSetup.setTextFont(2);
  sprGpsSetup.setTextColor(COLOR_PRIMARY, COLOR_BG);
  sprGpsSetup.setCursor(8, 6);
  sprGpsSetup.print("GPS Setup");

  // Garis pemisah
  sprGpsSetup.drawFastHLine(8, 24, SCREEN_WIDTH - 16, COLOR_DIM);

  // Isi log
  sprGpsSetup.setTextFont(1);
  sprGpsSetup.setTextColor(COLOR_TEXT, COLOR_BG);
  int y = 30;
  int start = gpsLogCount > GPSLOG_MAX ? (gpsLogCount - GPSLOG_MAX) : 0;
  for (int i = start; i < gpsLogCount; ++i) {
    int idx = i % GPSLOG_MAX;
    sprGpsSetup.setCursor(8, y);
    sprGpsSetup.print(gpsLogLines[idx]);
    y += 12;
  }
  sprGpsSetup.pushSprite(0, 0);
}

static void gpsSetupStart() {
  sprGpsSetup.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
  gpsLogCount = 0;
  gpsSetupRender();
}

static void gpsSetupDone() {
  sprGpsSetup.deleteSprite();
}

static void gpsLogf(const char* fmt, ...) {
  char buf[160];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  // Mirror ke Serial:
  Serial.println(buf);

  // Simpan ke buffer & render
  gpsLogLines[gpsLogCount % GPSLOG_MAX] = String(buf);
  gpsLogCount++;
  gpsSetupRender();
}


// ====== BAUD scan list ======
static const uint32_t BAUDS[] = {9600, 38400, 57600, 115200, 230400};

// --- util kecil
static void flushGPS() { while (GPS.available()) GPS.read(); }

// Deteksi ada NMEA (“$GP…/$GN…”) dalam ms tertentu
static bool seeNMEA(uint32_t ms) {
  uint32_t t0 = millis(); int lines=0; String s;
  while (millis()-t0 < ms) {
    int c = GPS.read();
    if (c < 0) { delay(1); continue; }
    char ch=(char)c;
    if (ch=='\n') { if (s.startsWith("$GP") || s.startsWith("$GN")) { lines++; if (lines>=1) return true; } s=""; }
    else if (ch!='\r') s+=ch;
  }
  return false;
}

// Scan beberapa baud sampai ketemu NMEA
static bool scanBaud(uint32_t &found) {
  for (uint32_t b: BAUDS) {
    GPS.begin(b, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(150);
    flushGPS();
    if (seeNMEA(600)) { found = b; return true; }
  }
  return false;
}

void ubxSend(uint8_t cls, uint8_t id, const uint8_t* pl, uint16_t len){
  auto upd=[&](uint8_t x, uint8_t& A, uint8_t& B){ A = (uint8_t)(A + x); B = (uint8_t)(B + A); };
  uint8_t A=0,B=0;
  GPS.write(0xB5); GPS.write(0x62);
  GPS.write(cls);  upd(cls,A,B);
  GPS.write(id);   upd(id,A,B);
  GPS.write((uint8_t)(len & 0xFF)); upd((uint8_t)(len & 0xFF),A,B);
  GPS.write((uint8_t)(len >> 8));   upd((uint8_t)(len >> 8),A,B);
  for (uint16_t i=0;i<len;i++){ GPS.write(pl[i]); upd(pl[i],A,B); }
  GPS.write(A); GPS.write(B); GPS.flush();
}
bool ubxWaitAck(uint8_t cls, uint8_t id, uint32_t to=500){
  uint32_t t0=millis(); int st=0; uint8_t gotCls=0, gotId=0; bool isAck=false;
  while (millis()-t0 < to){
    if (!GPS.available()) { delay(1); continue; }
    uint8_t b=GPS.read();
    switch(st){
      case 0: st=(b==0xB5)?1:0; break;
      case 1: st=(b==0x62)?2:0; break;
      case 2: if (b==0x05){ st=3; } else st=0; break;               // ACK/NAK class
      case 3: isAck=(b==0x01); st=4; break;                         // 0x01=ACK,0x00=NAK
      case 4: st=5; break;                                          // len L
      case 5: st=6; break;                                          // len H
      case 6: gotCls=b; st=7; break;                                // payload[0]
      case 7: gotId=b; goto done;
    }
  }
done:
  return (gotCls==cls && gotId==id && isAck);
}
// ===== VALSET (pakai ubxSend/ubxWaitAck milikmu) =====
static void putKeyU1(uint8_t*& p, uint32_t key, uint8_t  v){ memcpy(p,&key,4); p+=4; *p++=v; }
static void putKeyU2(uint8_t*& p, uint32_t key, uint16_t v){ memcpy(p,&key,4); p+=4; *p++=(uint8_t)v; *p++=(uint8_t)(v>>8); }
static void putKeyU4(uint8_t*& p, uint32_t key, uint32_t v){ memcpy(p,&key,4); p+=4; memcpy(p,&v,4); p+=4; }
static void putKeyL1(uint8_t*& p, uint32_t key, bool v)    { memcpy(p,&key,4); p+=4; *p++=(uint8_t)(v?1:0); }

static bool sendVALSET_RAM(void (*fill)(uint8_t*&)) {
  uint8_t pay[128]; uint8_t* p=pay;
  *p++=0x00;              // version
  *p++=0x01;              // layer RAM
  *p++=0x00; *p++=0x00;   // reserved
  fill(p);
  uint16_t L = (uint16_t)(p - pay);
  ubxSend(0x06,0x8A,pay,L);
  return ubxWaitAck(0x06,0x8A,800);
}

// ===== Keys (M10) =====
#define K_RATE_MEAS        0x30210001u // U2 ms
#define K_RATE_NAV         0x30210002u // U2 cycles
#define K_UART1_BAUD       0x40520001u // U4
#define K_UART1IN_UBX      0x10730001u // L1
#define K_UART1IN_NMEA     0x10730002u // L1
#define K_UART1OUT_NMEA    0x10740002u // L1
#define K_MSG_GGA_UART1    0x209100BBu // U1
#define K_MSG_RMC_UART1    0x209100ACu // U1
#define K_MSG_GSA_UART1    0x209100C0u // U1
#define K_MSG_GSV_UART1    0x209100C5u // U1
#define K_MSG_GLL_UART1    0x209100CAu // U1
#define K_MSG_VTG_UART1    0x209100B1u // U1
#define K_MSG_ZDA_UART1    0x209100D9u // U1

// ===== Fallback: enable UBX input via NMEA $PUBX,41 =====
static void sendPUBX41(uint32_t baud){
  auto sendNMEA=[&](const char* payload){
    uint8_t c=0; for (const char* p=payload; *p; ++p) c^=(uint8_t)(*p);
    char buf[96]; snprintf(buf,sizeof(buf),"$%s*%02X\r\n",payload,c);
    GPS.print(buf); GPS.flush();
  };
  char pl[64];
  // $PUBX,41,1,0003,0001,<baud>,0  -> in: UBX+NMEA, out: NMEA, port 1 (UART1)
  snprintf(pl,sizeof(pl),"PUBX,41,1,0003,0001,%u,0",(unsigned)baud);
  sendNMEA(pl);
  delay(150);
  GPS.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(150);
}

// Verifikasi kasar: hitung GGA per 2.5 detik
static float verifyGGA(uint32_t ms=2500){
  flushGPS();
  uint32_t t0=millis(); uint16_t gga=0; String s;
  while (millis()-t0<ms){
    int c=GPS.read(); if (c<0){ delay(1); continue; }
    char ch=(char)c;
    if (ch=='\n'){ if (s.startsWith("$GPGGA")||s.startsWith("$GNGGA")) gga++; s=""; }
    else if (ch!='\r') s+=ch;
  }
  return gga / (ms/1000.0f);
}

// Apply 20Hz (meas=50ms) + hanya GGA/RMC pada baud saat ini
static bool apply10Hz_currentBaud(){
  bool ok = sendVALSET_RAM([](uint8_t*& p){
    // 20 Hz => meas=50 ms (kalau mau 10 Hz: 100 ms)
    putKeyU2(p, K_RATE_MEAS, 50);
    putKeyU2(p, K_RATE_NAV , 1);
    // Protocol: in UBX+NMEA, out NMEA
    putKeyL1(p, K_UART1IN_UBX , 1);
    putKeyL1(p, K_UART1IN_NMEA, 1);
    putKeyL1(p, K_UART1OUT_NMEA,1);
    // NMEA message rates (per solution)
    putKeyU1(p, K_MSG_GGA_UART1, 1);
    putKeyU1(p, K_MSG_RMC_UART1, 1);
    putKeyU1(p, K_MSG_GSA_UART1, 0);
    putKeyU1(p, K_MSG_GSV_UART1, 0);
    putKeyU1(p, K_MSG_GLL_UART1, 0);
    putKeyU1(p, K_MSG_VTG_UART1, 0);
    putKeyU1(p, K_MSG_ZDA_UART1, 0);
  });
  gpsLogf("  [ACK] VALSET 20Hz: %s", ok ? "OK" : "FAIL");
  return ok;
}

// --- pindah baud ke 115200 tanpa ubah rate 10Hz ---
static bool switchTo115200(){
  bool okB = sendVALSET_RAM([](uint8_t*& p){ putKeyU4(p, K_UART1_BAUD, 115200); });
  gpsLogf("  [ACK] Set BAUD 115200 (VALSET): %s", okB?"OK":"FAIL");
  delay(120);
#if ARDUINO >= 10805
  GPS.updateBaudRate(115200);
#else
  GPS.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
#endif
  delay(150);

  if (!seeNMEA(700)) {
    gpsLogf("  [FALLBACK] $PUBX,41 -> 115200 …");
    sendPUBX41(115200);
    if (!seeNMEA(700)) {
      gpsLogf("  [ERR] NMEA belum terlihat @115200.");
      return false;
    }
  }
  gpsLogf("  [OK] NMEA aktif @115200.");
  return true;
}

// --- orkestrator: deteksi baud → (kalau bukan 115200) switch baud → set 10Hz → verifikasi ---
void gpsAutoSetup10Hz() {
  gpsLogf("[GPS] Auto-detect baud → force 20Hz @115200…");

  // 1) deteksi baud awal
  uint32_t cur=0;
  if (!scanBaud(cur)) { gpsLogf("  [ERR] NMEA tidak ditemukan. Cek wiring RX/TX & port UART modul."); return; }
  gpsLogf("  [OK] NMEA terdeteksi @ %u bps", (unsigned)cur);

  // 2) kalau bukan 115200 → pindah dulu ke 115200
  if (cur != 115200) {
    gpsLogf("  [STEP] Switch ke 115200 dulu supaya bandwidth lega…");
    if (!switchTo115200()) { gpsLogf("  [FAIL] Gagal switch ke 115200. Stop."); return; }
  } else {
#if ARDUINO >= 10805
    GPS.updateBaudRate(115200);
#else
    GPS.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
#endif
    delay(120);
  }

  // 3) apply 20Hz + cuma GGA/RMC
  gpsLogf("  [STEP] Set 20Hz + GGA/RMC only (VALSET) @115200…");
  apply10Hz_currentBaud();
  float hz = verifyGGA(2500);
  gpsLogf("  [VERIFY] GGA ~= %.1f Hz @115200", hz);
  if (hz >= 18.0f) { gpsLogf("  [DONE] 20Hz OK @115200."); return; }

  // 4) kalau belum, enable UBX input via PUBX lalu apply ulang
  gpsLogf("  [FALLBACK] Enable UBX via $PUBX,41 lalu apply 20Hz ulang…");
  sendPUBX41(115200);
  apply10Hz_currentBaud();
  hz = verifyGGA(2500);
  gpsLogf("  [VERIFY] GGA ~= %.1f Hz @115200", hz);
  if (hz >= 18.0f) { gpsLogf("  [DONE] 20Hz OK setelah PUBX."); return; }

  gpsLogf("  [FAIL] Masih <20Hz. Modul mungkin menolak setting/port bukan UART1.");
}

/************  END AUTO BAUD + 20 Hz SETUP  ************/





// logging helpers
void openRunTempFile();
void finalizeRunFile(bool success);
inline void appendToRing(uint8_t typeCode);
void flushRingNow();
void cleanupOrphanTmp();

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  // >>> Tambahkan sebelum GPS.begin
  GPS.setRxBufferSize(2048);   // butuh core ESP32 baru; kalau nggak ada, abaikan


  GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(BTN_RESET, INPUT_PULLUP);

  // SD init (VSPI explicit)
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  sdReady = SD.begin(SD_CS, SPI, 25000000);
  if (sdReady && !SD.exists("/race")) SD.mkdir("/race");
  if (sdReady) cleanupOrphanTmp();

  // TFT
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(COLOR_BG);
  tft.setTextFont(1); // font terkecil

  // === TAMPILKAN LOG GPS SETUP DI LAYAR ===
  gpsSetupStart();
  gpsAutoSetup10Hz();
  delay(400);
  gpsSetupDone();
  
  // draw Boot/Welcome (sekalian WiFi connect & SNR live)
  drawBootScreenWelcome();

  // Sprites
  sprStatusBar.createSprite(ZONE_STATUS_BAR.w, ZONE_STATUS_BAR.h);
  sprSpeedMain.createSprite(ZONE_SPEED_MAIN.w, ZONE_SPEED_MAIN.h);
  sprTimer.createSprite(ZONE_TIMER.w, ZONE_TIMER.h + 12); // tambahan ruang untuk IP
  sprTable.createSprite(ZONE_TABLE.w, ZONE_TABLE.h);

  resetSystem();
  drawStaticElements();

  // Web
  registerWebRoutes();
  if (wifiConnected) server.begin();
}

// ================== LOOP ==================
void loop() {
  readGPS();
  updateSpeedFilter();
  updateTimerLogic();

  if (millis() - lastUIUpdate > UI_UPDATE_INTERVAL) {
    updateDisplay();
    lastUIUpdate = millis();
  }
  if (loggingActive) {
    static uint32_t lastFlushTick=0;
    if (millis() - lastFlushTick > 3000) { flushRingNow(); lastFlushTick = millis(); }
  }
  if (wifiConnected) server.handleClient();
}

// ================== DISPLAY ==================
void drawStaticElements() {
  tft.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_DIM);
  tft.drawFastHLine(0, ZONE_STATUS_BAR.h, SCREEN_WIDTH, COLOR_DIM);
  tft.drawFastHLine(0, ZONE_PROGRESS.y + ZONE_PROGRESS.h, SCREEN_WIDTH, COLOR_DIM);
}

void updateDisplay() {
  drawStatusBar();
  drawSpeedDisplay();
  drawTimerDisplay();
  drawMilestoneTable();
  if (needsFullRedraw) needsFullRedraw = false;
}

void drawStatusBar() {
  static String lastStatusText = "";
  static int lastSatellites = -1;
  static String lastHdop = "";
  static float lastGpsHz = -1.0f;

  String statusText = String((int)totalDistance) + "m  MAX:" + String(maxSpeed, 1);
  String midText = "ACC:" + (hdop != "-" ? String(hdop) : "-") + " | SAT:" + String(satellites);

  if (statusText == lastStatusText
      && satellites == lastSatellites
      && hdop == lastHdop
      && fabs(gpsHz - lastGpsHz) < 0.05f) return;

  lastStatusText = statusText; lastSatellites = satellites; lastHdop = hdop; lastGpsHz = gpsHz;

  sprStatusBar.fillSprite(COLOR_BG);
  sprStatusBar.setTextFont(1);
  sprStatusBar.setTextColor(COLOR_TEXT, COLOR_BG);
  sprStatusBar.setCursor(2, 3); sprStatusBar.print(statusText);

  int twMid = sprStatusBar.textWidth(midText);
  int midX = (ZONE_STATUS_BAR.w - twMid) / 2;
  sprStatusBar.setTextColor(COLOR_DIM, COLOR_BG);
  sprStatusBar.setCursor(midX, 3);
  sprStatusBar.print(midText);

  drawGPSStatus();
  sprStatusBar.pushSprite(ZONE_STATUS_BAR.x, ZONE_STATUS_BAR.y);
}


void drawGPSStatus() {
  int x = ZONE_GPS_INDICATOR.x - ZONE_STATUS_BAR.x;
  int y = 2;

  float hd = hdop.toFloat();
  bool goodGPS = (satellites >= 6 && hd > 0 && hd <= 2.5 && gpsHz >= 9.0f);
  bool okGPS   = (satellites >= 4 && hd > 0 && hd <= 4.0  && gpsHz >= 4.0f);
  uint16_t gpsColor = goodGPS ? COLOR_SUCCESS : (okGPS ? COLOR_WARNING : COLOR_ERROR);

  char gpsText[20];
  if (gpsHz > 0.05f) snprintf(gpsText, sizeof(gpsText), "%.1fHz", gpsHz);
  else strcpy(gpsText, "--");

  sprStatusBar.setTextColor(gpsColor, COLOR_BG);
  sprStatusBar.setCursor(x, y + 2);
  sprStatusBar.print(gpsText);

  int tw = sprStatusBar.textWidth(gpsText);
  int barX = x + tw + 6;

  int bars = constrain(map(satellites, 0, 12, 0, 4), 0, 4);
  for (int i = 0; i < 4; i++) {
    uint16_t barColor = (i < bars) ? gpsColor : COLOR_DIM;
    int barHeight = 2 + i * 2;
    sprStatusBar.fillRect(barX + i * 2, y + 10 - barHeight, 1, barHeight, barColor);
  }
}


void drawSpeedDisplay() {
  static float lastSpeed = -999.0;
  static SystemState lastState = STATE_NOT_READY;
  if (fabs(filteredSpeed - lastSpeed) < 0.07f && currentState == lastState) return;
  lastSpeed = filteredSpeed; lastState = currentState;
  sprSpeedMain.fillSprite(COLOR_BG);

  sprSpeedMain.setTextFont(7);
  uint16_t speedColor = (currentState == STATE_RUNNING) ? COLOR_PRIMARY :
                       (currentState == STATE_FINISHED) ? COLOR_SUCCESS :
                       (currentState == STATE_FAILED) ? COLOR_ERROR : COLOR_TEXT;
  sprSpeedMain.setTextColor(speedColor, COLOR_BG);

  char speedStr[8];
  sprintf(speedStr, "%5.1f", filteredSpeed);
  int16_t sw = sprSpeedMain.textWidth(speedStr);

  int sx = (ZONE_SPEED_MAIN.w - sw) / 2;
  sprSpeedMain.setCursor(sx, 0);
  sprSpeedMain.print(speedStr);

  if (achieved_0_100) {
    sprSpeedMain.setTextFont(1);
    sprSpeedMain.setTextColor(COLOR_ACCENT, COLOR_BG);
    String label = "0-100: " + String(time_0_100, 3) + "s";
    int labelW = sprSpeedMain.textWidth(label);
    int labelX = ZONE_SPEED_MAIN.w - labelW - 4;
    int labelY = ZONE_SPEED_MAIN.h - 10;
    sprSpeedMain.setCursor(labelX, labelY);
    sprSpeedMain.print(label);
  }

  sprSpeedMain.pushSprite(ZONE_SPEED_MAIN.x, ZONE_SPEED_MAIN.y);
}

void drawTimerDisplay() {
  static double lastTime = -1;
  static SystemState lastState = STATE_NOT_READY;
  double currentTime = 0;
  if (currentState == STATE_RUNNING) currentTime = (millis() - startTime) / 1000.0;
  if (fabs(currentTime - lastTime) < 0.01 && currentState == lastState) return;
  lastTime = currentTime; lastState = currentState;

  sprTimer.fillSprite(COLOR_BG);
  int yBase = 2;

  sprTimer.setTextFont(4);
  sprTimer.setTextColor(COLOR_PRIMARY, COLOR_BG);

  if (currentState == STATE_RUNNING) {
    char timeStr[12];
    if (currentTime < 60) sprintf(timeStr, "%6.3f", currentTime);
    else {
      int mins = (int)(currentTime / 60);
      float secs = currentTime - (mins * 60);
      sprintf(timeStr, "%d:%05.2f", mins, secs);
    }
    char* ptr = timeStr; while (*ptr == ' ') ptr++;
    int tw = sprTimer.textWidth(ptr);
    int tx = (ZONE_TIMER.w - tw) / 2;
    sprTimer.setCursor(tx, yBase); sprTimer.print(ptr);
    yBase += 36;
  } else { yBase += 18; }

  sprTimer.setTextFont(1);
  String statusText = ""; uint16_t statusColor = COLOR_TEXT;
  switch (currentState) {
    case STATE_NOT_READY: statusText = "NOT READY"; statusColor = COLOR_ERROR; break;
    case STATE_READY:     statusText = "READY";     statusColor = COLOR_SUCCESS; break;
    case STATE_RUNNING:   statusText = "RUNNING";   statusColor = COLOR_PRIMARY; break;
    case STATE_FINISHED:  statusText = "FINISHED";  statusColor = COLOR_ACCENT; break;
    case STATE_FAILED:    statusText = "RUN FAILED";statusColor = COLOR_ERROR; break;
  }
  sprTimer.setTextColor(statusColor, COLOR_BG);
  int stw = sprTimer.textWidth(statusText);
  int stx = (ZONE_TIMER.w - stw) / 2;
  sprTimer.setCursor(stx, yBase); sprTimer.print(statusText);

  // IP di bawah READY
  if (currentState == STATE_READY && wifiConnected) {
    String ipLine = wifiIpStr;
    sprTimer.setTextFont(1);
    int maxWidth = ZONE_TIMER.w - 2;
    int ipW = sprTimer.textWidth(ipLine);
    int ipY;
    if (ipW <= maxWidth) {
      int ipX = (ZONE_TIMER.w - ipW) / 2;
      ipY = yBase + 12;
      sprTimer.setCursor(ipX, ipY);
      sprTimer.setTextColor(COLOR_TEXT, COLOR_BG);
      sprTimer.print(ipLine);
    } else {
      ipY = yBase + 22;
      sprTimer.fillRect(0, ipY, ZONE_TIMER.w, 10, COLOR_BG);
      sprTimer.setCursor((ZONE_TIMER.w - ipW) / 2, ipY);
      sprTimer.setTextColor(COLOR_TEXT, COLOR_BG);
      sprTimer.print(ipLine);
    }
    sprTimer.setTextFont(2);
  }

  sprTimer.pushSprite(ZONE_TIMER.x, ZONE_TIMER.y);
}

void drawMilestoneTable() {
  static String lastTableState = "";
  String currentTableState = "";
  for (int i = 0; i < N_MILESTONE; i++)
    currentTableState += String(milestones[i].achieved) + String(milestones[i].time, 3);
  currentTableState += String(achieved_0_100) + String(time_0_100, 3);
  if (currentTableState == lastTableState) return;
  lastTableState = currentTableState;

  sprTable.fillSprite(COLOR_BG);
  sprTable.setTextFont(1);
  sprTable.setTextColor(COLOR_SECONDARY, COLOR_BG);
  int hY = 1;
  sprTable.setCursor(2, hY);       sprTable.print("Milestone");
  sprTable.setCursor(76, hY);      sprTable.print("Time");
  sprTable.setCursor(136, hY);     sprTable.print("Speed");
  sprTable.setCursor(196, hY);     sprTable.print("Distance");
  sprTable.drawFastHLine(0, hY+11, ZONE_TABLE.w, COLOR_DIM);
  int y = hY + 13;

  uint16_t rowColor = achieved_0_100 ? COLOR_SUCCESS : COLOR_DIM;
  sprTable.setTextColor(rowColor, COLOR_BG);
  sprTable.setCursor(2, y);      sprTable.print("0-100kmh");
  sprTable.setCursor(76, y);     sprTable.print(achieved_0_100 ? String(time_0_100, 3) : "-");
  sprTable.setCursor(136, y);    sprTable.print(achieved_0_100 ? "100.0" : "-");
  sprTable.setCursor(196, y);    sprTable.print(achieved_0_100 ? String(dist_0_100, 1) + "m" : "-");
  y += 11;

  for (int i = 0; i < N_MILESTONE; i++) {
    rowColor = milestones[i].achieved ? COLOR_SUCCESS :
              milestones[i].isTarget ? COLOR_PRIMARY : COLOR_DIM;
    sprTable.setTextColor(rowColor, COLOR_BG);
    sprTable.setCursor(2, y);       sprTable.print(milestoneLabels[i]);
    sprTable.setCursor(76, y);      sprTable.print(milestones[i].achieved ? String(milestones[i].time, 3) : "-");
    sprTable.setCursor(136, y);     sprTable.print(milestones[i].achieved ? String(milestones[i].speed, 1) : "-");
    sprTable.setCursor(196, y);     sprTable.print(String(milestoneDist[i], 0) + "m");
    y += 11;
    if (y > ZONE_TABLE.h - 10) break;
  }
  sprTable.pushSprite(ZONE_TABLE.x, ZONE_TABLE.y);
}

// ================== CORE LOGIC ==================
void parseGSA(const String& sentence) {
  String fields[18];
  splitString(sentence, ',', fields, 17);
  if (fields[1].length() > 0) strncpy(gpsMode, fields[1].c_str(), sizeof(gpsMode));
  if (fields[2].length() > 0) strncpy(gpsFixType, fields[2].c_str(), sizeof(gpsFixType));
  for (int i = 0; i < MAX_SAT; i++) satList[i].used = false;
  int used = 0;
  for (int i = 3; i <= 14; i++) {
    if (fields[i].length() > 0) {
      int prn = fields[i].toInt();
      for (int j = 0; j < nSatInView; j++) {
        if (satList[j].prn == prn) satList[j].used = true;
      }
      used++;
    }
  }
  nSatInUse = used;
}

void parseGSV(const String& sentence) {
  String fields[22];
  splitString(sentence, ',', fields, 21);
  int page = fields[2].toInt();
  if (page == 1) nSatInView = 0;
  int fieldIdx = 4;
  while (fieldIdx + 3 < 22 && nSatInView < MAX_SAT) {
    int prn = fields[fieldIdx].toInt();
    int snr = (fields[fieldIdx + 3].length() > 0) ? fields[fieldIdx + 3].toInt() : 0;
    satList[nSatInView].prn = prn;
    satList[nSatInView].snr = snr;
    nSatInView++;
    fieldIdx += 4;
  }
}

void readGPS() {
  static String nmea = "";
  while (GPS.available()) {
    char c = GPS.read();
    if (c == '\n') {
      bool isRMC=false, isGGA=false, isGSA=false, isGSV=false;
      if (nmea.startsWith("$GNRMC") || nmea.startsWith("$GPRMC")) { parseRMC(nmea); isRMC=true; }
      else if (nmea.startsWith("$GNGGA") || nmea.startsWith("$GPGGA")) { parseGGA(nmea); isGGA=true; }
      else if (nmea.startsWith("$GNGSA") || nmea.startsWith("$GPGSA")) { parseGSA(nmea); isGSA=true; }
      else if (nmea.startsWith("$GPVTG") || nmea.startsWith("$GNVTG")) { parseVTG(nmea); }
      else if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GLGSV") || nmea.startsWith("$GAGSV") || nmea.startsWith("$GQGSV")) { parseGSV(nmea); isGSV=true; }

      // === logger tetap ===
      if (currentState == STATE_RUNNING) {
        if (isGGA) appendToRing(1);
        // if (isRMC) appendToRing(0);
      }

      // === hitung Hz dari GGA ===
      if (isGGA) _hzGGAcount++;

      nmea = "";
    } else if (c != '\r') nmea += c;
  }

  // tutup window 1 detik (EMA supaya smooth)
  uint32_t now = millis();
  uint32_t dt = now - _hzWinStart;
  if (dt >= 1000) {
    float inst = (_hzGGAcount * 1000.0f) / (float)dt;     // GGA per detik
    gpsHz = (gpsHz < 0.01f) ? inst : (0.3f * inst + 0.7f * gpsHz);
    _hzGGAcount = 0;
    _hzWinStart = now;
  }
}
void parseVTG(const String& sentence){
  // $..VTG,tracktrue,T,trackmag,M,knots,N,kmh,K*CS
  String f[12];
  splitString(sentence, ',', f, 12);
  // kolom ke-8 (index 7) adalah km/h jika ada
  if (f[7].length() > 0) {
    float v = f[7].toFloat();
    if (v >= 0) speedKmh = v;
  }
}


void parseRMC(const String& sentence) {
  String fields[13];
  splitString(sentence, ',', fields, 13);
  if (fields[1].length() >= 6) { memcpy(rmc_time, fields[1].c_str(), 6); rmc_time[6]=0; }
  if (fields[9].length() >= 6) { memcpy(rmc_date, fields[9].c_str(), 6); rmc_date[6]=0; }

  if (fields[3].length() > 0 && fields[5].length() > 0) {
    float lat = convertToDecimal(fields[3].toFloat(), fields[4]);
    float lon = convertToDecimal(fields[5].toFloat(), fields[6]);
    lastLat = lat; lastLon = lon;           // >>> JANGAN update totalDistance di sini
  }
  if (fields[7].length() > 0)               // speed (knots) → km/h @10 Hz
    speedKmh = fields[7].toFloat() * 1.852f;
}


void parseGGA(const String& sentence) {
  String fields[16];
  splitString(sentence, ',', fields, 15);

  if (fields[2].length() > 0 && fields[4].length() > 0) {
    float lat = convertToDecimal(fields[2].toFloat(), fields[3]);
    float lon = convertToDecimal(fields[4].toFloat(), fields[5]);

    // ==== TAMBAHAN: jarak dihitung di GGA @10 Hz ====
    if (!firstFix) {
      float delta = haversine(lastValidLat, lastValidLon, lat, lon);
      if (delta > 0.5f && delta < 100.0f && filteredSpeed >= MIN_COUNTING_SPEED) {
        totalDistance += delta;
        lastValidLat = lat; lastValidLon = lon;
      }
    } else {
      lastValidLat = lat; lastValidLon = lon;
      firstFix = false;
    }
    // ===============================================

    snprintf(gpsLat, sizeof(gpsLat), "%.5f %s", fabs(lat), (lat < 0 ? "S" : "N"));
    snprintf(gpsLon, sizeof(gpsLon), "%.5f %s", fabs(lon), (lon < 0 ? "W" : "E"));
    lastLat = lat; lastLon = lon;
  }

  if (fields[9].length() > 0)
    snprintf(gpsAlt, sizeof(gpsAlt), "%s %s", fields[9].c_str(), fields[10].c_str());
  if (fields[7].length() > 0)
    nSatInUse = satellites = fields[7].toInt();
  if (fields[8].length() > 0)
    hdop = fields[8];
  if (fields[6].length() > 0)
    gpsFixQuality = fields[6].toInt();
}


void updateTimerLogic() {
  float hdopVal = hdop.toFloat();
  switch (currentState) {
    case STATE_NOT_READY:
      if (hdopVal > 0 && hdopVal <= GPS_ACCURACY_THRESHOLD && satellites >= 6)
        currentState = STATE_READY;
      break;
    case STATE_READY:
      if (hdopVal <= 0 || hdopVal > GPS_ACCURACY_THRESHOLD || satellites < 6) {
        currentState = STATE_NOT_READY;
        break;
      }
      if (filteredSpeed > MIN_START_SPEED) startRun();
      break;
    case STATE_RUNNING:
      updateMilestones();
      if (filteredSpeed < STOP_SPEED_THRESHOLD || allMilestonesAchieved())
        finishRun();
      break;
    case STATE_FINISHED:
      // lock hasil di FINISHED
      break;
    case STATE_FAILED:
      if (millis() - failedTime >= 1000) {
        currentState = STATE_READY;
        needsFullRedraw = true;
      }
      break;
  }
}

void startRun() {
  currentState = STATE_RUNNING;
  startTime = millis();
  startDistance = totalDistance;
  for (int i = 0; i < N_MILESTONE; i++) {
    milestones[i].achieved = false;
    milestones[i].time = 0;
    milestones[i].speed = 0;
    milestones[i].isTarget = (i == 0);
  }
  achieved_0_100 = false; time_0_100 = 0; dist_0_100 = 0;

  // mulai file tmp (reset ring)
  openRunTempFile();
}

void updateMilestones() {
  float currentDistance = totalDistance - startDistance;
  float elapsedTime = (millis() - startTime) / 1000.0f;
  if (!achieved_0_100 && filteredSpeed >= 100.0f) {
    achieved_0_100 = true; time_0_100 = elapsedTime; dist_0_100 = currentDistance;
  }
  for (int i = 0; i < N_MILESTONE; i++) {
    if (!milestones[i].achieved && currentDistance >= milestoneDist[i]) {
      milestones[i].achieved = true;
      milestones[i].time = elapsedTime;
      milestones[i].speed = filteredSpeed;
      milestones[i].isTarget = false;
      if (i + 1 < N_MILESTONE) milestones[i + 1].isTarget = true;
    }
  }
}

void finishRun() {
  if (!milestones[0].achieved) {
    // === FAILED: zero-kan & buang tmp ===
    currentState = STATE_FAILED;
    failedTime = millis();
    needsFullRedraw = true;

    for (int i=0;i<N_MILESTONE;i++){ milestones[i].achieved=false; milestones[i].time=0; milestones[i].speed=0; milestones[i].isTarget=false; }
    achieved_0_100=false; time_0_100=0; dist_0_100=0;
    maxSpeed=0.0f; totalDistance=0.0f;

    finalizeRunFile(false);
    return;
  }
  currentState = STATE_FINISHED;

  // Sukses: rename tmp -> csv
  finalizeRunFile(true);
}

bool allMilestonesAchieved() {
  for (int i = 0; i < N_MILESTONE; i++) if (!milestones[i].achieved) return false;
  return true;
}

void resetSystem() {
  currentState = STATE_NOT_READY;
  startTime = 0; startDistance = 0; maxSpeed = 0.0f;
  for (int i=0;i<N_MILESTONE;i++){ milestones[i].achieved=false; milestones[i].time=0; milestones[i].speed=0; milestones[i].isTarget=false; }
  achieved_0_100=false; time_0_100=0; dist_0_100=0;
  maxSpeed=0; totalDistance=0; firstFix=true;

  tft.fillScreen(COLOR_BG); drawStaticElements();
}

// ================== FILTER ==================
void updateSpeedFilter() {
  float measurement = speedKmh;
  float innovation = fabs(measurement - kalmanX);
  if (innovation < 0.3f)      { kalmanQ = 0.005f; kalmanR = 0.20f; }
  else if (innovation < 1.5f) { kalmanQ = 0.04f;  kalmanR = 0.90f; }
  else if (innovation < 4.0f) { kalmanQ = 0.15f;  kalmanR = 2.50f; }
  else                        { kalmanQ = 0.90f;  kalmanR = 6.0f; }
  kalmanP += kalmanQ;
  kalmanK = kalmanP / (kalmanP + kalmanR);
  kalmanX = kalmanX + kalmanK * (measurement - kalmanX);
  kalmanP = (1 - kalmanK) * kalmanP;

  if (measurement < STOP_SPEED_THRESHOLD) {
    stopCounter++; if (stopCounter >= STOP_CONFIRM_SAMPLE) { kalmanX = 0; kalmanP = 1.0f; }
  } else stopCounter = 0;

  filteredSpeed = kalmanX;
  if (filteredSpeed > maxSpeed) maxSpeed = filteredSpeed;
}

// ================== UTILS ==================
void splitString(const String& str, char delimiter, String output[], int maxParts) {
  int start = 0, index = 0;
  for (int i = 0; i < (int)str.length() && index < maxParts; i++) {
    if (str[i] == delimiter) { output[index++] = str.substring(start, i); start = i + 1; }
  }
  if (index < maxParts) output[index] = str.substring(start);
}

float convertToDecimal(float raw, const String& direction) {
  int degrees = (int)(raw / 100);
  float minutes = raw - (degrees * 100);
  float decimal = degrees + minutes / 60.0f;
  if (direction == "S" || direction == "W") decimal = -decimal;
  return decimal;
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// ================== BOOT SCREEN (Wi-Fi non-blocking + SNR) ==================
int countRaceFiles() {
  if (!sdReady) return 0;
  int cnt = 0;
  File dir = SD.open("/race");
  if (!dir) return 0;
  File f;
  while ((f = dir.openNextFile())) {
    if (!f.isDirectory()) {
      String n = f.name();
      if (n.endsWith(".csv")) cnt++;
    }
    f.close();
  }
  dir.close();
  return cnt;
}

void drawBootScreenWelcome() {
  // Modern layout constants
  const int MARGIN = 12;
  const int HEADER_H = 24;
  const int SECTION_GAP = 16;
  const int LINE_H = 14;
  const int LABEL_W = 65;
  const int VALUE_X = MARGIN + LABEL_W + 8;
  
  // Color scheme
  const uint16_t BG_COLOR = 0x0841;      // Dark blue
  const uint16_t HEADER_COLOR = 0x07FF;  // Cyan
  const uint16_t LABEL_COLOR = 0x8410;   // Gray
  const uint16_t VALUE_COLOR = TFT_WHITE;
  const uint16_t SUCCESS_COLOR = 0x07E0; // Green
  const uint16_t WARNING_COLOR = 0xFD20; // Orange
  const uint16_t ERROR_COLOR = 0xF800;   // Red
  const uint16_t ACCENT_COLOR = 0xFFE0;  // Yellow
  
  // SNR bar settings
  const int SNR_BASE_Y = 180;
  const int SNR_MAX_H = 50;
  const int BAR_W = 8;
  const int BAR_GAP = 2;

  tft.fillScreen(BG_COLOR);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(1); // Use smallest font
  
  // Header
  tft.setTextColor(HEADER_COLOR, BG_COLOR);
  tft.setTextSize(2);
  tft.drawString("GPS TRACKER", MARGIN, 8);
  tft.setTextSize(1);
  tft.setTextColor(LABEL_COLOR, BG_COLOR);
  tft.drawString("System Status", MARGIN, 26);
  
  // Draw separator line
  tft.drawFastHLine(MARGIN, HEADER_H + 16, SCREEN_WIDTH - 2*MARGIN, LABEL_COLOR);
  
  int y = HEADER_H + 24;
  
  // Helper function for drawing status items
  auto drawStatusItem = [&](const char* label, const char* value, uint16_t valueColor, int yPos) {
    tft.setTextColor(LABEL_COLOR, BG_COLOR);
    tft.drawString(label, MARGIN, yPos);
    tft.setTextColor(valueColor, BG_COLOR);
    tft.drawString(value, VALUE_X, yPos);
  };
  
  // Network section header
  tft.setTextColor(HEADER_COLOR, BG_COLOR);
  tft.drawString("NETWORK", MARGIN, y);
  y += LINE_H + 4;
  
  // GPS section (will be updated in loop)
  int gps_section_y = y + LINE_H + SECTION_GAP;
  
  // Satellite section
  int sat_section_y = gps_section_y + 6*LINE_H + SECTION_GAP;
  
  // Files section  
  int files_section_y = sat_section_y + 4*LINE_H + SECTION_GAP;

  // Initialize last values for change detection
  last_wifi_ip = ""; last_wifi_conn = !wifiConnected;
  last_sd_ready = !sdReady; last_race_count = -1;
  last_hdop = -99; last_nSatInUse = -1; last_nSatInView = -1; last_gpsFixQuality = -1;
  strcpy(last_gpsMode, ""); strcpy(last_gpsFixType, ""); 
  strcpy(last_gpsLat, ""); strcpy(last_gpsLon, ""); strcpy(last_gpsAlt, "");

  // WiFi setup (non-blocking)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  
  int wifiPhase = 0;
  String chosenSsid = "", chosenPass = "";
  WiFi.scanNetworks(true, true);
  unsigned long wifiStartMs = millis();

  // Create SNR sprite with modern styling
  TFT_eSprite sprBar(&tft);
  int maxBars = (nSatInView < MAX_SAT) ? nSatInView : MAX_SAT;
  int spriteW = maxBars > 0 ? (maxBars * BAR_W + (maxBars-1) * BAR_GAP + 40) : 100;
  sprBar.createSprite(spriteW, SNR_MAX_H + 30);

  unsigned long lockMillis = 0;
  bool locked = false;
  int raceCount = countRaceFiles();

  while (1) {
    // WiFi state machine (same logic as original)
    if (wifiPhase == 0) {
      wifiPhase = 1;
    } else if (wifiPhase == 1) {
      int res = WiFi.scanComplete();
      if (res >= 0) {
        for (int i = 0; i < res; i++) {
          String ssid = WiFi.SSID(i);
          for (int k = 0; k < N_KNOWN; k++) {
            if (ssid == knownList[k].ssid) {
              chosenSsid = ssid; chosenPass = knownList[k].pass; 
              break;
            }
          }
          if (chosenSsid.length()) break;
        }
        if (chosenSsid.length()) {
          WiFi.begin(chosenSsid.c_str(), chosenPass.c_str());
          wifiPhase = 2;
          wifiStartMs = millis();
        } else {
          wifiPhase = 3;
        }
      } else if (res == WIFI_SCAN_FAILED && millis() - wifiStartMs > 3000) {
        WiFi.scanDelete();
        WiFi.scanNetworks(true, true);
        wifiStartMs = millis();
      }
    } else if (wifiPhase == 2) {
      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        wifiIpStr = WiFi.localIP().toString();
        wifiPhase = 3;
      } else if (millis() - wifiStartMs > 6000) {
        wifiConnected = false;
        wifiIpStr = "--";
        wifiPhase = 3;
      }
    }

    // Update WiFi status with modern styling
    if (wifiConnected != last_wifi_conn || wifiIpStr != last_wifi_ip) {
      tft.fillRect(MARGIN, y, SCREEN_WIDTH - 2*MARGIN, LINE_H, BG_COLOR);
      
      String status;
      uint16_t statusColor;
      if (wifiConnected) {
        status = "Connected • " + wifiIpStr;
        statusColor = SUCCESS_COLOR;
      } else if (wifiPhase < 3) {
        status = "Connecting...";
        statusColor = WARNING_COLOR;
      } else {
        status = "Disconnected";
        statusColor = ERROR_COLOR;
      }
      
      tft.setTextColor(LABEL_COLOR, BG_COLOR);
      tft.drawString("WiFi:", MARGIN, y);
      tft.setTextColor(statusColor, BG_COLOR);
      tft.drawString(status, VALUE_X, y);
      
      last_wifi_conn = wifiConnected;
      last_wifi_ip = wifiIpStr;
    }

    // Draw section headers
    tft.setTextColor(HEADER_COLOR, BG_COLOR);
    tft.drawString("GPS DATA", MARGIN, gps_section_y - 2);
    tft.drawString("SATELLITES", MARGIN, sat_section_y - 2);
    tft.drawString("STORAGE", MARGIN, files_section_y - 2);

    // GPS Data Section
    int gps_y = gps_section_y + 12;
    
    // HDOP
    float hdopVal = hdop.toFloat();
    if (fabs(hdopVal - last_hdop) > 0.01f) {
      tft.fillRect(VALUE_X, gps_y, 100, LINE_H, BG_COLOR);
      drawStatusItem("HDOP:", hdop.c_str(), VALUE_COLOR, gps_y);
      last_hdop = hdopVal;
    }
    gps_y += LINE_H;

    // Position
    if (strcmp(gpsLat, last_gpsLat) != 0) {
      tft.fillRect(VALUE_X, gps_y, 140, LINE_H, BG_COLOR);
      drawStatusItem("Lat:", gpsLat, VALUE_COLOR, gps_y);
      strcpy(last_gpsLat, gpsLat);
    }
    gps_y += LINE_H;

    if (strcmp(gpsLon, last_gpsLon) != 0) {
      tft.fillRect(VALUE_X, gps_y, 140, LINE_H, BG_COLOR);
      drawStatusItem("Lon:", gpsLon, VALUE_COLOR, gps_y);
      strcpy(last_gpsLon, gpsLon);
    }
    gps_y += LINE_H;

    if (strcmp(gpsAlt, last_gpsAlt) != 0) {
      tft.fillRect(VALUE_X, gps_y, 120, LINE_H, BG_COLOR);
      drawStatusItem("Alt:", gpsAlt, VALUE_COLOR, gps_y);
      strcpy(last_gpsAlt, gpsAlt);
    }
    gps_y += LINE_H;

    // Fix info
    if (gpsFixQuality != last_gpsFixQuality) {
      tft.fillRect(VALUE_X, gps_y, 80, LINE_H, BG_COLOR);
      drawStatusItem("Quality:", String(gpsFixQuality).c_str(), VALUE_COLOR, gps_y);
      last_gpsFixQuality = gpsFixQuality;
    }
    gps_y += LINE_H;

    if (strcmp(gpsFixType, last_gpsFixType) != 0) {
      tft.fillRect(VALUE_X, gps_y, 100, LINE_H, BG_COLOR);
      drawStatusItem("Fix:", gpsFixType, VALUE_COLOR, gps_y);
      strcpy(last_gpsFixType, gpsFixType);
    }

    // Satellites Section  
    int sat_y = sat_section_y + 12;
    
    if (nSatInUse != last_nSatInUse) {
      tft.fillRect(VALUE_X, sat_y, 60, LINE_H, BG_COLOR);
      uint16_t color = nSatInUse >= 4 ? SUCCESS_COLOR : (nSatInUse > 0 ? WARNING_COLOR : ERROR_COLOR);
      tft.setTextColor(LABEL_COLOR, BG_COLOR);
      tft.drawString("Used:", MARGIN, sat_y);
      tft.setTextColor(color, BG_COLOR);
      tft.drawString(String(nSatInUse), VALUE_X, sat_y);
      last_nSatInUse = nSatInUse;
    }
    sat_y += LINE_H;

    if (nSatInView != last_nSatInView) {
      tft.fillRect(VALUE_X, sat_y, 60, LINE_H, BG_COLOR);
      drawStatusItem("Visible:", String(nSatInView).c_str(), VALUE_COLOR, sat_y);
      last_nSatInView = nSatInView;
    }
    sat_y += LINE_H;

    // Storage Section
    int storage_y = files_section_y + 12;
    
    if (sdReady != last_sd_ready) {
      tft.fillRect(VALUE_X, storage_y, 100, LINE_H, BG_COLOR);
      uint16_t sdColor = sdReady ? SUCCESS_COLOR : ERROR_COLOR;
      String sdStatus = sdReady ? "Ready" : "Error";
      drawStatusItem("SD Card:", sdStatus.c_str(), sdColor, storage_y);
      last_sd_ready = sdReady;
      raceCount = countRaceFiles();
      last_race_count = -1;
    }
    storage_y += LINE_H;

    if (raceCount != last_race_count) {
      tft.fillRect(VALUE_X, storage_y, 60, LINE_H, BG_COLOR);
      drawStatusItem("Races:", String(raceCount).c_str(), ACCENT_COLOR, storage_y);
      last_race_count = raceCount;
    }

    // Modern SNR visualization
    maxBars = (nSatInView < MAX_SAT) ? nSatInView : MAX_SAT;
    if (maxBars > 0) {
      sprBar.fillSprite(BG_COLOR);
      
      for (int i = 0; i < maxBars; i++) {
        int snr = satList[i].snr;
        int prn = satList[i].prn;
        bool used = satList[i].used;
        
        // Modern color scheme for SNR bars
        uint16_t barColor;
        if (!used) {
          barColor = 0x2945; // Dark gray
        } else if (snr >= 40) {
          barColor = SUCCESS_COLOR;
        } else if (snr >= 30) {
          barColor = 0x07FF; // Cyan
        } else if (snr >= 20) {
          barColor = WARNING_COLOR;
        } else {
          barColor = ERROR_COLOR;
        }
        
        int barHeight = map(snr, 0, 50, 4, SNR_MAX_H);
        barHeight = constrain(barHeight, 4, SNR_MAX_H);
        
        int x = i * (BAR_W + BAR_GAP);
        int y = SNR_MAX_H - barHeight;
        
        // Draw bar with rounded top effect
        sprBar.fillRect(x, y, BAR_W, barHeight, barColor);
        if (barHeight > 2) {
          sprBar.drawFastHLine(x, y, BAR_W, TFT_WHITE); // Highlight top
        }
        
        // PRN number below bar
        sprBar.setTextDatum(MC_DATUM);
        sprBar.setTextColor(LABEL_COLOR, BG_COLOR);
        sprBar.drawString(String(prn), x + BAR_W/2, SNR_MAX_H + 8);
        
        // SNR value above bar (only if space)
        if (y > 12 && used) {
          sprBar.setTextColor(VALUE_COLOR, BG_COLOR);
          sprBar.drawString(String(snr), x + BAR_W/2, y - 6);
        }
      }
      
      // Center the sprite
      int totalWidth = maxBars * BAR_W + (maxBars-1) * BAR_GAP;
      int startX = (SCREEN_WIDTH - totalWidth) / 2;
      sprBar.pushSprite(startX, SNR_BASE_Y);
    }

    readGPS();

    // GPS lock detection with modern feedback
    bool gpsLocked = (hdopVal > 0 && hdopVal <= 5.0 && nSatInUse >= 4);
    if (gpsLocked) {
      if (!locked) {
        lockMillis = millis();
        locked = true;
        
        // Visual feedback for GPS lock
        tft.fillRect(SCREEN_WIDTH - 60, 8, 50, 12, SUCCESS_COLOR);
        tft.setTextColor(BG_COLOR, SUCCESS_COLOR);
        tft.drawString("GPS OK", SCREEN_WIDTH - 55, 9);
      }
      if (millis() - lockMillis > 3000) break; // Reduced wait time
    } else {
      locked = false;
      // Show GPS status
      tft.fillRect(SCREEN_WIDTH - 60, 8, 50, 12, BG_COLOR);
      tft.setTextColor(WARNING_COLOR, BG_COLOR);
      tft.drawString("GPS..", SCREEN_WIDTH - 55, 9);
    }

    delay(100);
    yield();
  }

  sprBar.deleteSprite();

  // Final WiFi status update
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiIpStr = WiFi.localIP().toString();
  } else {
    wifiConnected = false;
    wifiIpStr = "--";
  }
  
  // Boot complete indicator
  tft.fillRect(0, SCREEN_HEIGHT - 20, SCREEN_WIDTH, 20, SUCCESS_COLOR);
  tft.setTextColor(BG_COLOR, SUCCESS_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("SYSTEM READY", SCREEN_WIDTH/2, SCREEN_HEIGHT - 10);
  delay(1000);
}

// ================== WIFI & WEB ==================
static const char* CDN = "https://wicak.id/cdn";
static const char* JS_BASE = "https://wicak.id/js";

String htmlHeader(const String& title) {
  String h;
  h += "<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title>";

  // Styles di domain kamu
  h += "<link rel='preconnect' href='" + String(CDN) + "'>";
  h += "<link rel='stylesheet' href='https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css'>";
  h += "<link rel='stylesheet' href='https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css'>";
  h += "<link rel='stylesheet' href='" + String(CDN) + "/wicak.css'>";           // racing theme

  h += "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'>";
  h += "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>";

  // Chart.js + zoom plugin dari domain kamu
  h += "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js'></script>";
  h += "<script src='https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2'></script>";

  // Utils kecil (debounce/format), kamu host sendiri
  // h += "<script src='" + String(JS_BASE) + "/race-detail.v1.js' defer></script>";

  h += "</head><body class='bg-racing text-light'><div class='container-fluid py-3'>";
  h += "<header class='d-flex align-items-center justify-content-between mb-3 racing-header'>"
       "<h3 class='mb-0 title-glow'>" + title + "</h3>"
     "</header>";
  return h;
}
String htmlFooter(){ return "</div></body></html>"; }

bool validRacePath(const String& param, String& fullPath) {
  if (param.indexOf('/')>=0 || param.indexOf('\\')>=0) return false;
  fullPath = "/race/" + param;
  if (!fullPath.endsWith(".csv")) fullPath += ".csv";
  return SD.exists(fullPath);
}

// --- Helper: ambil top speed (dari kolom filtered_speed_kmh) ---
// ---------- TOP SPEED (robust + limit 1MB) ----------
float calcTopSpeedKmh(const String& fullPath) {
  File f = SD.open(fullPath, FILE_READ);
  if (!f) return 0.0f;

  // Skip header secara eksplisit (cari baris yang mengandung "filtered_speed_kmh")
  String line;
  bool headerSkipped = false;
  uint32_t processed = 0;

  while (f.available() && !headerSkipped) {
    line = f.readStringUntil('\n');        // tanpa '\n', mungkin masih ada '\r'
    processed += line.length() + 1;
    if (line.indexOf(F("filtered_speed_kmh")) >= 0) {
      headerSkipped = true;
      break;
    }
    if (processed > 1024UL * 1024UL) {     // guard 1MB
      f.close();
      return 0.0f;
    }
  }

  float vmax = 0.0f;

  while (f.available()) {
    line = f.readStringUntil('\n');
    processed += line.length() + 1;
    if (line.length() < 5) continue;

    // ambil kolom ke-5 (index 4) = filtered_speed_kmh
    int last = 0, idx = -1, col = 0;
    while (true) {
      idx = line.indexOf(',', last);
      String tok = (idx >= 0) ? line.substring(last, idx) : line.substring(last);
      if (col == 4) { // filtered_speed_kmh
        float vf = tok.toFloat();
        if (vf > vmax && vf < 2000.0f) vmax = vf;
        break;
      }
      if (idx < 0) break;
      last = idx + 1;
      col++;
      if (col > 4) break;
    }

    if (processed > 1024UL * 1024UL) break; // stop di ~1MB
  }

  f.close();
  return vmax;
}



// ---------- INDEX TABLE (hide .csv & /race/, fix links, show top speed) ----------
void handleIndex() {
  String out = htmlHeader("Race Results");

  // Status
  out += "<div class='mb-2 d-flex flex-wrap align-items-center gap-2'>"
         "<span class='badge " + String(wifiConnected ? "bg-success" : "bg-secondary") + "'>WiFi</span>"
         "<span class='mono'>" + (wifiConnected ? wifiIpStr : "--") + "</span>"
         "<span class='badge " + String(sdReady ? "bg-info" : "bg-danger") + "'>SD</span>"
         + (sdReady ? "<span>Mounted</span>" : "<span>Not mounted</span>") +
         "</div>";

  // Tombol reset RAM + bulk delete
  out += "<div class='mb-3 d-flex flex-wrap align-items-center gap-2'>"
         "<button id='btnReset' class='btn btn-warning btn-sm'><i class='bi bi-arrow-counterclockwise me-1'></i>Reset</button>"
         "<button id='btnBulkDel' class='btn btn-danger btn-sm'><i class='bi bi-trash3 me-1'></i>Hapus Semua File</button>"
         "</div>";

  // Toast
  out += "<div id='toast' class='alert alert-success py-2 px-3 d-none' role='alert'>Aksi berhasil.</div>";
  out += "<div id='toaste' class='alert alert-danger py-2 px-3 d-none' role='alert'>Aksi gagal.</div>";

  // Table (FULL WIDTH + responsive)
  out += "<div class='card bg-secondary-subtle text-dark'><div class='card-body'>";
  out += "<div class='table-responsive'><table class='table table-sm table-striped table-hover align-middle w-100'>";
  out += "<thead><tr>"
         "<th class='text-center'>No.</th>"
         "<th>Race</th>"
         "<th class='text-center'>Action</th>"
         "</tr></thead><tbody>";

  if (sdReady) {
    File dir = SD.open("/race");
    if (dir) {
      File f;
      int idx = 1;
      while ((f = dir.openNextFile())) {
        if (!f.isDirectory()) {
          String path = f.name();             // contoh: "/race/race_250812_181810.csv"
          if (path.endsWith(".csv")) {

            // --- siapkan display name (tanpa folder & tanpa ".csv") ---
            String base = path;
            int slash = base.lastIndexOf('/');
            if (slash >= 0) base = base.substring(slash + 1); // "race_250812_181810.csv"
            String disp = base;
            if (disp.endsWith(".csv")) disp.remove(disp.length() - 4); // "race_250812_181810"

            // --- param untuk URL (validRacePath butuh nama file tanpa slash) ---
            String fileParam = base; // tetap ada ".csv"
            // jaga-jaga: kalau user punya file tanpa .csv
            if (!fileParam.endsWith(".csv")) fileParam += ".csv";

            out += "<tr>"
                   "<td class='text-center'>" + String(idx++) + "</td>"
                   "<td class='mono file-col'>" + disp + "</td>"
                   "<td class='text-center nowrap'>"
                     "<div class='btn-group btn-group-sm flex-nowrap' role='group'>"
                       "<a class='btn btn-sm btn-primary' href='/race?file=" + fileParam + "' title='Detail'><i class='bi bi-eye'></i></a>"
                       "<a class='btn btn-sm btn-outline-secondary' href='/dl?file=" + fileParam + "' title='Download'><i class='bi bi-download'></i></a>"
                       "<button class='btn btn-sm btn-danger' title='Hapus' onclick='delFile(\"" + fileParam + "\")'><i class='bi bi-trash3'></i></button>"
                     "</div>"
                   "</td></tr>";
          }
        }
        f.close();
      }
      dir.close();
    } else {
      out += "<tr><td colspan='4'>/race tidak dapat dibuka</td></tr>";
    }
  } else {
    out += "<tr><td colspan='4'>SD tidak tersedia</td></tr>";
  }

  out += "</tbody></table></div></div></div>";

  // JS
  out += "<script>"
         "const toast=document.getElementById('toast');"
         "const toaste=document.getElementById('toaste');"
         "function show(el,msg){ if(msg) el.textContent=msg; el.classList.remove('d-none'); setTimeout(()=>el.classList.add('d-none'),1500);} "
         "document.getElementById('btnReset').onclick=async(e)=>{"
           "const btn=e.currentTarget; if(btn.disabled) return; btn.disabled=true;"
           "try{ const r=await fetch('/reset',{method:'POST'}); if(r.ok){show(toast,'Reset RAM berhasil'); setTimeout(()=>location.reload(),800);} else{show(toaste);} }"
           "catch(_){show(toaste);} finally{ setTimeout(()=>btn.disabled=false,1000);} "
         "};"
         "document.getElementById('btnBulkDel').onclick=async(e)=>{"
           "if(!confirm('Hapus SEMUA file .csv di /race?\\nTindakan ini tidak bisa dibatalkan.')) return;"
           "const btn=e.currentTarget; if(btn.disabled) return; btn.disabled=true;"
           "try{ const r=await fetch('/delete_all',{method:'POST'});"
           " if(r.ok){ show(toast,'Semua file dihapus'); setTimeout(()=>location.reload(),800);} else {show(toaste);} }"
           "catch(_){ show(toaste); } finally{ setTimeout(()=>btn.disabled=false,1000);} "
         "};"
         "async function delFile(f){"
           "if(!confirm('Hapus file '+f+' ?')) return;"
           "try{ const fd=new FormData(); fd.append('file',f); "
           "const r=await fetch('/delete',{method:'POST',body:fd});"
           "if(r.ok){alert('File dihapus'); location.reload();} else {alert('Gagal hapus');}"
           "}catch(_){alert('Error');}"
         "}"
         "</script>";

  out += htmlFooter();
  server.send(200, "text/html", out);
}
// HANDLER 1: Race Detail Page
void handleRaceDetail() {
  String file = server.arg("file");
  String base = file; int slash = base.lastIndexOf('/'); if (slash >= 0) base = base.substring(slash + 1);
  String fileParam = base; if (!fileParam.endsWith(".csv")) fileParam += ".csv";
  String disp = base; if (disp.endsWith(".csv")) disp.remove(disp.length() - 4);

  String out = htmlHeader("Race Detail – " + disp);

  out += "<div class='d-flex flex-wrap gap-2 mb-3'>"
           "<a class='btn btn-outline-light btn-ghost' href='/'><i class='bi bi-arrow-left me-1'></i>Back</a>"
           "<a class='btn btn-success btn-ghost' href='/dl?file="+fileParam+"'><i class='bi bi-download me-1'></i>Download CSV</a>"
           "<button class='btn btn-warning btn-ghost' onclick='renameCurrent()'><i class=\"bi bi-pencil-square me-1\"></i>Rename</button>"
           "<button class='btn btn-info btn-ghost' onclick='exportPDF()'><i class='bi bi-filetype-pdf me-1'></i>Export PDF</button>"
           "<button class='btn btn-danger btn-ghost' onclick='delCurrent()'><i class='bi bi-trash3 me-1'></i>Hapus</button>"
         "</div>";

  // Summary cards
  out += "<div class='row g-3 mb-3'>"
           "<div class='col-12 col-lg-4'>"
             "<div class='card card-racing h-100'><div class='card-body py-3'>"
               "<div class='d-grid gap-2 small mono'>"
                 "<div class='d-flex justify-content-between'><span class='text-secondary'>Max speed</span><b id='sumMax'>-</b><span>km/h</span></div>"
                 "<div class='d-flex justify-content-between'><span class='text-secondary'>0–60</span><b id='sum0060'>-</b></div>"
                 "<div class='d-flex justify-content-between'><span class='text-secondary'>0–100</span><b id='sum00100'>-</b></div>"
               "</div>"
             "</div></div>"
           "</div>"
           "<div class='col-12 col-lg-8'>"
             "<div class='card card-racing h-100'><div class='card-body py-0'>"
               "<div class='table-responsive small mb-0'>"
                 "<table class='table table-sm table-dark table-striped mb-0 align-middle'>"
                   "<thead><tr><th>Milestone</th><th>Target</th><th>Time</th><th>Speed (km/h)</th></tr></thead>"
                   "<tbody id='sumMilestones'></tbody>"
                 "</table>"
               "</div>"
             "</div></div>"
           "</div>"
         "</div>";

  // Charts + Controls
  out += "<div class='d-flex flex-wrap align-items-center gap-2 mb-2'>"
           "<div class='btn-group btn-group-sm' role='group'>"
             "<button id='btnResetZoom' class='btn btn-outline-light'><i class='bi bi-zoom-out'></i> Reset Zoom</button>"
             "<button id='btnToggleSmooth' class='btn btn-outline-light'><i class='bi bi-graph-up'></i> Smooth</button>"
           "</div>"
           "<span class='text-secondary small'>Scroll untuk zoom (X), drag untuk pan</span>"
         "</div>";

  out += "<div id='chartsWrap' class='row g-3 d-none'>"
           "<div class='col-12 col-lg-6'><div class='card card-racing'><div class='card-body'>"
             "<h6 class='mb-1'>Distance vs Time</h6>"
             "<div class='chart-box'><canvas id='distTime'></canvas></div>"
           "</div></div></div>"
           "<div class='col-12 col-lg-6'><div class='card card-racing'><div class='card-body'>"
             "<h6 class='mb-1'>Speed vs Time</h6>"
             "<div class='chart-box'><canvas id='speedTime'></canvas></div>"
           "</div></div></div>"
           "<div class='col-12 col-lg-6'><div class='card card-racing'><div class='card-body'>"
             "<h6 class='mb-1'>Speed vs Distance</h6>"
             "<div class='chart-box'><canvas id='speedDist'></canvas></div>"
           "</div></div></div>"
           "<div class='col-12 col-lg-6'><div class='card card-racing'><div class='card-body'>"
             "<h6 class='mb-1'>HDOP vs Time</h6>"
             "<div class='chart-box'><canvas id='hdopTime'></canvas></div>"
           "</div></div></div>"
         "</div>";

  // Track map (kanvas ringan warna-coded by speed)
  out += "<div class='card card-racing mt-3'><div class='card-body'>"
           "<div class='d-flex justify-content-between align-items-center mb-2'>"
             "<h6 class='mb-0'>Track Map (Lat/Lon)</h6>"
             "<div class='btn-group btn-group-sm'>"
               "<button id='btnFitMap' class='btn btn-outline-light'><i class='bi bi-aspect-ratio'></i> Fit</button>"
               "<button id='btnExportGPX' class='btn btn-outline-light'><i class='bi bi-download'></i> GPX</button>"
             "</div>"
           "</div>"
           "<div class='track-map-wrap'><canvas id='trackMap' width='1200' height='600'></canvas></div>"
           "<div class='small text-secondary mt-1'>Warna lintasan diwarnai oleh speed (km/h).</div>"
         "</div></div>";

  // Raw table
  out += "<hr><h5 class='text-center'>Raw Data</h5>"
         "<div id='loadingStatus' class='text-center text-warning mb-2'>Loading data...</div>"
         "<div class='table-responsive'>"
           "<table class='table table-sm table-dark table-striped w-100'>"
             "<thead><tr class='text-center'>"
               "<th style='min-width:34px'>#</th><th>t(s)</th><th>dist(m)</th><th>speed(km/h)</th><th>hdop</th>"
             "</tr></thead>"
             "<tbody id='tbody'></tbody>"
           "</table>"
         "</div>";

  // Pass config ke JS v2
  out += "<script>window.RACE_CFG={file:'" + fileParam + "', title:'" + disp + "'};</script>";

  // JS utama v2 (kamu host di domain kamu)
  out += "<script src='" + String(JS_BASE) + "/race-detail.v2.1(1).js' defer></script>";

  out += htmlFooter();
  server.send(200, "text/html", out);
}


// HANDLER 2: API Race Page (Fixed)
void handleApiRacePage() {
  String file = server.arg("file");
  String full;
  if (!validRacePath(file, full)) {
    server.send(404, "application/json", "{\"err\":\"not_found\"}");
    return;
  }

  // Parse & clamp params
  long fromL  = server.hasArg("from")  ? server.arg("from").toInt()  : 0;
  long takeL  = server.hasArg("take")  ? server.arg("take").toInt()  : 600;
  long everyL = server.hasArg("every") ? server.arg("every").toInt() : 1;
  if (fromL  < 0)    fromL  = 0;
  if (takeL  < 1)    takeL  = 600;
  if (takeL  > 1000) takeL  = 1000;
  if (everyL < 1)    everyL = 1;
  const int from  = (int)fromL;
  const int take  = (int)takeL;
  const int every = (int)everyL;

  File f = SD.open(full, FILE_READ);
  if (!f) {
    server.send(500, "application/json", "{\"err\":\"open_failed\"}");
    return;
  }

  if (f.available()) {
    f.readStringUntil('\n'); // skip header
  }

  for (int i = 0; i < from && f.available(); ++i) {
    f.readStringUntil('\n');
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "");
  server.sendContent("{\"rows\":[");

  bool needComma = false;
  int processed = 0;
  int absIndex  = from;
  int sent      = 0;

  String line;
  while (f.available() && sent < take) {
    line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 5) { absIndex++; continue; }

    int commaPos[16];
    int cc = 0;
    const int L = line.length();
    for (int i = 0; i < L && cc < 16; ++i) {
      if (line.charAt(i) == ',') commaPos[cc++] = i;
    }
    if (cc < 8) { absIndex++; continue; } // minimal 9 kolom (0..8)

    auto col = [&](int k)->String {
      int start = (k == 0) ? 0 : (commaPos[k-1] + 1);
      int end   = (k < cc) ? commaPos[k] : L;
      if (start < 0) start = 0;
      if (end < start) end = start;
      return line.substring(start, end);
    };

    String elapsed_s = col(1);
    String dist_m    = col(2);
    String v_filt    = col(4);
    String lat       = (cc >= 6) ? col(5) : "";
    String lon       = (cc >= 7) ? col(6) : "";
    String hdop      = (cc >= 8) ? col(7) : "";

    if (elapsed_s.length() == 0 || dist_m.length() == 0 || v_filt.length() == 0) {
      absIndex++; continue;
    }

    if ((absIndex % every) == 0) {
      String item = "{\"elapsed_s\":" + elapsed_s
                  + ",\"dist_m\":"    + dist_m
                  + ",\"v_filt\":"    + v_filt
                  + ",\"lat\":"       + (lat.length()? lat : "null")
                  + ",\"lon\":"       + (lon.length()? lon : "null")
                  + ",\"hdop\":\""    + hdop + "\"}";
      if (needComma) server.sendContent(",");
      server.sendContent(item);
      needComma = true;
      sent++;
    }

    processed++;
    absIndex++;
  }

  int next = f.available() ? absIndex : -1;
  f.close();

  server.sendContent("],\"next\":");
  server.sendContent(String(next));
  server.sendContent(",\"debug\":{\"from\":");
  server.sendContent(String(from));
  server.sendContent(",\"processed\":");
  server.sendContent(String(processed));
  server.sendContent(",\"sent\":");
  server.sendContent(String(sent));
  server.sendContent("}}");
}


void handleApiRace() {
  String file = server.arg("file");
  String full;
  if (!validRacePath(file, full)) { server.send(404, "application/json", "{\"rows\":[]}"); return; }

  File f = SD.open(full, FILE_READ);
  if (!f) { server.send(500, "application/json", "{\"rows\":[]}"); return; }

  // Batas maksimal 1 MB
  const uint32_t MAX_BYTES = 1000000UL;

  String json = "{\"rows\":[";
  bool firstRow = true;
  String line;
  while (f.available()) {
    line = f.readStringUntil('\n');
    if (line.length() < 5) continue;
    if (line.startsWith("ts_ms")) continue; // skip header

    // parse first 10 kolom CSV
    int idx = 0, last = 0, col = 0;
    double ts=0, el=0, dist=0, vraw=0, vf=0, lat=0, lon=0; 
    String shdop=""; 
    int sats=0; 
    String type="-";

    while (idx >= 0 && col < 10) {
      idx = line.indexOf(',', last);
      String tok = (idx >= 0) ? line.substring(last, idx) : line.substring(last);
      switch (col) {
        case 0: ts   = tok.toDouble(); break;
        case 1: el   = tok.toDouble(); break;
        case 2: dist = tok.toDouble(); break;
        case 3: vraw = tok.toDouble(); break;
        case 4: vf   = tok.toDouble(); break;
        case 5: lat  = tok.toDouble(); break;
        case 6: lon  = tok.toDouble(); break;
        case 7: shdop= tok;            break;
        case 8: sats = tok.toInt();    break;
        case 9: type = tok;            break;
      }
      if (idx < 0) break;
      last = idx + 1;
      col++;
    }

    if (!firstRow) json += ",";
    firstRow = false;

    json += "{\"elapsed_s\":" + String(el, 3)
         +  ",\"dist_m\":"    + String(dist, 3)
         +  ",\"v_raw\":"     + String(vraw, 3)
         +  ",\"v_filt\":"    + String(vf, 3)
         +  ",\"lat\":"       + String(lat, 6)
         +  ",\"lon\":"       + String(lon, 6)
         +  ",\"hdop\":\""    + shdop + "\""
         +  ",\"sats\":"      + String(sats)
         +  ",\"type\":\""    + type + "\"}";

    // stop jika sudah melewati 1 MB
    if (f.position() > MAX_BYTES) break;
  }
  f.close();
  json += "]}";
  server.send(200, "application/json", json);
}


void handleDownload() {
  String file = server.arg("file");
  String full;
  if (!validRacePath(file, full)) { server.send(404, "text/plain", "Not found"); return; }
  File f = SD.open(full, FILE_READ);
  if (!f) { server.send(500, "text/plain", "Open failed"); return; }
  server.streamFile(f, "text/csv");
  f.close();
}

void handleDelete() {
  if (!sdReady) { server.send(500,"text/plain","SD not ready"); return; }
  String file = server.hasArg("file") ? server.arg("file") : "";
  String full;
  if (!validRacePath(file, full)) { server.send(404, "text/plain", "Not found"); return; }
  bool ok = SD.remove(full);
  server.send(ok?200:500, "text/plain", ok?"OK":"FAIL");
}

void handleDeleteAll() {
  if (!sdReady) { server.send(500, "text/plain", "SD not ready"); return; }

  File dir = SD.open("/race");
  if (!dir) { server.send(500, "text/plain", "Open dir failed"); return; }

  bool anyFail = false;
  int deleted = 0;
  for (;;) {
    File f = dir.openNextFile();
    if (!f) break;                  // habis
    if (!f.isDirectory()) {
      String nm = f.name();         // bisa "xxx.csv" atau "/race/xxx.csv"
      // Normalisasi path penuh di /race
      String full = nm;
      if (!nm.startsWith("/")) {
        full = String("/race/") + nm;
      } else if (!nm.startsWith("/race/")) {
        // jaga-jaga kalau lib ngasih path lain
        int slash = nm.lastIndexOf('/');
        String base = (slash >= 0) ? nm.substring(slash + 1) : nm;
        full = String("/race/") + base;
      }

      f.close();                    // TUTUP dulu sebelum remove!

      if (full.endsWith(".csv")) {
        if (SD.exists(full)) {
          if (SD.remove(full)) {
            deleted++;
          } else {
            anyFail = true;
          }
        }
      } else {
        // bukan csv, lewati
      }
    } else {
      f.close();
    }
  }
  dir.close();

  if (anyFail) {
    server.send(500, "application/json", "{\"ok\":false}");
  } else {
    server.send(200, "application/json", String("{\"ok\":true,\"deleted\":") + deleted + "}");
  }
}

void handleResetRun() {
  // Zero hasil (RAM) + kembali READY/NOT_READY sesuai GPS
  for (int i=0;i<N_MILESTONE;i++){ milestones[i].achieved=false; milestones[i].time=0; milestones[i].speed=0; milestones[i].isTarget=false; }
  achieved_0_100=false; time_0_100=0; dist_0_100=0;
  maxSpeed=0.0f; totalDistance=0.0f;

  // buang file tmp kalau ada
  finalizeRunFile(false);

  float hdopVal = hdop.toFloat();
  if (hdopVal > 0 && hdopVal <= GPS_ACCURACY_THRESHOLD && satellites >= 6) currentState = STATE_READY;
  else currentState = STATE_NOT_READY;
  needsFullRedraw = true;

  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------- RENAME HANDLER (aman) ----------
static bool isSafeFileBase(const String& s) {
  if (s.length() < 1 || s.length() > 48) return false; // batas wajar
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if ( (c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') ||
          c == '_' || c == '-' ) {
      continue;
    }
    return false;
  }
  return true;
}

void handleRename() {
  if (!sdReady) { server.send(500, "application/json", "{\"ok\":false,\"err\":\"sd_not_ready\"}"); return; }

  // Ambil argumen
  String oldParam = server.hasArg("old") ? server.arg("old") : "";
  String newBase  = server.hasArg("nw")  ? server.arg("nw")  : "";

  // Validasi "old" (harus file yang ada di /race, validRacePath butuh param nama file)
  String oldFull;
  if (!validRacePath(oldParam, oldFull)) {
    server.send(404, "application/json", "{\"ok\":false,\"err\":\"old_not_found\"}");
    return;
  }

  // newBase: base name tanpa .csv, hanya [A-Za-z0-9_-]
  if (!isSafeFileBase(newBase)) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_new_name\"}");
    return;
  }

  // Bangun path baru di /race
  String newFull = "/race/" + newBase + ".csv";

  // Cegah overwrite
  if (SD.exists(newFull)) {
    server.send(409, "application/json", "{\"ok\":false,\"err\":\"conflict_exists\"}");
    return;
  }

  // Rename
  bool ok = SD.rename(oldFull, newFull);
  if (!ok) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"rename_failed\"}");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}


void registerWebRoutes() {
  server.on("/", HTTP_GET, handleIndex);  
  server.on("/rename", HTTP_POST, handleRename);   // <<-- NEW
  server.on("/race", HTTP_GET, handleRaceDetail);
  server.on("/api/race_page", HTTP_GET, handleApiRacePage);
  server.on("/api/race", HTTP_GET, handleApiRace);
  server.on("/dl", HTTP_GET, handleDownload);
  server.on("/delete", HTTP_POST, handleDelete);
  server.on("/delete_all", HTTP_POST, handleDeleteAll);   // <<-- BARU
  server.on("/reset", HTTP_POST, handleResetRun);
}


// ================== LOGGING HELPERS ==================
void openRunTempFile() {
  if (!sdReady || loggingActive) return;
  snprintf(tmpPath, sizeof(tmpPath), "/race/_run_%lu.tmp", (unsigned long)millis());
  runFile = SD.open(tmpPath, FILE_WRITE);
  if (!runFile) return;
  runFile.println(F("ts_ms,elapsed_s,dist_from_start_m,raw_speed_kmh,filtered_speed_kmh,lat,lon,hdop,sats,type"));
  runFile.flush();
  loggingActive = true;
  lastFlushPos = 0;
  logHead = 0;
}

inline void appendToRing(uint8_t typeCode) {
  if (!loggingActive) return;
  LogRow &r = logRing[logHead++];
  r.ts_ms = millis();
  r.elapsed_s = (r.ts_ms - startTime) / 1000.0f;
  r.dist_m = totalDistance - startDistance;
  r.v_raw = speedKmh;
  r.v_filt = filteredSpeed;
  r.lat = lastLat;
  r.lon = lastLon;
  r.hdop = hdop.toFloat();
  r.sats = satellites;
  r.type = typeCode;

  if (logHead >= LOG_RING_CAP) {
    for (uint16_t i = 0; i < LOG_RING_CAP; i++) {
      const LogRow& x = logRing[i];
      runFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%u,%u\n",
        (unsigned long)x.ts_ms, x.elapsed_s, x.dist_m, x.v_raw, x.v_filt,
        x.lat, x.lon, x.hdop, (unsigned)x.sats, (unsigned)x.type);
    }
    logHead = 0;
    if (runFile.position() - lastFlushPos > 32768) { runFile.flush(); lastFlushPos = runFile.position(); }
  }
}

void flushRingNow() {
  if (!loggingActive || logHead == 0) return;
  for (uint16_t i = 0; i < logHead; i++) {
    const LogRow& x = logRing[i];
    runFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%u,%u\n",
      (unsigned long)x.ts_ms, x.elapsed_s, x.dist_m, x.v_raw, x.v_filt,
      x.lat, x.lon, x.hdop, (unsigned)x.sats, (unsigned)x.type);
  }
  logHead = 0;
  runFile.flush();
  lastFlushPos = runFile.position();
}

void finalizeRunFile(bool success) {
  if (!loggingActive) return;
  flushRingNow();
  runFile.close();
  if (success) {
    char namebuf[64];
    if (rmc_date[0] && rmc_time[0]) {
      char yymmdd[7]{rmc_date[4],rmc_date[5],rmc_date[2],rmc_date[3],rmc_date[0],rmc_date[1],0};
      snprintf(namebuf, sizeof(namebuf), "/race/race_%s_%c%c%c%c%c%c.csv",
               yymmdd, rmc_time[0], rmc_time[1], rmc_time[2], rmc_time[3], rmc_time[4], rmc_time[5]);
    } else {
      snprintf(namebuf, sizeof(namebuf), "/race/race_%lu.csv", (unsigned long)millis());
    }
    SD.rename(tmpPath, namebuf);
  } else {
    SD.remove(tmpPath);
  }
  loggingActive = false;
  tmpPath[0] = 0;
  logHead = 0;
}

void cleanupOrphanTmp() {
  File dir = SD.open("/race");
  if (!dir) return;
  File f;
  while ((f = dir.openNextFile())) {
    if (!f.isDirectory()) {
      String n = f.name();
      if (n.startsWith("/race/_run_") && n.endsWith(".tmp")) SD.remove(n);
    }
    f.close();
  }
  dir.close();
}
