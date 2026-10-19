#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <DFRobotDFPlayerMini.h>
#include <TinyGPS++.h>
#include <LittleFS.h>

// ===== Forward declarations to defeat Arduino auto-prototype bugs =====
struct NavStateFile;

bool loadNavState(NavStateFile &ns);
void saveNavState(uint8_t fromIdx, uint8_t toIdx, uint16_t navIndex, double lat, double lon);
void clearNavState();

// --- NEW: GPS connect/loss buzzer one-shot ---
void gpsBeepStart(bool fixNow);
void gpsBeepUpdate(unsigned long now);

/* ===================== HAZARD ENUM ===================== */
enum Hazard : uint8_t {
  H_NONE = 0,
  H_FALL,
  H_HOLE,
  H_WATER,
  H_OBS_CLOSE,
  H_OBS_AHEAD,
  H_TOF_LOW_WALL
};

enum VoiceKind : uint8_t { VK_NONE=0, VK_HAZARD, VK_UI, VK_NAV };

/* ===================== PINS ===================== */
// Sensors
#define US_TRIG     12
#define US_ECHO     13
#define WATER_DO    15
#define BUZZER_PIN  10

// Vibration (MOSFET TRIG)
#define VIB_PIN     8    // GP8 -> MOSFET TRIG

// I2C (ToF + MPU + Compass)
#define SDA_PIN     20
#define SCL_PIN     21

// DFPlayer UART (Serial1)
#define MP3_TX_PIN  0   // Pico TX -> DFPlayer RX
#define MP3_RX_PIN  1   // Pico RX <- DFPlayer TX

// GPS UART (Serial2)
#define GPS_TX_PIN  4   // Pico TX -> GPS RX
#define GPS_RX_PIN  5   // Pico RX <- GPS TX
static const uint32_t GPS_BAUD = 9600;

// Buttons (active LOW)
#define BTN1 2
#define BTN2 3
#define BTN3 6
#define BTN4 7
#define BTN5 11

/* ===================== I2C ADDRESSES ===================== */
const uint8_t MPU_ADDR = 0x68;
const uint8_t QMC5883L_ADDR = 0x0D;

/* ===================== OBJECTS ===================== */
VL53L0X tof;
DFRobotDFPlayerMini mp3;
TinyGPSPlus gps;

bool tofOk = false;
bool mp3Ok = false;
bool fsOk  = false;

/* ===================== BUTTON SETTINGS ===================== */
const unsigned long DEBOUNCE_MS   = 35;
const unsigned long LONG_MS       = 1000;
const unsigned long SUPER_LONG_MS = 5000;

bool sensorsOn = false;    // default OFF (Safe Mode OFF)

/* ===================== LOCATION / UI (CONFIRMED ORDER) ===================== */
// Internal order:
// 0=HOME, 1=OFFICE, 2=SCHOOL, 3=BUS STOP, 4=SHOP
uint8_t curLoc = 0;
uint8_t dest   = 0;
bool curSelected  = false;
bool destSelected = false;

int last1=HIGH, last2=HIGH, last3=HIGH, last4=HIGH;
unsigned long lastEdge1=0, lastEdge2=0, lastEdge3=0, lastEdge4=0;

int last5=HIGH;
unsigned long btn5DownAt=0;

/* ===================== BASELINE ===================== */
int   baseToF   = 0;
float basePitch = 0;
float baseRoll  = 0;

bool baselineReady = false;
bool mpuOk = false;

/* ===================== THRESHOLDS ===================== */
// HOLE: ToF bigger than baseline
const int TOF_SMALL_HOLE_DELTA_MM = 80;
const int TOF_DEEP_HOLE_DELTA_MM  = 180;

// LOW WALL/CURB: ToF smaller than baseline
const int TOF_LOW_WALL_DELTA_MM = 50;
const uint8_t TOF_LOW_CONFIRM_N  = 2;

uint8_t holeCount = 0;
uint8_t lowWallCount = 0;

// Ultrasonic
const int US_OBS_MAX_CM = 40;
const int US_VERY_CM    = 30;

// Water filter
uint8_t waterLowCount = 0;
const uint8_t WATER_CONFIRM = 3;

// ToF trust gating
const float TOF_PITCH_GATE_DEG = 18.0;
const float TOF_ROLL_GATE_DEG  = 25.0;
const int   TOF_MAX_VALID_MM   = 600;

/* ===================== FALL DETECTION (MPU ONLY) ===================== */
const float FALL_ANGLE_DEG         = 15.0;
const unsigned long FALL_ANGLE_MS  = 150;
const float FREEFALL_G_THR         = 0.35;
const unsigned long FREEFALL_MS    = 180;

bool fallLatched = false;
unsigned long fallUntil = 0;
const unsigned long FALL_HOLD_MS = 1800;

unsigned long angleBadSince = 0;
unsigned long freeFallSince = 0;

/* ===================== VOICE TIMING ===================== */
const unsigned long VOICE_START_AFTER_STABLE_MS = 350;
const unsigned long INTERRUPT_CONFIRM_MS        = 150;
const unsigned long VOICE_GAP_AFTER_FINISH_MS   = 5000;

/* ===================== NAV VOICE TIMING ===================== */
const unsigned long NAV_REPEAT_MS      = 1600;
const unsigned long NAV_RECOMPUTE_MS   = 250;

/* ===================== PRINT TIMING ===================== */
unsigned long lastPrint = 0;
const unsigned long PRINT_EVERY_MS = 200;

/* ===================== BUZZER ENGINE ===================== */
unsigned long buzNext = 0;
bool buzOn = false;
uint8_t buzStep = 0;

void buzzerReset() {
  noTone(BUZZER_PIN);
  buzOn = false;
  buzStep = 0;
  buzNext = 0;
}

/* ===================== VIBRATION ENGINE ===================== */
const unsigned long VIB_PULSE_MS    = 160;
const unsigned long VIB_STABLE_MS   = 120;
const unsigned long VIB_COOLDOWN_MS = 700;

bool vibOn = false;
unsigned long vibOffAt = 0;
unsigned long vibNextAllowed = 0;

Hazard vibEpisodeHaz = H_NONE;
bool vibDidPulse = false;

bool vibWanted(Hazard h) {
  return (h == H_HOLE || h == H_WATER || h == H_TOF_LOW_WALL ||
          h == H_OBS_AHEAD || h == H_OBS_CLOSE);
}

void vibReset() {
  digitalWrite(VIB_PIN, LOW);
  vibOn = false;
  vibOffAt = 0;
  vibNextAllowed = 0;
  vibEpisodeHaz = H_NONE;
  vibDidPulse = false;
}

void vibStart(unsigned long now) {
  digitalWrite(VIB_PIN, HIGH);
  vibOn = true;
  vibOffAt = now + VIB_PULSE_MS;
  vibNextAllowed = now + VIB_COOLDOWN_MS;
}

void vibUpdate(unsigned long now) {
  if (vibOn && now >= vibOffAt) {
    digitalWrite(VIB_PIN, LOW);
    vibOn = false;
  }
}

/* ===================== MP3 VOICE STATE ===================== */
bool voicePlaying = false;
VoiceKind voiceKind = VK_NONE;
Hazard voiceHazard = H_NONE;

unsigned long pendingHiSince = 0;
Hazard pendingHi = H_NONE;

unsigned long nextHazardRepeatAllowed = 0;
bool fallVoiceSent = false;

/* ===================== UI QUEUE ===================== */
bool uiQueueActive = false;
uint16_t uiQueue[4];
uint8_t uiQLen = 0;
uint8_t uiQIdx = 0;

void uiQueueCancel() {
  uiQueueActive = false;
  uiQLen = 0;
  uiQIdx = 0;
}

/* ===================== PRIORITY + TRACK ===================== */
int hazPriority(Hazard h) {
  switch (h) {
    case H_FALL:         return 6;
    case H_HOLE:         return 5;
    case H_WATER:        return 4;
    case H_TOF_LOW_WALL: return 3;
    case H_OBS_CLOSE:    return 2;
    case H_OBS_AHEAD:    return 1;
    default:             return 0;
  }
}

uint16_t hazTrack(Hazard h) {
  switch (h) {
    case H_OBS_AHEAD:    return 1;
    case H_OBS_CLOSE:    return 2;
    case H_WATER:        return 3;
    case H_HOLE:         return 6;
    case H_TOF_LOW_WALL: return 7;
    case H_FALL:         return 38;
    default:             return 0;
  }
}

const char* hazName(Hazard h) {
  switch (h) {
    case H_FALL: return "FALL";
    case H_HOLE: return "HOLE";
    case H_WATER: return "WATER";
    case H_OBS_CLOSE: return "OBS_CLOSE";
    case H_OBS_AHEAD: return "OBS_AHEAD";
    case H_TOF_LOW_WALL: return "TOF_LOW_WALL";
    default: return "NONE";
  }
}

/* ===================== NAV/UI TRACK MAP ===================== */
const uint16_t TO_TRK[5]   = { 12, 13, 14, 15, 11 };
const uint16_t NAV_TRK[5]  = { 17, 18, 19, 20, 16 };
const uint16_t REC_TRK[5]  = { 22, 23, 24, 25, 21 };
const uint16_t FROM_TRK[5] = { 42, 43, 44, 45, 41 };

static inline uint16_t trTo(uint8_t idx)   { return TO_TRK[idx]; }
static inline uint16_t trNav(uint8_t idx)  { return NAV_TRK[idx]; }
static inline uint16_t trRec(uint8_t idx)  { return REC_TRK[idx]; }
static inline uint16_t trFrom(uint8_t idx) { return FROM_TRK[idx]; }

const uint16_t TR_ROUTE_MISSING     = 46;
const uint16_t TR_CUR_NOT_SELECTED  = 47;
const uint16_t TR_DEST_NOT_SELECTED = 48;

// Direction tracks
const uint16_t TR_STRAIGHT = 32;
const uint16_t TR_SLEFT    = 33;
const uint16_t TR_LEFT     = 34;
const uint16_t TR_SRIGHT   = 35;
const uint16_t TR_RIGHT    = 36;
const uint16_t TR_ARRIVED  = 37;

/* ===================== MPU6050 ===================== */
void mpuWrite(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}
void setupMPU() { mpuWrite(0x6B, 0x00); }

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;
  ax = (Wire.read()<<8) | Wire.read();
  ay = (Wire.read()<<8) | Wire.read();
  az = (Wire.read()<<8) | Wire.read();
  return true;
}

/* ===================== COMPASS (QMC5883L) ===================== */
void qmcWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setupQMC() {
  qmcWrite(0x0A, 0x80);
  delay(10);
  qmcWrite(0x09, 0xD5);
  qmcWrite(0x0B, 0x01);
}

bool readQMC(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;

  int16_t xRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t yRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t zRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
  x = xRaw; y = yRaw; z = zRaw;
  return true;
}

bool getHeadingDeg(float pitchDeg, float rollDeg, float &headingOut) {
  int16_t mx, my, mz;
  if (!readQMC(mx, my, mz)) return false;

  float pitch = pitchDeg * PI / 180.0f;
  float roll  = rollDeg  * PI / 180.0f;

  float x = (float)mx;
  float y = (float)my;
  float z = (float)mz;

  float Xh = x * cos(pitch) + z * sin(pitch);
  float Yh = x * sin(roll) * sin(pitch) + y * cos(roll) - z * sin(roll) * cos(pitch);

  float hdg = atan2(Yh, Xh) * 180.0f / PI;
  if (hdg < 0) hdg += 360.0f;

  headingOut = hdg;
  return true;
}

/* ===================== GPS ===================== */
double curLat = 0, curLon = 0;
bool gpsFix = false;

static inline bool isFixValid() {
  return gps.location.isValid() && gps.location.age() < 2000;
}

// --- NEW: GPS beep state (one-shot on fix change) ---
enum GpsBeepMode : uint8_t { GPSB_NONE=0, GPSB_ACQ, GPSB_LOST };
GpsBeepMode gpsBeepMode = GPSB_NONE;
uint8_t gpsBeepStep = 0;
unsigned long gpsBeepNext = 0;
unsigned long gpsBeepCooldownUntil = 0;
const unsigned long GPS_BEEP_COOLDOWN_MS = 1500;

void gpsBeepStart(bool fixNow) {
  unsigned long now = millis();
  if (now < gpsBeepCooldownUntil) return; // anti-flicker
  gpsBeepCooldownUntil = now + GPS_BEEP_COOLDOWN_MS;

  gpsBeepMode = fixNow ? GPSB_ACQ : GPSB_LOST;
  gpsBeepStep = 0;
  gpsBeepNext = 0;
}

void gpsBeepUpdate(unsigned long now) {
  if (gpsBeepMode == GPSB_NONE) return;
  if (now < gpsBeepNext) return;

  if (gpsBeepMode == GPSB_ACQ) { // low -> high
    switch (gpsBeepStep) {
      case 0: tone(BUZZER_PIN, 2000); gpsBeepNext = now + 110; gpsBeepStep = 1; break;
      case 1: noTone(BUZZER_PIN);   gpsBeepNext = now + 70;  gpsBeepStep = 2; break;
      case 2: tone(BUZZER_PIN, 3000); gpsBeepNext = now + 130; gpsBeepStep = 3; break;
      case 3: noTone(BUZZER_PIN);   gpsBeepNext = now + 70;  gpsBeepStep = 4; break;
      case 4: tone(BUZZER_PIN, 4000); gpsBeepNext = now + 130; gpsBeepStep = 5; break;
      default:
        noTone(BUZZER_PIN);
        gpsBeepMode = GPSB_NONE;
        break;
    }
  } else { // GPSB_LOST, high -> low
    switch (gpsBeepStep) {
      case 0: tone(BUZZER_PIN, 4000); gpsBeepNext = now + 110; gpsBeepStep = 1; break;
      case 1: noTone(BUZZER_PIN);   gpsBeepNext = now + 70;  gpsBeepStep = 2; break;
      case 2: tone(BUZZER_PIN, 3000); gpsBeepNext = now + 130; gpsBeepStep = 3; break;
      case 3: noTone(BUZZER_PIN);   gpsBeepNext = now + 70;  gpsBeepStep = 4; break;
      case 4: tone(BUZZER_PIN, 2000); gpsBeepNext = now + 130; gpsBeepStep = 5; break;
      default:
        noTone(BUZZER_PIN);
        gpsBeepMode = GPSB_NONE;
        break;
    }
  }
}

void updateGPS() {
  while (Serial2.available()) gps.encode(Serial2.read());

  bool newFix = isFixValid();
  if (newFix != gpsFix) gpsBeepStart(newFix);   // one-shot edge
  gpsFix = newFix;

  if (gpsFix) {
    curLat = gps.location.lat();
    curLon = gps.location.lng();
  }
}

/* ===================== ULTRASONIC ===================== */
int readCM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  long us = pulseIn(US_ECHO, HIGH, 30000);
  if (us == 0) return 999;
  return us / 58;
}

/* ===================== WATER FILTER ===================== */
bool waterConfirmedNow(bool &rawLowOut) {
  bool rawLow = (digitalRead(WATER_DO) == LOW);
  rawLowOut = rawLow;

  if (rawLow) { if (waterLowCount < 255) waterLowCount++; }
  else waterLowCount = 0;

  return (waterLowCount >= WATER_CONFIRM);
}

/* ===================== BUZZER PATTERNS ===================== */
void updateBuzzer(unsigned long now, Hazard h, int usCm, int holeDiff) {
  // NEW: if hazard starts, cancel any GPS status beep
  if (h != H_NONE && gpsBeepMode != GPSB_NONE) {
    gpsBeepMode = GPSB_NONE;
    gpsBeepStep = 0;
    gpsBeepNext = 0;
  }

  // NEW: when no hazard, allow GPS one-shot beep to run (don’t reset tone)
  if (h == H_NONE) {
    if (gpsBeepMode != GPSB_NONE) { gpsBeepUpdate(now); return; }
    buzzerReset();
    return;
  }

  if (now < buzNext) return;

  if (h == H_FALL) {
    switch (buzStep) {
      case 0: tone(BUZZER_PIN, 5200); buzNext = now + 120; buzStep = 1; break;
      case 1: noTone(BUZZER_PIN);     buzNext = now + 90;  buzStep = 2; break;
      case 2: tone(BUZZER_PIN, 5200); buzNext = now + 120; buzStep = 3; break;
      case 3: noTone(BUZZER_PIN);     buzNext = now + 90;  buzStep = 4; break;
      case 4: tone(BUZZER_PIN, 5200); buzNext = now + 120; buzStep = 5; break;
      default:noTone(BUZZER_PIN);     buzNext = now + 600; buzStep = 0; break;
    }
    return;
  }

  if (h == H_HOLE) {
    bool deep = (holeDiff >= TOF_DEEP_HOLE_DELTA_MM);
    int freq = deep ? 5600 : 5200;
    if (!buzOn) { tone(BUZZER_PIN, freq); buzOn = true;  buzNext = now + 1100; }
    else        { noTone(BUZZER_PIN);     buzOn = false; buzNext = now + 200;  }
    return;
  }

  if (h == H_WATER) {
    switch (buzStep) {
      case 0: tone(BUZZER_PIN, 4000); buzNext = now + 160; buzStep = 1; break;
      case 1: noTone(BUZZER_PIN);     buzNext = now + 120; buzStep = 2; break;
      case 2: tone(BUZZER_PIN, 4000); buzNext = now + 160; buzStep = 3; break;
      default:noTone(BUZZER_PIN);     buzNext = now + 900; buzStep = 0; break;
    }
    return;
  }

  if (h == H_TOF_LOW_WALL) {
    int f = (buzStep & 1) ? 3400 : 2600;
    if (!buzOn) {
      tone(BUZZER_PIN, f);
      buzOn = true;
      buzNext = now + 55;
    } else {
      noTone(BUZZER_PIN);
      buzOn = false;
      buzNext = now + 55;
      buzStep++;
    }
    return;
  }

  if (h == H_OBS_AHEAD || h == H_OBS_CLOSE) {
    int cm = constrain(usCm, 5, US_OBS_MAX_CM);
    int fMax = (h == H_OBS_CLOSE) ? 6200 : 5800;
    int fMin = (h == H_OBS_CLOSE) ? 3600 : 3200;

    int freq = map(cm, 5, US_OBS_MAX_CM, fMax, fMin);
    freq = constrain(freq, fMin, fMax);

    int gap = map(cm, 5, US_OBS_MAX_CM, 80, 320);
    gap = constrain(gap, 80, 320);

    int onMs = map(cm, 5, US_OBS_MAX_CM, 70, 35);
    onMs = constrain(onMs, 35, 70);

    if (!buzOn) { tone(BUZZER_PIN, freq); buzOn = true;  buzNext = now + onMs; }
    else        { noTone(BUZZER_PIN);     buzOn = false; buzNext = now + gap;  }
    return;
  }
}

/* ===================== MP3 HELPERS ===================== */
void playTrack(uint16_t n, VoiceKind kind, Hazard haz) {
  Serial.print("Request track "); Serial.println(n);

  if (!mp3Ok) {
    Serial.println("!! DFPlayer NOT OK (mp3Ok=false). No play.");
    return;
  }

  uiQueueCancel();
  mp3.stop();
  mp3.playMp3Folder(n);

  voicePlaying = true;
  voiceKind = kind;
  voiceHazard = haz;

  Serial.print("Play "); Serial.println(n);
}


void playUiTrack(uint16_t n)  { playTrack(n, VK_UI, H_NONE); }
void playNavTrack(uint16_t n) { playTrack(n, VK_NAV, H_NONE); }

void startHazardVoice(Hazard h) {
  if (!mp3Ok) return;
  if (h == H_FALL && fallVoiceSent) return;

  uint16_t trk = hazTrack(h);
  if (trk == 0) return;

  playTrack(trk, VK_HAZARD, h);
  if (h == H_FALL) fallVoiceSent = true;
}

void startUiSequence3(uint16_t a, uint16_t b, uint16_t c) {
  if (!mp3Ok) return;
  uiQueueActive = true;
  uiQueue[0] = a; uiQueue[1] = b; uiQueue[2] = c;
  uiQLen = 3;
  uiQIdx = 0;

  mp3.stop();
  mp3.playMp3Folder(uiQueue[0]);
  voicePlaying = true;
  voiceKind = VK_UI;
  voiceHazard = H_NONE;

  if (Serial) { Serial.print("Play "); Serial.println(uiQueue[0]); }
}

void handleMp3Events() {
  if (!mp3Ok) return;
  if (!mp3.available()) return;

  uint8_t type = mp3.readType();
  mp3.read();

  if (type == DFPlayerPlayFinished) {

    if (voiceKind == VK_UI && uiQueueActive) {
      uiQIdx++;
      if (uiQIdx < uiQLen) {
        mp3.playMp3Folder(uiQueue[uiQIdx]);
        voicePlaying = true;
        voiceKind = VK_UI;
        voiceHazard = H_NONE;
        nextHazardRepeatAllowed = millis();

        if (Serial) { Serial.print("Play "); Serial.println(uiQueue[uiQIdx]); }
        return;
      } else {
        uiQueueCancel();
      }
    }

    voicePlaying = false;

    if (voiceKind == VK_HAZARD) nextHazardRepeatAllowed = millis() + VOICE_GAP_AFTER_FINISH_MS;
    else nextHazardRepeatAllowed = millis();

    voiceKind = VK_NONE;
    voiceHazard = H_NONE;
  }
}

/* ===================== BUTTON HELPERS ===================== */
bool pressedEdge(int pin, int &lastState, unsigned long &lastEdgeAt) {
  int s = digitalRead(pin);
  unsigned long now = millis();
  bool edge = false;

  if (s != lastState && (now - lastEdgeAt) > DEBOUNCE_MS) {
    lastEdgeAt = now;
    if (lastState == HIGH && s == LOW) edge = true;
    lastState = s;
  }
  return edge;
}

/* ===================== BASELINE CALIBRATION ===================== */
void calibrateBaseline() {
  baselineReady = false;
  mpuOk = false;

  holeCount = 0;
  lowWallCount = 0;
  waterLowCount = 0;

  delay(600);

  if (tofOk) {
    long sum = 0;
    int good = 0;
    for (int i = 0; i < 6; i++) {
      int mm = tof.readRangeContinuousMillimeters();
      if (mm > 0 && mm != 65535 && mm <= TOF_MAX_VALID_MM) {
        sum += mm;
        good++;
      }
      delay(40);
    }
    baseToF = (good > 0) ? (int)(sum / good) : 0;
  } else {
    baseToF = 0;
  }

  int16_t ax, ay, az;
  float pSum = 0, rSum = 0;
  int pGood = 0;

  for (int i = 0; i < 10; i++) {
    if (readMPU(ax, ay, az)) {
      float axg=ax/16384.0, ayg=ay/16384.0, azg=az/16384.0;
      float pitch = atan2(axg, sqrt(ayg*ayg + azg*azg)) * 180.0 / PI;
      float roll  = atan2(ayg, sqrt(axg*axg + azg*azg)) * 180.0 / PI;
      pSum += pitch;
      rSum += roll;
      pGood++;
    }
    delay(25);
  }

  if (pGood > 0) {
    basePitch = pSum / pGood;
    baseRoll  = rSum / pGood;
    mpuOk = true;
  }

  baselineReady = true;
}

/* ===================== ROUTE STORAGE (LittleFS) ===================== */
const uint32_t MAGIC_ROUTE = 0x524F5554; // 'ROUT'
const uint16_t ROUTE_VER   = 2;

enum PointType : uint8_t { POINT_START = 0, POINT_NORMAL = 1, POINT_END = 2 };

struct RoutePoint {
  int32_t latE7;
  int32_t lonE7;
  uint8_t type;
};

const int MAX_POINTS = 200;
RoutePoint routeBuf[MAX_POINTS];
int pointCount = 0;

static inline int32_t toE7(double deg) { return (int32_t)llround(deg * 10000000.0); }
static inline double fromE7(int32_t e7) { return ((double)e7) / 10000000.0; }

void makeRoutePath(uint8_t fromIdx, uint8_t toIdx, char *out, size_t n) {
  snprintf(out, n, "/r_%u_%u.bin", (unsigned)fromIdx, (unsigned)toIdx);
}

bool routeExists(uint8_t fromIdx, uint8_t toIdx) {
  if (!fsOk) return false;
  char path[24];
  makeRoutePath(fromIdx, toIdx, path, sizeof(path));
  return LittleFS.exists(path);
}

bool saveRoute(uint8_t fromIdx, uint8_t toIdx) {
  if (!fsOk) return false;
  char path[24];
  makeRoutePath(fromIdx, toIdx, path, sizeof(path));

  File f = LittleFS.open(path, "w");
  if (!f) return false;

  f.write((uint8_t*)&MAGIC_ROUTE, 4);
  f.write((uint8_t*)&ROUTE_VER, 2);
  f.write(&fromIdx, 1);
  f.write(&toIdx, 1);

  uint16_t cnt = (uint16_t)pointCount;
  f.write((uint8_t*)&cnt, 2);

  for (int i = 0; i < pointCount; i++) {
    f.write((uint8_t*)&routeBuf[i].latE7, 4);
    f.write((uint8_t*)&routeBuf[i].lonE7, 4);
    f.write((uint8_t*)&routeBuf[i].type, 1);
  }

  f.close();
  return true;
}

bool loadRoute(uint8_t fromIdx, uint8_t toIdx) {
  if (!fsOk) return false;
  char path[24];
  makeRoutePath(fromIdx, toIdx, path, sizeof(path));

  File f = LittleFS.open(path, "r");
  if (!f) return false;

  uint32_t m=0; uint16_t v=0;
  f.read((uint8_t*)&m, 4);
  f.read((uint8_t*)&v, 2);
  if (m != MAGIC_ROUTE || v != ROUTE_VER) { f.close(); return false; }

  uint8_t fFrom=255, fTo=255;
  f.read(&fFrom, 1);
  f.read(&fTo, 1);
  if (fFrom != fromIdx || fTo != toIdx) { f.close(); return false; }

  uint16_t cnt=0;
  f.read((uint8_t*)&cnt, 2);
  if (cnt > MAX_POINTS) cnt = MAX_POINTS;

  pointCount = (int)cnt;
  for (int i = 0; i < pointCount; i++) {
    f.read((uint8_t*)&routeBuf[i].latE7, 4);
    f.read((uint8_t*)&routeBuf[i].lonE7, 4);
    f.read((uint8_t*)&routeBuf[i].type, 1);
  }
  f.close();

  return (pointCount >= 2);
}

/* ===================== NAV STATE (backup resume only) ===================== */
const char *NAVSTATE_FILE = "/navstate.bin";
const uint32_t MAGIC_NAVS = 0x4E415653; // 'NAVS'
const uint16_t NAVS_VER   = 1;

struct NavStateFile {
  uint32_t magic;
  uint16_t ver;
  uint8_t fromIdx;
  uint8_t toIdx;
  uint16_t navIndex;
  int32_t lastLatE7;
  int32_t lastLonE7;
};

bool loadNavState(NavStateFile &ns) {
  if (!fsOk) return false;
  File f = LittleFS.open(NAVSTATE_FILE, "r");
  if (!f) return false;
  if (f.available() < (int)sizeof(NavStateFile)) { f.close(); return false; }
  f.read((uint8_t*)&ns, sizeof(NavStateFile));
  f.close();
  return (ns.magic == MAGIC_NAVS && ns.ver == NAVS_VER);
}

void saveNavState(uint8_t fromIdx, uint8_t toIdx, uint16_t navIndex, double lat, double lon) {
  if (!fsOk) return;
  NavStateFile ns;
  ns.magic = MAGIC_NAVS;
  ns.ver = NAVS_VER;
  ns.fromIdx = fromIdx;
  ns.toIdx = toIdx;
  ns.navIndex = navIndex;
  ns.lastLatE7 = toE7(lat);
  ns.lastLonE7 = toE7(lon);

  File f = LittleFS.open(NAVSTATE_FILE, "w");
  if (!f) return;
  f.write((uint8_t*)&ns, sizeof(NavStateFile));
  f.close();
}

void clearNavState() {
  if (!fsOk) return;
  if (LittleFS.exists(NAVSTATE_FILE)) LittleFS.remove(NAVSTATE_FILE);
}

/* ===================== NAV MATH ===================== */
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double radLat1 = lat1 * PI / 180.0;
  double radLat2 = lat2 * PI / 180.0;
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;

  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radLat1)*cos(radLat2) * sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double φ1 = lat1 * PI / 180.0;
  double φ2 = lat2 * PI / 180.0;
  double Δλ = (lon2 - lon1) * PI / 180.0;

  double y = sin(Δλ) * cos(φ2);
  double x = cos(φ1)*sin(φ2) - sin(φ1)*cos(φ2)*cos(Δλ);
  double brng = atan2(y, x) * 180.0 / PI;
  if (brng < 0) brng += 360.0;
  return brng;
}

float angleDiffDeg(float target, float heading) {
  float d = target - heading;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

/* ===================== NAV ENGINE ===================== */
enum NavMode : uint8_t { NAV_IDLE=0, NAV_SEEK_NEAREST, NAV_ACTIVE, NAV_RECORD };
NavMode navMode = NAV_IDLE;

uint8_t navFrom = 0;
uint8_t navTo   = 0;
int navIndex = 0;        // used in NAV_ACTIVE
int seekIndex = 0;       // used in NAV_SEEK_NEAREST

unsigned long navLastCompute = 0;
unsigned long navLastSaidAt  = 0;
uint16_t navLastTrack = 0;

unsigned long navLastStateSave = 0;

const double SEEK_REACHED_M    = 8.0;   // when close to the nearest waypoint -> start NAV_ACTIVE there
const double WAYPOINT_RADIUS_M = 6.0;   // waypoint reached during NAV_ACTIVE
const double ARRIVE_RADIUS_M   = 7.0;

bool recHasLast = false;
double recLastLat = 0, recLastLon = 0;
const double MIN_STEP_METERS = 4.0;

void navStop(bool clearState=true) {
  navMode = NAV_IDLE;
  navIndex = 0;
  seekIndex = 0;
  navLastTrack = 0;
  navLastSaidAt = 0;
  navLastCompute = 0;
  navLastStateSave = 0;
  if (clearState) clearNavState();
}

bool tryResumeNav(uint8_t fromIdx, uint8_t toIdx) {
  if (!gpsFix || !fsOk) return false;

  NavStateFile ns;
  if (!loadNavState(ns)) return false;
  if (ns.fromIdx != fromIdx || ns.toIdx != toIdx) return false;

  double sLat = fromE7(ns.lastLatE7);
  double sLon = fromE7(ns.lastLonE7);
  double d = distanceMeters(curLat, curLon, sLat, sLon);

  if (d <= 25.0) {
    if (!loadRoute(fromIdx, toIdx)) return false;
    navFrom = fromIdx;
    navTo = toIdx;
    navIndex = (int)ns.navIndex;
    if (navIndex < 0) navIndex = 0;
    if (navIndex >= pointCount) navIndex = pointCount - 1;
    navMode = NAV_ACTIVE;
    return true;
  }

  clearNavState();
  return false;
}

int findNearestWaypoint(double lat, double lon, double &bestDistOut) {
  bestDistOut = 1e18;
  if (pointCount < 2) return -1;
  int bestIdx = -1;
  for (int i = 0; i < pointCount; i++) {
    double wLat = fromE7(routeBuf[i].latE7);
    double wLon = fromE7(routeBuf[i].lonE7);
    double d = distanceMeters(lat, lon, wLat, wLon);
    if (d < bestDistOut) { bestDistOut = d; bestIdx = i; }
  }
  return bestIdx;
}

uint16_t decideNavTrack(float diffAbs, bool rightSide) {
  if (diffAbs < 12.0f) return TR_STRAIGHT;
  if (diffAbs < 30.0f) return rightSide ? TR_SRIGHT : TR_SLEFT;
  return rightSide ? TR_RIGHT : TR_LEFT;
}

bool navStart(uint8_t fromIdx, uint8_t toIdx) {
  if (!routeExists(fromIdx, toIdx)) return false;
  if (!loadRoute(fromIdx, toIdx)) return false;

  navFrom = fromIdx;
  navTo   = toIdx;

  // backup resume only
  if (tryResumeNav(fromIdx, toIdx)) {
    navLastTrack = 0;
    navLastSaidAt = 0;
    return true;
  }

  // seek to nearest waypoint
  double bestD;
  int idx = findNearestWaypoint(curLat, curLon, bestD);
  if (idx < 0) return false;

  seekIndex = idx;
  navMode = NAV_SEEK_NEAREST;
  navLastTrack = 0;
  navLastSaidAt = 0;
  return true;
}

void navUpdate(unsigned long now, float pitchDeg, float rollDeg, Hazard currentHaz) {
  if (navMode == NAV_IDLE) return;
  if (!gpsFix) return;

  if (now - navLastCompute < NAV_RECOMPUTE_MS) return;
  navLastCompute = now;

  if (navMode == NAV_RECORD) return;
  if (pointCount < 2) { navStop(true); return; }

  if (currentHaz != H_NONE) return;
  if (!mp3Ok) return;
  if (voicePlaying) return;

  float heading;
  if (!getHeadingDeg(pitchDeg, rollDeg, heading)) return;

  if (navMode == NAV_SEEK_NEAREST) {
    double bestD;
    int idx = findNearestWaypoint(curLat, curLon, bestD);
    if (idx >= 0) seekIndex = idx;

    double tLat = fromE7(routeBuf[seekIndex].latE7);
    double tLon = fromE7(routeBuf[seekIndex].lonE7);
    double dist = distanceMeters(curLat, curLon, tLat, tLon);

    if (dist <= SEEK_REACHED_M) {
      navMode = NAV_ACTIVE;
      navIndex = seekIndex;
      navLastTrack = 0;
      navLastSaidAt = 0;
      return;
    }

    double brng = bearingDeg(curLat, curLon, tLat, tLon);
    float diff = angleDiffDeg((float)brng, heading);

    bool needRight = (diff > 0);
    float absDiff = fabs(diff);
    uint16_t trk = decideNavTrack(absDiff, needRight);

    bool changed = (trk != navLastTrack);
    bool canRepeat = (now - navLastSaidAt >= NAV_REPEAT_MS);

    if (changed || canRepeat) {
      playNavTrack(trk);
      navLastTrack = trk;
      navLastSaidAt = now;
    }
    return;
  }

  if (navIndex < 0) navIndex = 0;
  if (navIndex >= pointCount) navIndex = pointCount - 1;

  double tLat = fromE7(routeBuf[navIndex].latE7);
  double tLon = fromE7(routeBuf[navIndex].lonE7);

  double dist = distanceMeters(curLat, curLon, tLat, tLon);

  if (dist <= WAYPOINT_RADIUS_M) {
    navIndex++;
    if (navIndex >= pointCount) {
      playNavTrack(TR_ARRIVED);
      navStop(true);
      return;
    }
    tLat = fromE7(routeBuf[navIndex].latE7);
    tLon = fromE7(routeBuf[navIndex].lonE7);
  }

  double endLat = fromE7(routeBuf[pointCount-1].latE7);
  double endLon = fromE7(routeBuf[pointCount-1].lonE7);
  double distEnd = distanceMeters(curLat, curLon, endLat, endLon);
  if (distEnd <= ARRIVE_RADIUS_M) {
    playNavTrack(TR_ARRIVED);
    navStop(true);
    return;
  }

  double brng = bearingDeg(curLat, curLon, tLat, tLon);
  float diff = angleDiffDeg((float)brng, heading);

  bool needRight = (diff > 0);
  float absDiff = fabs(diff);
  uint16_t trk = decideNavTrack(absDiff, needRight);

  bool changed = (trk != navLastTrack);
  bool canRepeat = (now - navLastSaidAt >= NAV_REPEAT_MS);

  if (changed || canRepeat) {
    playNavTrack(trk);
    navLastTrack = trk;
    navLastSaidAt = now;
  }

  if (fsOk && now - navLastStateSave >= 5000) {
    navLastStateSave = now;
    saveNavState(navFrom, navTo, (uint16_t)navIndex, curLat, curLon);
  }
}

/* ===================== RECORD ENGINE ===================== */
bool recStart(uint8_t fromIdx, uint8_t toIdx) {
  if (!fsOk) return false;
  if (!gpsFix) return false;

  navFrom = fromIdx;
  navTo = toIdx;

  navMode = NAV_RECORD;
  pointCount = 0;
  recHasLast = false;

  routeBuf[0] = { toE7(curLat), toE7(curLon), POINT_START };
  pointCount = 1;
  recHasLast = true;
  recLastLat = curLat;
  recLastLon = curLon;

  return true;
}

bool recStopAndSave() {
  if (navMode != NAV_RECORD) return false;
  if (pointCount >= 1) routeBuf[pointCount-1].type = POINT_END;

  bool ok = saveRoute(navFrom, navTo);

  navMode = NAV_IDLE;
  recHasLast = false;

  clearNavState();

  tone(BUZZER_PIN, ok ? 4800 : 2800, 120);
  delay(160);
  tone(BUZZER_PIN, ok ? 5200 : 2400, 120);
  delay(160);
  noTone(BUZZER_PIN);

  return ok;
}

/* ===================== HAZARD DECISION ===================== */
Hazard decideHazard(bool fallNow,
                    bool tofTrusted,
                    int us, int tofmm,
                    bool waterOk,
                    int &holeDiffOut) {
  holeDiffOut = 0;

  if (!baselineReady) return H_NONE;
  if (fallNow) return H_FALL;

  if (tofOk && tofTrusted && baseToF > 0 && tofmm > 0 && tofmm != 65535) {
    int diff = tofmm - baseToF;
    holeDiffOut = diff;

    if (diff >= TOF_SMALL_HOLE_DELTA_MM) { if (holeCount < 255) holeCount++; }
    else holeCount = 0;

    if (holeCount >= 3) return H_HOLE;
  } else holeCount = 0;

  if (waterOk) return H_WATER;

  if (tofOk && tofTrusted && baseToF > 0 && tofmm > 0 && tofmm != 65535) {
    int lowDiff = baseToF - tofmm;

    if (lowDiff >= TOF_LOW_WALL_DELTA_MM) { if (lowWallCount < 255) lowWallCount++; }
    else lowWallCount = 0;

    if (lowWallCount >= TOF_LOW_CONFIRM_N) return H_TOF_LOW_WALL;
  } else lowWallCount = 0;

  if (us < US_OBS_MAX_CM) {
    if (us <= US_VERY_CM) return H_OBS_CLOSE;
    return H_OBS_AHEAD;
  }

  return H_NONE;
}

/* ===================== NAV ACTIONS (BTN5) ===================== */
void startNavigationUi() {
  if (!curSelected)  { playUiTrack(TR_CUR_NOT_SELECTED);  return; }
  if (!destSelected) { playUiTrack(TR_DEST_NOT_SELECTED); return; }

  if (!routeExists(curLoc, dest)) {
    startUiSequence3(TR_ROUTE_MISSING, trFrom(curLoc), trTo(dest));
    return;
  }

  playUiTrack(trNav(dest));
  navStart(curLoc, dest);
}

void startRecordUi() {
  if (!curSelected)  { playUiTrack(TR_CUR_NOT_SELECTED);  return; }
  if (!destSelected) { playUiTrack(TR_DEST_NOT_SELECTED); return; }

  if (navMode == NAV_RECORD) {
    recStopAndSave();
    return;
  }

  playUiTrack(trRec(dest));
  recStart(curLoc, dest);
}

/* ===================== BUTTON HANDLER ===================== */
void handleButtons(unsigned long now) {
  if (pressedEdge(BTN1, last1, lastEdge1)) playUiTrack(10);

  if (pressedEdge(BTN2, last2, lastEdge2)) {
    sensorsOn = !sensorsOn;

    if (!sensorsOn) {
      buzzerReset();
      vibReset();
      holeCount = 0; lowWallCount = 0; waterLowCount = 0;
      fallLatched = false;
      fallVoiceSent = false;
      baselineReady = false;
      playUiTrack(8);
    } else {
      playUiTrack(9);
      calibrateBaseline();
      vibReset();
    }
  }

  if (pressedEdge(BTN3, last3, lastEdge3)) {
    if (!curSelected) curSelected = true;
    else curLoc = (curLoc + 1) % 5;

    destSelected = false;
    playUiTrack(trFrom(curLoc));
  }

  if (pressedEdge(BTN4, last4, lastEdge4)) {
    if (!curSelected) {
      playUiTrack(TR_CUR_NOT_SELECTED);
    } else {
      if (!destSelected) { dest = curLoc; destSelected = true; }
      do { dest = (dest + 1) % 5; } while (dest == curLoc);
      playUiTrack(trTo(dest));
    }
  }

  int s5 = digitalRead(BTN5);

  if (last5 == HIGH && s5 == LOW) btn5DownAt = now;

  if (last5 == LOW && s5 == HIGH) {
    unsigned long held = now - btn5DownAt;

    if (held >= SUPER_LONG_MS) {
      if (navMode == NAV_ACTIVE || navMode == NAV_SEEK_NEAREST || navMode == NAV_RECORD) {
        navStop(true);
        uiQueueCancel();
        mp3.stop();
        voicePlaying = false;
        voiceKind = VK_NONE;
        voiceHazard = H_NONE;
        tone(BUZZER_PIN, 2600, 120);
        delay(140);
        tone(BUZZER_PIN, 2200, 120);
        delay(140);
        noTone(BUZZER_PIN);
      }
    } else if (held >= LONG_MS) {
      startRecordUi();
    } else if (held >= 40) {
      if (navMode == NAV_ACTIVE || navMode == NAV_SEEK_NEAREST) {
        navStop(true);
        tone(BUZZER_PIN, 2400, 90);
        delay(120);
        noTone(BUZZER_PIN);
      } else {
        startNavigationUi();
      }
    }
  }

  last5 = s5;
}

/* ===================== SETUP ===================== */
void setup() {
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(WATER_DO, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
  pinMode(BTN5, INPUT_PULLUP);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1200) delay(10);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  tof.setTimeout(200);
  tofOk = tof.init();
  if (tofOk) tof.startContinuous(50);

  setupMPU();
  setupQMC();

#if defined(ARDUINO_ARCH_RP2040)
  Serial2.setTX(GPS_TX_PIN);
  Serial2.setRX(GPS_RX_PIN);
#endif
  Serial2.begin(GPS_BAUD);

#if defined(ARDUINO_ARCH_RP2040)
  Serial1.setTX(MP3_TX_PIN);
  Serial1.setRX(MP3_RX_PIN);
#endif
  Serial1.begin(9600);

  mp3Ok = mp3.begin(Serial1);
  Serial.print("mp3Ok = "); Serial.println(mp3Ok);
  Serial.println("If mp3Ok=false: check DFPlayer power (VCC), GND, TX/RX, SD card.");

  if (mp3Ok) {
    mp3.volume(24);
    mp3.EQ(DFPLAYER_EQ_NORMAL);
  }

  fsOk = LittleFS.begin();

  tone(BUZZER_PIN, 4200, 120); delay(180);
  tone(BUZZER_PIN, 4800, 120); delay(180);
  noTone(BUZZER_PIN);

  playUiTrack(31);
  delay(2000);
  playUiTrack(8);

  sensorsOn = false;
  baselineReady = false;
  vibReset();

  curSelected = false;
  destSelected = false;
  curLoc = 0; // HOME
  dest = 0;

  navStop(true);
  nextHazardRepeatAllowed = millis();
}

/* ===================== LOOP ===================== */
void loop() {
  unsigned long now = millis();

  handleButtons(now);
  handleMp3Events();
  vibUpdate(now);

  updateGPS();

  // --- MPU read (for fall + compass tilt comp) ---
  int16_t ax=0, ay=0, az=0;
  float pitch = basePitch, roll = baseRoll;
  float aMag = 1.0;
  bool fallNow = false;

  if (readMPU(ax, ay, az)) {
    float axg = ax / 16384.0;
    float ayg = ay / 16384.0;
    float azg = az / 16384.0;

    pitch = atan2(axg, sqrt(ayg*ayg + azg*azg)) * 180.0 / PI;
    roll  = atan2(ayg, sqrt(axg*axg + azg*azg)) * 180.0 / PI;
    aMag  = sqrt(axg*axg + ayg*ayg + azg*azg);

    if (baselineReady && mpuOk) {
      bool angleBad = (fabs(pitch - basePitch) > FALL_ANGLE_DEG) ||
                      (fabs(roll  - baseRoll)  > FALL_ANGLE_DEG);
      if (angleBad) { if (!angleBadSince) angleBadSince = now; }
      else angleBadSince = 0;

      bool lowG = (aMag < FREEFALL_G_THR);
      if (lowG) { if (!freeFallSince) freeFallSince = now; }
      else freeFallSince = 0;

      if ((angleBadSince && now - angleBadSince >= FALL_ANGLE_MS) ||
          (freeFallSince && now - freeFallSince >= FREEFALL_MS)) {
        if (!fallLatched) fallVoiceSent = false;
        fallLatched = true;
        fallUntil = now + FALL_HOLD_MS;
      }
    }
  }

  if (fallLatched && now > fallUntil) fallLatched = false;
  fallNow = fallLatched;

  // --- Hazard sensing only when Safe Mode ON ---
  static Hazard currentHaz = H_NONE;
  static unsigned long hazChangedAt = 0;
  int us = 999;
  int tofmm = -1;
  bool waterRawLow = false;
  bool waterOk = false;
  int dHole = 0;

  if (sensorsOn) {
    us = readCM();
    tofmm = tofOk ? tof.readRangeContinuousMillimeters() : -1;
    waterOk = waterConfirmedNow(waterRawLow);

    bool tofTrusted = true;
    if (!tofOk) tofTrusted = false;
    if (tofmm <= 0 || tofmm == 65535 || tofmm > TOF_MAX_VALID_MM) tofTrusted = false;

    if (baselineReady && mpuOk) {
      if (fabs(pitch - basePitch) > TOF_PITCH_GATE_DEG) tofTrusted = false;
      if (fabs(roll  - baseRoll)  > TOF_ROLL_GATE_DEG)  tofTrusted = false;
    }

    Hazard haz = decideHazard(fallNow, tofTrusted, us, tofmm, waterOk, dHole);

    if (haz != currentHaz) {
      currentHaz = haz;
      hazChangedAt = now;
      buzzerReset();
    }

    if (currentHaz != vibEpisodeHaz) {
      vibEpisodeHaz = currentHaz;
      vibDidPulse = false;
    }

    if (!vibDidPulse &&
        baselineReady &&
        vibWanted(currentHaz) &&
        (now - hazChangedAt >= VIB_STABLE_MS) &&
        (now >= vibNextAllowed)) {
      vibStart(now);
      vibDidPulse = true;
    }

    updateBuzzer(now, currentHaz, us, dHole);

    // Hazard voice interrupt logic
    if (voicePlaying) {
      int curP = hazPriority(currentHaz);
      int playingP = (voiceKind == VK_HAZARD) ? hazPriority(voiceHazard) : 0;

      if (curP > playingP) {
        if (pendingHi != currentHaz) {
          pendingHi = currentHaz;
          pendingHiSince = now;
        } else if (now - pendingHiSince >= INTERRUPT_CONFIRM_MS) {
          if (mp3Ok) mp3.stop();
          uiQueueCancel();
          voicePlaying = false;
          voiceKind = VK_NONE;
          voiceHazard = H_NONE;
          nextHazardRepeatAllowed = now;
          pendingHi = H_NONE;
        }
      } else pendingHi = H_NONE;
    } else pendingHi = H_NONE;

    if (!voicePlaying && mp3Ok && currentHaz != H_NONE) {
      bool stable = (now - hazChangedAt >= VOICE_START_AFTER_STABLE_MS);
      if (stable && now >= nextHazardRepeatAllowed) {
        startHazardVoice(currentHaz);
      }
    }
  } else {
    currentHaz = H_NONE;

    // NEW: allow GPS one-shot beep even when Safe Mode is OFF
    if (gpsBeepMode != GPSB_NONE) gpsBeepUpdate(now);
    else buzzerReset();

    vibReset();
  }

  // --- Recording points ---
  if (navMode == NAV_RECORD && gpsFix && pointCount < MAX_POINTS) {
    double d = distanceMeters(recLastLat, recLastLon, curLat, curLon);
    if (!recHasLast || d >= MIN_STEP_METERS) {
      routeBuf[pointCount++] = { toE7(curLat), toE7(curLon), POINT_NORMAL };
      recHasLast = true;
      recLastLat = curLat;
      recLastLon = curLon;
    }
  }

  // --- Navigation update ---
  navUpdate(now, pitch, roll, currentHaz);

  delay(20);
}
