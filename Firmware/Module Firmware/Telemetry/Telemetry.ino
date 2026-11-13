#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

#ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
#endif

// ======================================================
// CONFIG
// ======================================================
#define BUTTON_ACTIVE_LOW 0

#define OWN_ADDRESS 40

// Buttons, in panel order. The pressed button is the page (mode) reported to the hub.
#define BTN_FLIGHT 9   // AL/VL
#define BTN_MNVR   8
#define BTN_ORBIT  7
#define BTN_FUEL   6

#define PAGE_FLIGHT 1
#define PAGE_MNVR   2
#define PAGE_ORBIT  3
#define PAGE_FUEL   4

#define FIELD_COUNT       20
#define FIELD_LEN         9    // 8 chars + null
#define FIELDS_PER_PACKET 4
#define PACKET_COUNT      (FIELD_COUNT / FIELDS_PER_PACKET)

#define DRAW_PERIOD_MS 250  // 4 Hz fallback redraw

// ======================================================
// FIELD MAP (keep in sync with Hub_3_1/definitions.h LCD_F_*)
// ======================================================
// Field 0 tags which page the data is for ('1'..'4'), or "X" = not flying (screen blanks).
#define F_TAG  0
// Same meaning on every page that shows them
#define F_APO  1
#define F_PER  2
#define F_TTA  3
#define F_TTP  4
#define F_DVT  5
#define F_DVS  6
#define F_ALT  7
// Flight page
#define F_HDG  8
#define F_PIT  9
#define F_RLL  10
#define F_SVH  11
#define F_SVP  12
#define F_SPD  13
#define F_VRT  14
#define F_HRZ  15
#define F_RHO  16
#define F_MCH  17
#define F_G    18
#define F_THR  19
// Maneuver page
#define F_NDV  8
#define F_TTN  9
#define F_TTB  10
#define F_BRN  11
#define F_ADV  12
#define F_NHD  13
#define F_NPT  14
#define F_MRG  15
// Orbit page
#define F_SMA  8
#define F_ECC  9
#define F_INC  10
#define F_PRD  11
#define F_LAN  12
#define F_AOP  13
#define F_TA   14
#define F_VOB  15
#define F_SOI  16
// Fuel page (percent, "-" = none aboard)
#define F_LF   8
#define F_OX   9
#define F_SF   10
#define F_EC   11
#define F_XE   12
#define F_MP   13

// ======================================================
// DISPLAY
// ======================================================
U8G2_ST7920_128X64_2_SW_SPI u8g2(
  U8G2_R2,
  /* clock=*/ 11,
  /* data=*/  13,
  /* CS=*/    10,
  /* reset=*/ U8X8_PIN_NONE
);

// ======================================================
// DATA BUFFERS
// ======================================================
static char in_back [FIELD_COUNT][FIELD_LEN];
static char in_front[FIELD_COUNT][FIELD_LEN];

static volatile bool packetReady = false;

// ======================================================
// STATE
// ======================================================
uint8_t mode = PAGE_FLIGHT;
static uint32_t lastDrawMs = 0;
static uint32_t lastBtnMs  = 0;
static bool redraw         = false;   // draw immediately after fresh data or a page change

// Flight page values computed here (from fields)
static bool  aoaValid = false;
static float aoa = 0.0f;
static float q   = 0.0f;

// ======================================================
// UTILS
// ======================================================
static inline bool pressed(uint8_t pin) {
#if BUTTON_ACTIVE_LOW
  return digitalRead(pin) == LOW;
#else
  return digitalRead(pin) == HIGH;
#endif
}

static bool blanked() {
  return in_front[F_TAG][0] == 'X';
}

// The data belongs to the page on screen (the hub re-tags after every page change).
static bool fresh() {
  return in_front[F_TAG][0] == (char)('0' + mode) && in_front[F_TAG][1] == '\0';
}

// Field text, or "" while the data is still for another page.
static const char* fld(uint8_t i) {
  return fresh() ? in_front[i] : "";
}

// Angle between the nose and the surface velocity vector (total angle of attack incl. sideslip).
// Uses the spherical angle, so it stays correct near vertical where heading is meaningless.
static void computeFlight() {
  const float hdg  = atof(in_front[F_HDG]) * DEG_TO_RAD;
  const float pit  = atof(in_front[F_PIT]) * DEG_TO_RAD;
  const float vHdg = atof(in_front[F_SVH]) * DEG_TO_RAD;
  const float vPit = atof(in_front[F_SVP]) * DEG_TO_RAD;
  const float v    = atof(in_front[F_SPD]);
  const float rho  = atof(in_front[F_RHO]);

  // Velocity direction is meaningless when (nearly) stationary.
  aoaValid = (v >= 1.0f);
  if (aoaValid) {
    float c = sinf(pit) * sinf(vPit) + cosf(pit) * cosf(vPit) * cosf(hdg - vHdg);
    c = constrain(c, -1.0f, 1.0f);
    aoa = acosf(c) * RAD_TO_DEG;
  }
  q = (rho > 0.0f) ? 0.5f * rho * v * v / 1000.0f : 0.0f;   // kPa
}

// ======================================================
// DRAW HELPERS
// ======================================================
// Labels are F("...") strings: they stay in flash to save RAM.

static void drawLabel(int x, int y, const __FlashStringHelper* name) {
  char buf[6];
  strncpy_P(buf, (PGM_P)name, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  u8g2.drawStr(x, y, buf);
}

static void drawRight(int xRight, int y, const char* value) {
  u8g2.drawStr(xRight - u8g2.getStrWidth(value), y, value);
}

// Two columns of 8 rows: label left-aligned, value right-aligned.
static int rowY(uint8_t row) {
  return 7 + 8 * row;
}

static void drawInfoLeft(uint8_t row, const __FlashStringHelper* name, const char* value) {
  drawLabel(0, rowY(row), name);
  drawRight(60, rowY(row), value);
}

static void drawInfoRight(uint8_t row, const __FlashStringHelper* name, const char* value) {
  drawLabel(68, rowY(row), name);
  drawRight(127, rowY(row), value);
}

static void drawColumns() {
  u8g2.drawVLine(64, 0, 64);
}

// ======================================================
// PAGE: FLIGHT (AL/VL)
// ======================================================
void displayFlight() {
  char aoaBuf[9] = "";
  char qBuf[9]   = "";
  if (fresh()) {
    if (aoaValid) {
      dtostrf(aoa, 4, 1, aoaBuf);
    } else {
      strcpy(aoaBuf, "---");
    }
    dtostrf(q, 4, 1, qBuf);
  }

  drawColumns();
  drawInfoLeft (0, F("APO"), fld(F_APO));  drawInfoRight(0, F("ALT"), fld(F_ALT));
  drawInfoLeft (1, F("PER"), fld(F_PER));  drawInfoRight(1, F("VRT"), fld(F_VRT));
  drawInfoLeft (2, F("TTA"), fld(F_TTA));  drawInfoRight(2, F("HRZ"), fld(F_HRZ));
  drawInfoLeft (3, F("AOA"), aoaBuf);      drawInfoRight(3, F("G"),   fld(F_G));
  drawInfoLeft (4, F("PIT"), fld(F_PIT));  drawInfoRight(4, F("dVS"), fld(F_DVS));
  drawInfoLeft (5, F("RLL"), fld(F_RLL));  drawInfoRight(5, F("dVT"), fld(F_DVT));
  drawInfoLeft (6, F("HDG"), fld(F_HDG));  drawInfoRight(6, F("MCH"), fld(F_MCH));
  drawInfoLeft (7, F("THR"), fld(F_THR));  drawInfoRight(7, F("Q"),   qBuf);
}

// ======================================================
// PAGE: MANEUVER
// ======================================================
void displayManeuver() {
  drawColumns();
  drawInfoLeft (0, F("NDV"), fld(F_NDV));  drawInfoRight(0, F("APO"), fld(F_APO));
  drawInfoLeft (1, F("TTN"), fld(F_TTN));  drawInfoRight(1, F("PER"), fld(F_PER));
  drawInfoLeft (2, F("TTB"), fld(F_TTB));  drawInfoRight(2, F("TTA"), fld(F_TTA));
  drawInfoLeft (3, F("BRN"), fld(F_BRN));  drawInfoRight(3, F("TTP"), fld(F_TTP));
  drawInfoLeft (4, F("ADV"), fld(F_ADV));  drawInfoRight(4, F("dVS"), fld(F_DVS));
  drawInfoLeft (5, F("NHD"), fld(F_NHD));  drawInfoRight(5, F("dVT"), fld(F_DVT));
  drawInfoLeft (6, F("NPT"), fld(F_NPT));  drawInfoRight(6, F("MRG"), fld(F_MRG));
}

// ======================================================
// PAGE: ORBIT
// ======================================================
void displayOrbit() {
  drawColumns();
  drawInfoLeft (0, F("SOI"), fld(F_SOI));  drawInfoRight(0, F("APO"), fld(F_APO));
  drawInfoLeft (1, F("SMA"), fld(F_SMA));  drawInfoRight(1, F("PER"), fld(F_PER));
  drawInfoLeft (2, F("ECC"), fld(F_ECC));  drawInfoRight(2, F("TTA"), fld(F_TTA));
  drawInfoLeft (3, F("INC"), fld(F_INC));  drawInfoRight(3, F("TTP"), fld(F_TTP));
  drawInfoLeft (4, F("PRD"), fld(F_PRD));  drawInfoRight(4, F("ALT"), fld(F_ALT));
  drawInfoLeft (5, F("LAN"), fld(F_LAN));  drawInfoRight(5, F("VOB"), fld(F_VOB));
  drawInfoLeft (6, F("AOP"), fld(F_AOP));
  drawInfoLeft (7, F("TA"),  fld(F_TA));
}

// ======================================================
// PAGE: FUEL
// ======================================================
// Six vertical gauges, 52 px tall; no bar when the field is "-" (none aboard) or not yet received.
static void drawFuelGauge(uint8_t col, const __FlashStringHelper* name, const char* value) {
  const int x = 16 + 21 * col;            // tick column; bar sits 10 px left of it
  if (value[0] != '\0' && value[0] != '-') {
    int pct = constrain(atoi(value), 0, 100);
    int h = (pct * 52 + 50) / 100;
    u8g2.drawBox(x - 10, 52 - h, 10, h);
  }
  drawLabel(x - 10, 64, name);
  u8g2.drawHLine(x, 0, 2);
  u8g2.drawHLine(x, 12, 2);
  u8g2.drawHLine(x, 25, 2);
  u8g2.drawHLine(x, 37, 2);
  u8g2.drawHLine(x, 51, 2);
  if (col < 5) {
    u8g2.drawVLine(x + 5, 0, 64);
  }
}

void displayFuel() {
  u8g2.drawHLine(2, 54, 124);
  drawFuelGauge(0, F("LF"), fld(F_LF));
  drawFuelGauge(1, F("OX"), fld(F_OX));
  drawFuelGauge(2, F("SF"), fld(F_SF));
  drawFuelGauge(3, F("EC"), fld(F_EC));
  drawFuelGauge(4, F("XE"), fld(F_XE));
  drawFuelGauge(5, F("MP"), fld(F_MP));
}

// ======================================================
// I2C
// ======================================================
void readField(uint8_t index) {
  if (!Wire.available()) {
    in_back[index][0] = '\0';
    return;
  }
  uint8_t len = Wire.read();

  uint8_t i = 0;
  while (Wire.available() && i < len) {
    char c = (char)Wire.read();
    // Keep what fits; still consume the rest so the next field starts at its length byte.
    if (i < FIELD_LEN - 1) in_back[index][i] = c;
    i++;
  }
  in_back[index][(i < FIELD_LEN - 1) ? i : FIELD_LEN - 1] = '\0';
}

void onI2CReceive(int) {
  uint8_t type = Wire.read();
  if (type >= PACKET_COUNT) {
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t base = (uint8_t)(type * FIELDS_PER_PACKET);
  for (uint8_t i = 0; i < FIELDS_PER_PACKET; i++) {
    readField((uint8_t)(base + i));
  }

  // Swap buffers once the last packet of the cycle is in.
  if (type == PACKET_COUNT - 1) {
    packetReady = true;
  }
}

void onI2CRequest() {
  Wire.write(mode);
}

// ======================================================
// SETUP
// ======================================================
void setup() {
#if BUTTON_ACTIVE_LOW
  pinMode(BTN_FLIGHT, INPUT_PULLUP);
  pinMode(BTN_MNVR,   INPUT_PULLUP);
  pinMode(BTN_ORBIT,  INPUT_PULLUP);
  pinMode(BTN_FUEL,   INPUT_PULLUP);
#else
  pinMode(BTN_FLIGHT, INPUT);
  pinMode(BTN_MNVR,   INPUT);
  pinMode(BTN_ORBIT,  INPUT);
  pinMode(BTN_FUEL,   INPUT);
#endif

  u8g2.begin();
  u8g2.setBusClock(4000000UL);

  Wire.begin(OWN_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  // ---------- Button handling ----------
  if (millis() - lastBtnMs > 50) {
    uint8_t newMode = mode;
    if      (pressed(BTN_FLIGHT)) newMode = PAGE_FLIGHT;
    else if (pressed(BTN_MNVR))   newMode = PAGE_MNVR;
    else if (pressed(BTN_ORBIT))  newMode = PAGE_ORBIT;
    else if (pressed(BTN_FUEL))   newMode = PAGE_FUEL;
    if (newMode != mode) {
      mode = newMode;
      lastBtnMs = millis();
      redraw = true;
    }
  }

  // ---------- Data swap ----------
  if (packetReady) {
    noInterrupts();
    memcpy(in_front, in_back, sizeof(in_front));
    packetReady = false;
    interrupts();

    if (mode == PAGE_FLIGHT && fresh()) {
      computeFlight();
    }
    redraw = true;
  }

  // ---------- Draw ----------
  const uint32_t now = millis();
  if (!redraw && (now - lastDrawMs < DRAW_PERIOD_MS)) return;
  lastDrawMs = now;
  redraw = false;

  u8g2.firstPage();
  do {
    if (blanked()) continue;   // not flying: empty screen
    u8g2.setFont(u8g2_font_6x10_tf);
    switch (mode) {
      case PAGE_FLIGHT: displayFlight();   break;
      case PAGE_MNVR:   displayManeuver(); break;
      case PAGE_ORBIT:  displayOrbit();    break;
      case PAGE_FUEL:   displayFuel();     break;
      default: break;
    }
  } while (u8g2.nextPage());
}
