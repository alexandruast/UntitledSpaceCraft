#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
#endif

// ======================================================
// CONFIG
// ======================================================
#define BUTTON_ACTIVE_LOW 0

#define OWN_ADDRESS 40

#define BTN_T1 9
#define BTN_T2 8
#define BTN_T3 7
#define BTN_T4 6

#define FIELD_COUNT 20
#define FIELD_LEN   9   // 8 chars + null

#define DRAW_PERIOD_MS 250  // 4 Hz fallback redraw

// ======================================================
// DISPLAY
// ======================================================
// U8G2_ST7920_128X64_1_SW_SPI u8g2(
//   U8G2_R2,
//   /* clock=*/ 11,
//   /* data=*/ 13,
//   /* CS=*/   10,
//   /* reset=*/ U8X8_PIN_NONE
// );

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

// Only parse floats you need (Screen 1 currently uses these)
static float f[FIELD_COUNT];

static volatile bool packetReady = false;
static volatile uint8_t lastPacketType = 0;

// ======================================================
// STATE
// ======================================================
uint8_t mode = 1;
static uint32_t lastDrawMs = 0;
static uint32_t lastBtnMs  = 0;
static bool dataUpdated    = false;   // draw immediately after fresh data

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

float wrap180(float x) {
  x = fmodf(x + 180.0f, 360.0f);
  if (x < 0) x += 360.0f;
  return x - 180.0f;
}

// ======================================================
// DRAW HELPERS
// ======================================================
void drawInfoLeft(const char* name, int y, const char* value) {
  u8g2.drawStr(0, y, name);
  int w = u8g2.getStrWidth(value);
  u8g2.drawStr(60 - w, y, value);
}

void drawInfoRight(const char* name, int y, const char* value) {
  u8g2.drawStr(68, y, name);
  int w = u8g2.getStrWidth(value);
  u8g2.drawStr(127 - w, y, value);
}

// ======================================================
// TELEMETRY SCREEN 1
// ======================================================
void displayTelemetry1() {
  const float rho   = f[13];
  const float v     = f[3];
  const float pit   = f[2];
  const float hdg   = f[1];
  const float vPit  = f[9];
  const float vHdg  = f[7];

  float q = 0.0f;
  if (rho > 0.0f) {
    q = 0.5f * rho * v * v / 1000.0f;
  }

  const float dP = pit - vPit;
  const float dH = wrap180(hdg - vHdg);
  const float aoa = sqrtf(dP * dP + dH * dH);

  u8g2.drawVLine(64, 0, 64);

  // Row 1
  drawInfoLeft ("APO", 7,  in_front[6]);
  drawInfoRight("ALT", 7,  in_front[10]);

  // Row 2
  drawInfoLeft ("PER", 15, in_front[18]);
  drawInfoRight("VRT", 15, in_front[4]);

  // Row 3
  drawInfoLeft ("TTA", 23, in_front[12]);
  drawInfoRight("HRZ", 23, in_front[11]);

  // Row 4
  char buf[9];
  dtostrf(aoa, 4, 1, buf);
  drawInfoLeft ("AOA", 31, buf);
  drawInfoRight("G",   31, in_front[15]);

  // Row 5
  drawInfoLeft ("PIT", 39, in_front[2]);
  drawInfoRight("dVS", 39, in_front[8]);

  // Row 6
  drawInfoLeft ("RLL", 47, in_front[17]);
  drawInfoRight("dVT", 47, in_front[5]);

  // Row 7
  drawInfoLeft ("HDG", 55, in_front[1]);
  drawInfoRight("MCH", 55, in_front[14]);

  // Row 8
  dtostrf(q, 4, 1, buf);
  drawInfoLeft ("THR", 63, in_front[16]);
  drawInfoRight("Q",   63, buf);
}

// ======================================================
// OTHER SCREENS
// ======================================================
void displayTelemetry2() {
  u8g2.drawStr(0, 12, "SCREEN 2");
  u8g2.drawStr(0, 28, "Orbit / Planning");
}

void displayTelemetry3() {
  u8g2.drawStr(0, 12, "SCREEN 3");
  u8g2.drawStr(0, 28, "Cruise / Transfer");
}

void displayTelemetry4() {
  u8g2.drawStr(0, 12, "SCREEN 4");
  u8g2.drawStr(0, 28, "Landing / Dock");
}

// ======================================================
// I2C
// ======================================================
void readField(uint8_t index) {
  uint8_t len = Wire.read();
  if (len > FIELD_LEN - 1) len = FIELD_LEN - 1;

  uint8_t i = 0;
  while (Wire.available() && i < len) {
    in_back[index][i++] = (char)Wire.read();
  }
  in_back[index][i] = '\0';
}

void onI2CReceive(int) {
  uint8_t type = Wire.read();
  lastPacketType = type;

  uint8_t base = (uint8_t)(type * 4);
  for (uint8_t i = 0; i < 4; i++) {
    readField((uint8_t)(base + i));
  }

  if (type == 3) {
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
  pinMode(BTN_T1, INPUT_PULLUP);
  pinMode(BTN_T2, INPUT_PULLUP);
  pinMode(BTN_T3, INPUT_PULLUP);
  pinMode(BTN_T4, INPUT_PULLUP);
#else
  pinMode(BTN_T1, INPUT);
  pinMode(BTN_T2, INPUT);
  pinMode(BTN_T3, INPUT);
  pinMode(BTN_T4, INPUT);
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
    if      (pressed(BTN_T1)) { mode = 1; lastBtnMs = millis(); }
    else if (pressed(BTN_T2)) { mode = 2; lastBtnMs = millis(); }
    else if (pressed(BTN_T3)) { mode = 3; lastBtnMs = millis(); }
    else if (pressed(BTN_T4)) { mode = 4; lastBtnMs = millis(); }
  }

  // ---------- Data swap ----------
  if (packetReady) {
    noInterrupts();
    memcpy(in_front, in_back, sizeof(in_front));
    packetReady = false;
    interrupts();

    // parse only what Screen 1 needs (fast)
    f[1]  = atof(in_front[1]);   // heading
    f[2]  = atof(in_front[2]);   // pitch
    f[3]  = atof(in_front[3]);   // surface speed
    f[7]  = atof(in_front[7]);   // surface velocity heading
    f[9]  = atof(in_front[9]);   // surface velocity pitch
    f[13] = atof(in_front[13]);  // air density

    dataUpdated = true;
  }

  // ---------- Draw ----------
  const uint32_t now = millis();
  if (!dataUpdated && (now - lastDrawMs < DRAW_PERIOD_MS)) return;
  lastDrawMs = now;
  dataUpdated = false;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    switch (mode) {
      case 1: displayTelemetry1(); break;
      case 2: displayTelemetry2(); break;
      case 3: displayTelemetry3(); break;
      case 4: displayTelemetry4(); break;
      default: break;
    }
  } while (u8g2.nextPage());
}

// Test
