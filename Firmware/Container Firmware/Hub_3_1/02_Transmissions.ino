//|-------------------|
//|   Transmissions   |
//|-------------------|

void module_transmission(int ctrl_address, byte ctrl[], int in_bytes, int dspl_address, byte dspl[], int out_bytes) {
  // Control
  if (ctrl_address != 0) {
    Wire.requestFrom(ctrl_address, in_bytes);
    for (int i = 0; i < in_bytes; i++) {
      ctrl[i] = Wire.read();
    }
  }

  // Display
  if ((dspl_address != 0)) {
    Wire.beginTransmission(dspl_address);
    for (int i = 0; i < out_bytes; i++) {
      Wire.write(dspl[i]);
    }
    Wire.endTransmission();
    if (dspl_address == Ctrl_Sys_Dspl_) {
      Wire.beginTransmission(dspl_address);
      for (int i = 0; i < out_bytes; i++) {
        Wire.write(dspl[i]);
      }
      Wire.endTransmission();
    }
  }
}

// One LCD field: length byte followed by the characters (at most LCD_FIELD_MAX_CHARS).
void sendPacket(const char* s) {
  uint8_t n = (uint8_t)strnlen(s, LCD_FIELD_MAX_CHARS);
  Wire.write(n);
  Wire.write((const uint8_t*)s, n);
}

//|-------------------------|
//|   Telemetry Formatting  |
//|-------------------------|

// Copy text into an LCD field, truncating to the field size.
void lcd_set(uint8_t idx, const char* s) {
  strncpy(LCD_data[idx], s, LCD_FIELD_MAX_CHARS);
  LCD_data[idx][LCD_FIELD_MAX_CHARS] = '\0';
}

// Whole-number part of v; past 5 digits it is shortened with k / M / G / T (1234567 -> "1234k").
void lcd_units(uint8_t idx, float v) {
  if (isnan(v) || isinf(v)) {
    lcd_set(idx, "---");
    return;
  }
  char buf[16];
  uint8_t n = 0;
  if (v < 0) {
    buf[n++] = '-';
    v = -v;
  }
  char suffix = 0;
  if (v >= 1e13f)      { v /= 1e12f; suffix = 'T'; }
  else if (v >= 1e10f) { v /= 1e9f;  suffix = 'G'; }
  else if (v >= 1e7f)  { v /= 1e6f;  suffix = 'M'; }
  else if (v >= 1e5f)  { v /= 1e3f;  suffix = 'k'; }
  uint32_t whole = (v >= 4294967040.0f) ? 0xFFFFFFFFUL : (uint32_t)v;
  ultoa(whole, buf + n, 10);
  n = (uint8_t)strlen(buf);
  if (suffix) {
    buf[n++] = suffix;
    buf[n] = '\0';
  }
  lcd_set(idx, buf);
}

// Decimal value; falls back to lcd_units when it would not fit the field.
void lcd_f(uint8_t idx, float v, uint8_t decimals) {
  if (isnan(v) || isinf(v) || fabs(v) >= 10000.0f) {
    lcd_units(idx, v);
    return;
  }
  char buf[16];  // worst case "-9999." + 5 decimals = 11 chars
  dtostrf(v, 0, decimals, buf);
  lcd_set(idx, buf);
}

// Time until a future event in Kerbin time: mm:ss under 1h, hh:mm up to 96h, then days, then years.
// Negative (no future event, e.g. escape trajectory) shows "---".
void lcd_time_until(uint8_t idx, int32_t seconds) {
  if (seconds < 0) {
    lcd_set(idx, "---");
    return;
  }
  uint32_t t = (uint32_t)seconds;

  const uint32_t SEC_PER_MIN  = 60UL;
  const uint32_t SEC_PER_HOUR = 3600UL;
  const uint32_t SEC_PER_DAY  = 21600UL;     // 6h Kerbin day
  const uint32_t SEC_PER_YEAR = 9201600UL;   // 426d Kerbin year

  char buf[12];
  uint32_t totalHours = t / SEC_PER_HOUR;
  uint32_t totalDays  = t / SEC_PER_DAY;

  if (totalDays > 9999UL) {          // > 9999d -> Yy
    ultoa(t / SEC_PER_YEAR, buf, 10);
    strcat(buf, "y");
  } else if (totalHours > 96UL) {    // > 96h -> Dd
    ultoa(totalDays, buf, 10);
    strcat(buf, "d");
  } else {
    // < 1h -> mm:ss, otherwise hh:mm (total hours, up to 96)
    uint32_t a = (t < SEC_PER_HOUR) ? (t / SEC_PER_MIN) % 60UL : totalHours;
    uint32_t b = (t < SEC_PER_HOUR) ? t % 60UL : (t / SEC_PER_MIN) % 60UL;
    buf[0] = (char)('0' + (a / 10));
    buf[1] = (char)('0' + (a % 10));
    buf[2] = ':';
    buf[3] = (char)('0' + (b / 10));
    buf[4] = (char)('0' + (b % 10));
    buf[5] = '\0';
  }
  lcd_set(idx, buf);
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  lastSimpitMessageAt = millis();
  switch(messageType) {
    case ACTIONSTATUS_MESSAGE:
      if (Ctrl_Sys_Ctrl_Con or Rotation_Con or Analog_Con or Analog_Throttle_Con or EA_Control_Ctrl_Con) {
        if ((msgSize == 1) and (isFlying)) {
          currentActionStatus = msg[0];
        } else {
          currentActionStatus = 0;
        }
        bitWrite(EA_Control_Dspl[0], 0, currentActionStatus & LIGHT_ACTION);
        bitWrite(EA_Control_Dspl[0], 2, currentActionStatus & GEAR_ACTION);
        bitWrite(EA_Control_Dspl[0], 4, currentActionStatus & SAS_ACTION);
        bitWrite(EA_Control_Dspl[0], 5, currentActionStatus & RCS_ACTION);
        //Ctrl_Sys_Update = true;
      }
      break;

    case FLIGHT_STATUS_MESSAGE:
      if (msgSize == sizeof(flightStatusMessage)) {
        flightStatusMessage myFlight;
        myFlight = parseFlightStatusMessage(msg);
        if (Translation_Con or EVA_Ctrl_Con or Util_Nav_Ctrl_Con || Util_Time_Ctrl_Con){
          On_EVA = (myFlight.flightStatusFlags & FLIGHT_IS_EVA);
          if (!On_EVA) {
            EVA_Dspl[0] = 0;
          }
        }
        if (Time_Ctrl_Con or Navigation_Time_Ctrl_Con){
          if (myFlight.currentTWIndex > 0) {
            Time_Dspl[0] = 1;
          } else {
            Time_Dspl[0] = 0;
          }
        }
      }
      break;
  
    case SCENE_CHANGE_MESSAGE:
      isFlying = !msg[0];
      mySimpit.deregisterChannel(CAGSTATUS_MESSAGE);
      mySimpit.registerChannel(CAGSTATUS_MESSAGE);
      mySimpit.deregisterChannel(SAS_MODE_INFO_MESSAGE);
      mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
      mySimpit.deregisterChannel(ACTIONSTATUS_MESSAGE);
      mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
      break;
  }
}
