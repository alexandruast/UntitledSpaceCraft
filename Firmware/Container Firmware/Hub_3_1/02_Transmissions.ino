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

// One LCD field: length byte followed by the characters.
void sendPacket(const char* s) {
  uint8_t n = (uint8_t)strnlen(s, 255);
  Wire.write(n);
  Wire.write((const uint8_t*)s, n);
}

//|-------------------------|
//|   Telemetry Formatting  |
//|-------------------------|

// Time until a future event in Kerbin time: mm:ss under 1h, hh:mm up to 96h, then days, then years.
String formatKerbalFutureDelta(uint64_t secondsIntoFuture) {
  uint32_t t = (secondsIntoFuture > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)secondsIntoFuture;

  const uint32_t SEC_PER_MIN  = 60UL;
  const uint32_t SEC_PER_HOUR = 3600UL;
  const uint32_t SEC_PER_DAY  = 21600UL;     // 6h Kerbin day
  const uint32_t SEC_PER_YEAR = 9201600UL;   // 426d Kerbin year

  char buf[11];

  uint32_t totalHours = t / SEC_PER_HOUR;
  uint32_t totalDays  = t / SEC_PER_DAY;

  // > 9999d -> Yy
  if (totalDays > 9999UL) {
    ultoa(t / SEC_PER_YEAR, buf, 10);
    uint8_t n = strlen(buf);
    if (n < 10) { buf[n++] = 'y'; buf[n] = '\0'; }
    return String(buf);
  }

  // > 96h -> Dd
  if (totalHours > 96UL) {
    ultoa(totalDays, buf, 10);
    uint8_t n = strlen(buf);
    if (n < 10) { buf[n++] = 'd'; buf[n] = '\0'; }
    return String(buf);
  }

  // < 1h -> mm:ss
  if (t < SEC_PER_HOUR) {
    uint32_t mm = (t / SEC_PER_MIN) % 60UL;
    uint32_t ss = t % 60UL;
    buf[0] = (char)('0' + (mm / 10));
    buf[1] = (char)('0' + (mm % 10));
    buf[2] = ':';
    buf[3] = (char)('0' + (ss / 10));
    buf[4] = (char)('0' + (ss % 10));
    buf[5] = '\0';
    return String(buf);
  }

  // <= 96h -> hh:mm (total hours)
  uint32_t hh = totalHours;
  uint32_t mm = (t / SEC_PER_MIN) % 60UL;
  buf[0] = (char)('0' + (hh / 10));
  buf[1] = (char)('0' + (hh % 10));
  buf[2] = ':';
  buf[3] = (char)('0' + (mm / 10));
  buf[4] = (char)('0' + (mm % 10));
  buf[5] = '\0';
  return String(buf);
}

// Shorten a whole-number string with a k / M / G / T suffix so it fits the LCD field.
void getUnits(String &value) {
  int n = value.length();
  if (n <= 0) return;

  if (n > 13) {            // -> T
    value.remove(n - 12, 12);
    value += 'T';
    return;
  }
  if (n > 10) {            // -> G
    value.remove(n - 9, 9);
    value += 'G';
    return;
  }
  if (n > 7) {             // -> M
    value.remove(n - 6, 6);
    value += 'M';
    return;
  }
  if (n > 5) {             // -> k
    value.remove(n - 3, 3);
    value += 'k';
    return;
  }
}

// Integer square root for 32-bit values (avoids libm).
uint16_t isqrt_u32(uint32_t x) {
  uint32_t op = x;
  uint32_t res = 0;
  uint32_t one = 1UL << 30;

  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op -= res + one;
      res = res + 2 * one;
    }
    res >>= 1;
    one >>= 2;
  }
  return (uint16_t)res;
}

// Round float to nearest int without pulling lround()/libm.
int32_t round_to_i32(float f) {
  return (int32_t)(f + (f >= 0.0f ? 0.5f : -0.5f));
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
