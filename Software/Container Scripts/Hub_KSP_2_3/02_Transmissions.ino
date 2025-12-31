//|-------------------|
//|   Transmissions   |
//|-------------------|

void module_transmission(uint8_t ctrl_address,
                         uint8_t dspl_address,
                         uint8_t in_bytes,
                         uint8_t out_bytes,
                         byte ctrl[],
                         byte dspl[],
                         bool &UpdateFlag) {
  if (ctrl_address != 0) {
    Wire.requestFrom(ctrl_address, in_bytes);
    for (uint8_t i = 0; i < in_bytes; i++) {
      ctrl[i] = Wire.read();
    }
  }

  if (dspl_address != 0) { // and (UpdateFlag)
    Wire.beginTransmission(dspl_address);
    for (uint8_t i = 0; i < out_bytes; i++) {
      Wire.write(dspl[i]);
    }
    Wire.endTransmission();
    UpdateFlag = false;
  }
}

void LCD_transmission(uint8_t dspl_address, String LCD_data_register[]) {
  if (LCD_transmit <= 4) {
    Wire.beginTransmission(dspl_address);
    Wire.write((uint8_t)LCD_transmit);

    uint8_t base = (uint8_t)(LCD_transmit * 4);
    for (uint8_t i = 0; i < 4; i++) {
      sendPacket(LCD_data_register[base + i].c_str());
    }

    Wire.endTransmission();
    LCD_transmit++;
  } else {
    Wire.requestFrom(dspl_address, (uint8_t)1);
    uint8_t LCD_mode_incoming = Wire.read();

    LCD_transmit = 0;
    if (LCD_mode_incoming != LCD_mode) {
      LCD_mode = LCD_mode_incoming;
      mySimpit.requestMessageOnChannel(0);
    }
  }
}

static void sendPacket(const char* s) {
  uint8_t n = (uint8_t)strnlen(s, 255);
  Wire.write(n);
  Wire.write((const uint8_t*)s, n);
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  int stringLength;
  switch(messageType) {
    case EVA_MESSAGE:
      if (EVA_Ctrl_Con || Util_Nav_Ctrl_Con || Util_Time_Ctrl_Con) {
        if (msgSize == sizeof(resourceMessage)) {
          resourceMessage myEVA;
          myEVA = parseResource(msg);
          if (myEVA.available) {
            int myEVA_available = myEVA.available * 100.00;
            int myEVA_total = myEVA.total * 100.00;
            EVA_Dspl[0] = map(myEVA_available, 0, myEVA_total, 1, 10);
            if (myEVA_available==0) {
              EVA_Dspl[0] = 0;
            }
          }
        }
        EVA_Update = true;
      }
      break;
      
    case CAGSTATUS_MESSAGE:
      if (Action_Ctrl_Con or Action2_Ctrl_Con or Action3_Ctrl_Con or EA_Groups_Ctrl_Con) {
        if (msgSize == sizeof(cagStatusMessage)) {
          cagStatusMessage myAG;
          myAG = parseCAGStatusMessage(msg);
          if (Action_Ctrl_Con) {
            bool Action_Dspl_b[10] = {};
            for (int i = 0; i < 10; i++) {
              Action_Dspl_b[i] = (myAG.is_action_activated(AG_List[i]));
            }
            for (int i = 0; i < 8; i++) {
              bitWrite(Action_Dspl[0], i, Action_Dspl_b[i]);
            }
            for (int i = 0; i < 2; i++) {
              bitWrite(Action_Dspl[1], i, Action_Dspl_b[i+8]);
            }
            AG_Update = true;
          }
          if (Action2_Ctrl_Con) {
            bool Action2_Dspl_b[10] = {};
            for (int i = 0; i < 10; i++) {
              Action2_Dspl_b[i] = (myAG.is_action_activated(AG2_List[i]));
            }
            for (int i = 0; i < 8; i++) {
              bitWrite(Action2_Dspl[0], i, Action2_Dspl_b[i]);
            }
            for (int i = 0; i < 2; i++) {
              bitWrite(Action2_Dspl[1], i, Action2_Dspl_b[i+8]);
            }
            AG2_Update = true;
          }
          if (Action3_Ctrl_Con) {
            bool Action3_Dspl_b[10] = {};
            for (int i = 0; i < 10; i++) {
              Action3_Dspl_b[i] = (myAG.is_action_activated(AG3_List[i]));
            }
            for (int i = 0; i < 8; i++) {
              bitWrite(Action3_Dspl[0], i, Action3_Dspl_b[i]);
            }
            for (int i = 0; i < 2; i++) {
              bitWrite(Action3_Dspl[1], i, Action3_Dspl_b[i+8]);
            }
            AG3_Update = true;
          }
          if (EA_Groups_Ctrl_Con) {
            bool EA_Groups_Dspl_b[8] = {0,0,0,0,0,0,0,0};
            for (int i = 0; i < 8; i++) {
              EA_Groups_Dspl_b[i] = (myAG.is_action_activated(i+1));
            }
            for (int i = 0; i < 8; i++) {
              bitWrite(EA_Groups_Dspl[0], i, EA_Groups_Dspl_b[i]);
            }
          }
        }
      }
      break;
      
    case SAS_MODE_INFO_MESSAGE:
      if (Ctrl_Sys_Ctrl_Con or EA_Control_Ctrl_Con) {
        if (msgSize == sizeof(SASInfoMessage)) {
          SASInfoMessage mySAS;
          mySAS = parseSASInfoMessage(msg);
          bool stabAssist = (mySAS.currentSASMode==AP_STABILITYASSIST);
          bitWrite(EA_Control_Dspl[0], 3, stabAssist);
          if (ControlSys_combinedBytes & 1024) {
            Ctrl_Sys_Dspl[0] = mySAS.currentSASMode;
          } else {
            Ctrl_Sys_Dspl[0] = 11;
          }
        }
        Ctrl_Sys_Update = true;
      }
      break;

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
            EVA_Update = true;
          }
        }
        if (Time_Ctrl_Con or Navigation_Time_Ctrl_Con){
          if (myFlight.currentTWIndex > 0) {
            Time_Dspl[0] = 1;
          } else {
            Time_Dspl[0] = 0;
          }
          TW_Update = true;
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

    case APSIDES_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(apsidesMessage)) {
          apsidesMessage myAps;
          myAps = parseMessage<apsidesMessage>(msg);
          String a;
          int stringLength;
          a = String(myAps.apoapsis);
          stringLength = a.length();
          a.remove(stringLength - 3, 3);
          getUnits(a);
          LCD_data[6] = a;
          a = String(myAps.periapsis);
          stringLength = a.length();
          a.remove(stringLength - 3, 3);
          getUnits(a);
          LCD_data[18] = a;
        }
      }
      break;

    case APSIDESTIME_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(apsidesTimeMessage)) {
          apsidesTimeMessage myAps;
          myAps = parseMessage<apsidesTimeMessage>(msg);
          LCD_data[12] = formatKerbalFutureDelta(myAps.apoapsis);
        }
      }
      break;

    case DELTAV_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(deltaVMessage)) {
          deltaVMessage mydV;
          mydV = parseMessage<deltaVMessage>(msg);
          String totaldV = String(mydV.totalDeltaV);
          String stagedV = String(mydV.stageDeltaV);
          int stringLength = 0;
          stringLength = totaldV.length();
          totaldV.remove(stringLength - 3, 3);
          LCD_data[5] = totaldV;
          stringLength = stagedV.length();
          stagedV.remove(stringLength - 3, 3);
          LCD_data[8] = stagedV;
        }
      }
      break;

    case ALTITUDE_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(altitudeMessage)) {
          altitudeMessage myAlt;
          myAlt = parseMessage<altitudeMessage>(msg);
          String sealevel = String(myAlt.sealevel);
          String surface = String(myAlt.surface);
          int stringLength = sealevel.length();
          sealevel.remove(stringLength - 3, 3);
          getUnits(sealevel);
          LCD_data[10] = sealevel;
        }
      }
      break;

    case VELOCITY_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(velocityMessage)) {
          velocityMessage myVel;
          myVel = parseMessage<velocityMessage>(msg);
          float f_surface = myVel.surface;
          float f_vertical = myVel.vertical;
          String surface = String(f_surface);
          String vertical = String(f_vertical);
          stringLength = surface.length();
          surface.remove(stringLength - 3, 3);
          getUnits(surface);
          LCD_data[3] = surface;
          stringLength = vertical.length();
          vertical.remove(stringLength - 3, 3);
          getUnits(vertical);
          LCD_data[4] = vertical;
          int32_t s = round_to_i32(f_surface);
          int32_t v = round_to_i32(f_vertical);
          // term = s^2 - v^2 (use 64-bit to avoid overflow)
          int64_t term = (int64_t)s * (int64_t)s - (int64_t)v * (int64_t)v;
          String horizontal = "0";
          if (term > 0) {
            uint32_t uterm = (uint32_t)term;
            uint16_t h = isqrt_u32(uterm);
            horizontal = String(h);
          }
          getUnits(horizontal);
          LCD_data[11] = horizontal;
        }
      }
      break;

    case AIRSPEED_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(airspeedMessage)) {
          airspeedMessage air = parseMessage<airspeedMessage>(msg);
          LCD_data[14] = String(air.mach);
          LCD_data[15] = String(air.gForces);
        }
      }
      break;
    
    case ATMO_CONDITIONS_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(atmoConditionsMessage)) {
          atmoConditionsMessage atmo = parseMessage<atmoConditionsMessage>(msg);
          LCD_data[13] = String(atmo.airDensity);
        }
      }
      break;
    
    case ROTATION_DATA_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(vesselPointingMessage)) {
          vesselPointingMessage vessel = parseMessage<vesselPointingMessage>(msg);
          LCD_data[1] = String(vessel.heading,1);
          LCD_data[2] = String(vessel.pitch,1);
          LCD_data[17] = String(vessel.roll,1);
          LCD_data[7] = String(vessel.surfaceVelocityHeading);
          LCD_data[9] = String(vessel.surfaceVelocityPitch);
        }
      }
      break;


    case THROTTLE_CMD_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(throttleMessage)) {
          throttleMessage t = parseMessage<throttleMessage>(msg);
          // t.throttle is a 0–65535 value (0–100%); scale as needed
          uint16_t raw = t.throttle;
          int pct = (int)((raw * 100UL) / 32767UL);
          LCD_data[16] = String(pct);
        }
      }
      break;
  }
}


// ----------------------- Time formatting -----------------------
String formatSeconds(const String& secondsChar) {
  uint32_t s = (uint32_t)strtoul(secondsChar.c_str(), NULL, 10);

  // >= 1 Kerbin day -> "Xd"
  if (s >= 21600UL) {
    uint32_t d = s / 21600UL;
    char buf[11];
    ultoa(d, buf, 10);
    uint8_t n = strlen(buf);
    if (n < 10) { buf[n++] = 'd'; buf[n] = '\0'; }
    return String(buf);
  }

  uint32_t h = s / 3600UL;
  uint32_t m = (s % 3600UL) / 60UL;
  uint32_t sec = s % 60UL;

  char buf[11];
  // h:mm:ss (hours can be 0..5 here)
  buf[0] = (char)('0' + (h % 10));
  buf[1] = ':';
  buf[2] = (char)('0' + (m / 10));
  buf[3] = (char)('0' + (m % 10));
  buf[4] = ':';
  buf[5] = (char)('0' + (sec / 10));
  buf[6] = (char)('0' + (sec % 10));
  buf[7] = '\0';
  return String(buf);
}

String formatKerbalFutureDelta(uint64_t secondsIntoFuture) {
  // keep API, but compute in 32-bit for smaller code
  uint32_t t = (secondsIntoFuture > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)secondsIntoFuture;

  // constants locally (often smaller than global 64-bit consts)
  const uint32_t SEC_PER_MIN  = 60UL;
  const uint32_t SEC_PER_HOUR = 3600UL;
  const uint32_t SEC_PER_DAY  = 21600UL;     // 6h
  const uint32_t SEC_PER_YEAR = 9201600UL;   // 426d

  char buf[11];

  uint32_t totalHours = t / SEC_PER_HOUR;
  uint32_t totalDays  = t / SEC_PER_DAY;

  // > 9999d -> Yy
  if (totalDays > 9999UL) {
    uint32_t yy = t / SEC_PER_YEAR;
    ultoa(yy, buf, 10);
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
  uint32_t hh = totalHours;                 // 0..96
  uint32_t mm = (t / SEC_PER_MIN) % 60UL;

  buf[0] = (char)('0' + (hh / 10));
  buf[1] = (char)('0' + (hh % 10));
  buf[2] = ':';
  buf[3] = (char)('0' + (mm / 10));
  buf[4] = (char)('0' + (mm % 10));
  buf[5] = '\0';
  return String(buf);
}


// ----------------------- Units -----------------------
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

// Fast integer sqrt for 32-bit values (no libm)
static uint16_t isqrt_u32(uint32_t x) {
  uint32_t op = x;
  uint32_t res = 0;
  uint32_t one = 1UL << 30;              // second-to-top bit

  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op -= res + one;
      res = res + 2 * one;
    }
    res >>= 1;
    one >>= 2;
  }
  return (uint16_t)res;                  // result fits for typical speeds
}

// Round float to nearest int without pulling lround()/libm
static int32_t round_to_i32(float f) {
  return (int32_t)(f + (f >= 0.0f ? 0.5f : -0.5f));
}
