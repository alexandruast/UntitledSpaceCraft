// Transmissions
if (LCD_Con) {LCD_transmission(LCD_, LCD_data);}

// Message Handler
// Field layout matches Firmware/Module Firmware/Telemetry/Telemetry.ino (screen 1):
//  1 HDG   2 PIT   3 surface speed   4 VRT   5 dVT   6 APO   7 surface vel. heading
//  8 dVS   9 surface vel. pitch   10 ALT   11 HRZ   12 TTA   13 air density
//  14 MCH  15 G    16 THR   17 RLL   18 PER
    case APSIDES_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(apsidesMessage)) {
          apsidesMessage myAps;
          myAps = parseMessage<apsidesMessage>(msg);
          String apo = String(myAps.apoapsis);
          int stringLength = apo.length();
          apo.remove(stringLength - 3, 3);
          getUnits(apo);
          LCD_data[6] = apo;
          String peri = String(myAps.periapsis);
          stringLength = peri.length();
          peri.remove(stringLength - 3, 3);
          getUnits(peri);
          LCD_data[18] = peri;
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
          int stringLength = totaldV.length();
          totaldV.remove(stringLength - 3, 3);
          LCD_data[5] = totaldV;
          String stagedV = String(mydV.stageDeltaV);
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
          int stringLength = surface.length();
          surface.remove(stringLength - 3, 3);
          getUnits(surface);
          LCD_data[3] = surface;
          String vertical = String(f_vertical);
          stringLength = vertical.length();
          vertical.remove(stringLength - 3, 3);
          getUnits(vertical);
          LCD_data[4] = vertical;
          // Horizontal speed = sqrt(surface^2 - vertical^2), 64-bit to avoid overflow.
          int32_t s = round_to_i32(f_surface);
          int32_t v = round_to_i32(f_vertical);
          int64_t term = (int64_t)s * (int64_t)s - (int64_t)v * (int64_t)v;
          String horizontal = "0";
          if (term > 0) {
            horizontal = String(isqrt_u32((uint32_t)term));
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
          LCD_data[1] = String(vessel.heading, 1);
          LCD_data[2] = String(vessel.pitch, 1);
          LCD_data[17] = String(vessel.roll, 1);
          LCD_data[7] = String(vessel.surfaceVelocityHeading);
          LCD_data[9] = String(vessel.surfaceVelocityPitch);
        }
      }
      break;

    case THROTTLE_CMD_MESSAGE:
      if (LCD_mode == 1) {
        if (msgSize == sizeof(throttleMessage)) {
          throttleMessage t = parseMessage<throttleMessage>(msg);
          // 0..32767 maps to 0..100%
          int pct = (int)((t.throttle * 100UL) / 32767UL);
          LCD_data[16] = String(pct);
        }
      }
      break;
