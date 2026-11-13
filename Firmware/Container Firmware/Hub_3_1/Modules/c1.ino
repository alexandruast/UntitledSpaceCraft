// Transmissions
if (LCD_Con) {LCD_transmission(LCD_, LCD_data);}

// Message Handler
// Field layout matches Firmware/Module Firmware/Telemetry/Telemetry.ino (screen 1):
//  1 HDG   2 PIT   3 surface speed   4 VRT   5 dVT   6 APO   7 surface vel. heading
//  8 dVS   9 surface vel. pitch   10 ALT   11 HRZ   12 TTA   13 air density
//  14 MCH  15 G    16 THR   17 RLL   18 PER
    case APSIDES_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(apsidesMessage)) {
        apsidesMessage myAps = parseMessage<apsidesMessage>(msg);
        lcd_units(6, myAps.apoapsis);
        lcd_units(18, myAps.periapsis);
      }
      break;

    case APSIDESTIME_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(apsidesTimeMessage)) {
        apsidesTimeMessage myAps = parseMessage<apsidesTimeMessage>(msg);
        lcd_time_until(12, myAps.apoapsis);
      }
      break;

    case DELTAV_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(deltaVMessage)) {
        deltaVMessage mydV = parseMessage<deltaVMessage>(msg);
        lcd_units(5, mydV.totalDeltaV);
        lcd_units(8, mydV.stageDeltaV);
      }
      break;

    case ALTITUDE_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(altitudeMessage)) {
        altitudeMessage myAlt = parseMessage<altitudeMessage>(msg);
        lcd_units(10, myAlt.sealevel);
      }
      break;

    case VELOCITY_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(velocityMessage)) {
        velocityMessage myVel = parseMessage<velocityMessage>(msg);
        lcd_units(3, myVel.surface);
        lcd_units(4, myVel.vertical);
        // Horizontal speed = sqrt(surface^2 - vertical^2)
        float term = myVel.surface * myVel.surface - myVel.vertical * myVel.vertical;
        lcd_units(11, (term > 0.0f) ? sqrtf(term) : 0.0f);
      }
      break;

    case AIRSPEED_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(airspeedMessage)) {
        airspeedMessage air = parseMessage<airspeedMessage>(msg);
        lcd_f(14, air.mach, 2);
        lcd_f(15, air.gForces, 2);
      }
      break;

    case ATMO_CONDITIONS_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(atmoConditionsMessage)) {
        atmoConditionsMessage atmo = parseMessage<atmoConditionsMessage>(msg);
        // 5 decimals: the module computes dynamic pressure from this, and upper-atmosphere densities are < 0.01.
        lcd_f(13, atmo.airDensity, 5);
      }
      break;

    case ROTATION_DATA_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(vesselPointingMessage)) {
        vesselPointingMessage vessel = parseMessage<vesselPointingMessage>(msg);
        lcd_f(1, vessel.heading, 1);
        lcd_f(2, vessel.pitch, 1);
        lcd_f(17, vessel.roll, 1);
        lcd_f(7, vessel.surfaceVelocityHeading, 2);
        lcd_f(9, vessel.surfaceVelocityPitch, 2);
      }
      break;

    case THROTTLE_CMD_MESSAGE:
      if (LCD_mode == 1 && msgSize == sizeof(throttleMessage)) {
        throttleMessage t = parseMessage<throttleMessage>(msg);
        // 0..32767 maps to 0..100%
        int32_t raw = constrain((int32_t)t.throttle, 0L, 32767L);
        lcd_units(16, (float)((raw * 100L) / 32767L));
      }
      break;
