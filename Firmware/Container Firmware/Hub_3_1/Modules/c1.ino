// Transmissions
if (LCD_Con) {LCD_syncTag(); LCD_transmission(LCD_, LCD_data);}

// Message Handler
// Fields and pages: see LCD_F_* / LCD_PAGE_* in definitions.h and
// Firmware/Module Firmware/Telemetry/Telemetry.ino. Page-specific writes check LCD_mode so a
// late message from the previous page can't land in the new page's fields.
    case APSIDES_MESSAGE:
      if (msgSize == sizeof(apsidesMessage)) {
        apsidesMessage myAps = parseMessage<apsidesMessage>(msg);
        lcd_units(LCD_F_APO, myAps.apoapsis);
        lcd_units(LCD_F_PER, myAps.periapsis);
      }
      break;

    case APSIDESTIME_MESSAGE:
      if (msgSize == sizeof(apsidesTimeMessage)) {
        apsidesTimeMessage myAps = parseMessage<apsidesTimeMessage>(msg);
        lcd_time_until(LCD_F_TTA, myAps.apoapsis);
        lcd_time_until(LCD_F_TTP, myAps.periapsis);
      }
      break;

    case DELTAV_MESSAGE:
      if (msgSize == sizeof(deltaVMessage)) {
        deltaVMessage mydV = parseMessage<deltaVMessage>(msg);
        lcd_units(LCD_F_DVT, mydV.totalDeltaV);
        lcd_units(LCD_F_DVS, mydV.stageDeltaV);
        LCD_ship_dV = mydV.totalDeltaV;
        if (LCD_mode == LCD_PAGE_MNVR) {
          lcd_units(LCD_F_MRG, LCD_ship_dV - LCD_nodes_dV);
        }
      }
      break;

    case ALTITUDE_MESSAGE:
      if (msgSize == sizeof(altitudeMessage)) {
        altitudeMessage myAlt = parseMessage<altitudeMessage>(msg);
        lcd_units(LCD_F_ALT, myAlt.sealevel);
      }
      break;

    case VELOCITY_MESSAGE:
      if (msgSize == sizeof(velocityMessage)) {
        velocityMessage myVel = parseMessage<velocityMessage>(msg);
        if (LCD_mode == LCD_PAGE_FLIGHT) {
          lcd_units(LCD_F_SPD, myVel.surface);
          lcd_units(LCD_F_VRT, myVel.vertical);
          // Horizontal speed = sqrt(surface^2 - vertical^2)
          float term = myVel.surface * myVel.surface - myVel.vertical * myVel.vertical;
          lcd_units(LCD_F_HRZ, (term > 0.0f) ? sqrtf(term) : 0.0f);
        } else if (LCD_mode == LCD_PAGE_ORBIT) {
          lcd_units(LCD_F_VOB, myVel.orbital);
        }
      }
      break;

    case AIRSPEED_MESSAGE:
      if (LCD_mode == LCD_PAGE_FLIGHT && msgSize == sizeof(airspeedMessage)) {
        airspeedMessage air = parseMessage<airspeedMessage>(msg);
        lcd_f(LCD_F_MCH, air.mach, 2);
        lcd_f(LCD_F_G, air.gForces, 2);
      }
      break;

    case ATMO_CONDITIONS_MESSAGE:
      if (LCD_mode == LCD_PAGE_FLIGHT && msgSize == sizeof(atmoConditionsMessage)) {
        atmoConditionsMessage atmo = parseMessage<atmoConditionsMessage>(msg);
        // 5 decimals: the module computes dynamic pressure from this, and upper-atmosphere densities are < 0.01.
        lcd_f(LCD_F_RHO, atmo.airDensity, 5);
      }
      break;

    case ROTATION_DATA_MESSAGE:
      if (LCD_mode == LCD_PAGE_FLIGHT && msgSize == sizeof(vesselPointingMessage)) {
        vesselPointingMessage vessel = parseMessage<vesselPointingMessage>(msg);
        lcd_f(LCD_F_HDG, vessel.heading, 1);
        lcd_f(LCD_F_PIT, vessel.pitch, 1);
        lcd_f(LCD_F_RLL, vessel.roll, 1);
        lcd_f(LCD_F_SVH, vessel.surfaceVelocityHeading, 2);
        lcd_f(LCD_F_SVP, vessel.surfaceVelocityPitch, 2);
      }
      break;

    case THROTTLE_CMD_MESSAGE:
      if (LCD_mode == LCD_PAGE_FLIGHT && msgSize == sizeof(throttleMessage)) {
        throttleMessage t = parseMessage<throttleMessage>(msg);
        // 0..32767 maps to 0..100%
        int32_t raw = constrain((int32_t)t.throttle, 0L, 32767L);
        lcd_units(LCD_F_THR, (float)((raw * 100L) / 32767L));
      }
      break;

    case MANEUVER_MESSAGE:
      if (LCD_mode == LCD_PAGE_MNVR && msgSize == sizeof(maneuverMessage)) {
        maneuverMessage myMnv = parseMessage<maneuverMessage>(msg);
        LCD_nodes_dV = myMnv.deltaVTotal;
        lcd_units(LCD_F_ADV, myMnv.deltaVTotal);
        lcd_units(LCD_F_MRG, LCD_ship_dV - LCD_nodes_dV);
        if (myMnv.deltaVTotal > 0.0f) {
          lcd_units(LCD_F_NDV, myMnv.deltaVNextManeuver);
          lcd_time_until(LCD_F_TTN, myMnv.timeToNextManeuver);
          float toStart = myMnv.timeToNextManeuver - myMnv.durationNextManeuver / 2.0f;
          if (toStart < 0.0f && myMnv.timeToNextManeuver >= 0.0f) {
            lcd_set(LCD_F_TTB, "NOW");
          } else {
            lcd_time_until(LCD_F_TTB, toStart);
          }
          lcd_time_until(LCD_F_BRN, myMnv.durationNextManeuver);
          lcd_f(LCD_F_NHD, myMnv.headingNextManeuver, 1);
          lcd_f(LCD_F_NPT, myMnv.pitchNextManeuver, 1);
        } else {
          // No maneuver node planned.
          lcd_set(LCD_F_NDV, "---");
          lcd_set(LCD_F_TTN, "---");
          lcd_set(LCD_F_TTB, "---");
          lcd_set(LCD_F_BRN, "---");
          lcd_set(LCD_F_NHD, "---");
          lcd_set(LCD_F_NPT, "---");
        }
      }
      break;

    case ORBIT_MESSAGE:
      if (LCD_mode == LCD_PAGE_ORBIT && msgSize == sizeof(orbitInfoMessage)) {
        orbitInfoMessage myOrb = parseMessage<orbitInfoMessage>(msg);
        lcd_units(LCD_F_SMA, myOrb.semiMajorAxis);
        lcd_f(LCD_F_ECC, myOrb.eccentricity, 4);
        lcd_f(LCD_F_INC, myOrb.inclination, 2);
        lcd_time_until(LCD_F_PRD, myOrb.period);
        lcd_f(LCD_F_LAN, myOrb.longAscendingNode, 1);
        lcd_f(LCD_F_AOP, myOrb.argPeriapsis, 1);
        lcd_f(LCD_F_TA, myOrb.trueAnomaly, 1);
      }
      break;

    case SOI_MESSAGE: {
      uint8_t n = (msgSize < LCD_FIELD_MAX_CHARS) ? msgSize : LCD_FIELD_MAX_CHARS;
      memcpy(LCD_soi, msg, n);
      LCD_soi[n] = '\0';
      if (LCD_mode == LCD_PAGE_ORBIT) {
        lcd_set(LCD_F_SOI, LCD_soi);
      }
      break;
    }

    case LF_MESSAGE:
    case OX_MESSAGE:
    case SF_MESSAGE:
    case ELECTRIC_MESSAGE:
    case XENON_GAS_MESSAGE:
    case MONO_MESSAGE:
      if (LCD_mode == LCD_PAGE_FUEL && msgSize == sizeof(resourceMessage)) {
        uint8_t idx;
        switch (messageType) {
          case LF_MESSAGE:       idx = LCD_F_LF; break;
          case OX_MESSAGE:       idx = LCD_F_OX; break;
          case SF_MESSAGE:       idx = LCD_F_SF; break;
          case ELECTRIC_MESSAGE: idx = LCD_F_EC; break;
          case XENON_GAS_MESSAGE: idx = LCD_F_XE; break;
          default:               idx = LCD_F_MP; break;
        }
        lcd_resource(idx, msg);
      }
      break;
