//|-------------------------------|
//|       System Functions        |
//|-------------------------------|

// Register message channels to Simpit.
void registerMessageChannels() {
  mySimpit.inboundHandler(messageHandler);
  if (Action_Ctrl_Con or Action2_Ctrl_Con or Action3_Ctrl_Con or EA_Groups_Ctrl_Con) {mySimpit.registerChannel(CAGSTATUS_MESSAGE);}
  if (Ctrl_Sys_Ctrl_Con or Rotation_Con or Analog_Con or Analog_Throttle_Con or EA_Control_Ctrl_Con) {mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);}
  if (Ctrl_Sys_Ctrl_Con or EA_Control_Ctrl_Con) {mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);}
  if (EVA_Ctrl_Con || Util_Nav_Ctrl_Con || Util_Time_Ctrl_Con) {mySimpit.registerChannel(EVA_MESSAGE);}
  mySimpit.registerChannel(FLIGHT_STATUS_MESSAGE);
  mySimpit.registerChannel(SCENE_CHANGE_MESSAGE);
}

// Remove every telemetry channel so we can re-register only what we need.
void deregisterAllTelemetryChannels() {
  mySimpit.deregisterChannel(LF_MESSAGE);
  mySimpit.deregisterChannel(OX_MESSAGE);
  mySimpit.deregisterChannel(SF_MESSAGE);
  mySimpit.deregisterChannel(ELECTRIC_MESSAGE);
  mySimpit.deregisterChannel(XENON_GAS_MESSAGE);
  mySimpit.deregisterChannel(MONO_MESSAGE);
  mySimpit.deregisterChannel(APSIDES_MESSAGE);
  mySimpit.deregisterChannel(APSIDESTIME_MESSAGE);
  mySimpit.deregisterChannel(ALTITUDE_MESSAGE);
  mySimpit.deregisterChannel(VELOCITY_MESSAGE);
  mySimpit.deregisterChannel(DELTAV_MESSAGE);
  mySimpit.deregisterChannel(ORBIT_MESSAGE);
  mySimpit.deregisterChannel(MANEUVER_MESSAGE);
  mySimpit.deregisterChannel(AIRSPEED_MESSAGE);
  mySimpit.deregisterChannel(ATMO_CONDITIONS_MESSAGE);
  mySimpit.deregisterChannel(ROTATION_DATA_MESSAGE);
  mySimpit.deregisterChannel(THROTTLE_CMD_MESSAGE);
}

// Register telemetry channels only for the requested LCD mode.
void registerTelemetryChannelsForMode(byte mode) {
  // Matches the custom Telemetry module firmware: only screen 1 shows data,
  // screens 2-4 are placeholders and need no telemetry.
  switch (mode) {
    case 1: // Flight page
      mySimpit.registerChannel(APSIDES_MESSAGE);
      mySimpit.registerChannel(APSIDESTIME_MESSAGE);
      mySimpit.registerChannel(ALTITUDE_MESSAGE);
      mySimpit.registerChannel(VELOCITY_MESSAGE);
      mySimpit.registerChannel(DELTAV_MESSAGE);
      mySimpit.registerChannel(AIRSPEED_MESSAGE);
      mySimpit.registerChannel(ATMO_CONDITIONS_MESSAGE);
      mySimpit.registerChannel(ROTATION_DATA_MESSAGE);
      mySimpit.registerChannel(THROTTLE_CMD_MESSAGE);
      break;
    default: // Placeholder / unknown page
      break;
  }
}

void updateTelemetrySubscriptions(byte mode, bool forceRefresh) {
  if (!forceRefresh && mode == LCD_mode_registered) {
    return;
  }
  deregisterAllTelemetryChannels();
  registerTelemetryChannelsForMode(mode);
  LCD_mode_registered = mode;
}

// Register telemetry channels to Simpit.
void registerTelemetryChannels() {
  updateTelemetrySubscriptions(LCD_mode, true);
}

// Send data to the LCD screen.
// Packet n (0..LCD_PACKET_COUNT-1) carries fields n*4 .. n*4+3; after the last packet, read the screen mode back.
void LCD_transmission(int dspl_address, char LCD_data_register[][LCD_FIELD_SIZE]) {
  if (LCD_transmit < LCD_PACKET_COUNT) {
    Wire.beginTransmission(dspl_address);
    Wire.write((uint8_t)LCD_transmit);
    uint8_t base = (uint8_t)(LCD_transmit * LCD_FIELDS_PER_PACKET);
    for (uint8_t i = 0; i < LCD_FIELDS_PER_PACKET; i++) {
      sendPacket(LCD_data_register[base + i]);
    }
    Wire.endTransmission();
    LCD_transmit++;
  } else {
    Wire.requestFrom(dspl_address, 1);
    byte LCD_mode_incoming = Wire.read();
    LCD_transmit = 0;
    if (LCD_mode_incoming != LCD_mode) {
      LCD_mode = LCD_mode_incoming;
      updateTelemetrySubscriptions(LCD_mode, false);
    }
  }
}

// Set all LEDs and Screen to blank.
void allZero() {
  if (Ctrl_Sys_Ctrl_Con) {
    Wire.beginTransmission(Ctrl_Sys_Dspl_);
    Wire.write(11);
    Wire.endTransmission();
  }
  if (Action_Ctrl_Con) {
    Wire.beginTransmission(Action_Dspl_);
    Wire.write(0);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (Action2_Ctrl_Con) {
    Wire.beginTransmission(Action2_Dspl_);
    Wire.write(0);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (EA_Groups_Ctrl_Con) {
    Wire.beginTransmission(EA_Groups_Dspl_);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (EA_Control_Ctrl_Con) {
    Wire.beginTransmission(EA_Control_Dspl_);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (Time_Ctrl_Con) {
    Wire.beginTransmission(Time_Dspl_);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (EVA_Ctrl_Con || Util_Nav_Ctrl_Con || Util_Time_Ctrl_Con) {
    Wire.beginTransmission(EVA_Dspl_);
    Wire.write(0);
    Wire.endTransmission();
  }
  if (LCD_Con) {
    for (int i = 0; i < LCD_FIELD_COUNT; i++) {
      strcpy(LCD_data[i], "X");
    }
    // One call per data packet, plus one for the mode read-back.
    for (int i = 0; i <= LCD_PACKET_COUNT; i++) {
      LCD_transmission(LCD_, LCD_data);
    }
  }
}

// Keyboard emulation that presses and releases a key in a single event.
static void tapKey(uint8_t key) {
  Keyboard.press(key);
  delay(10);
  Keyboard.release(key);
} 

// Keyboard emulation functions that consist of a simple press action, like pushing a button.
static void setKey(bool state, uint8_t key, bool &flag) {
  if (state && !flag) {
    Keyboard.press(key);
    flag = true;
  }
  if (!state && flag) {
    Keyboard.release(key);
    flag = false;
  }
}

// Keyboard emulation functions that consist of a press and release action, like a toggle switch.
static void setKey_Switch(bool state, uint8_t key, bool &flag) {
  if (state && !flag) {
    tapKey(key);
    flag = true;
  }
  if (!state && flag) {
    tapKey(key);
    flag = false;
  }
}

// Simpit functions that consist if a simple press action, like pushing a button.
static void setSimpitAction(bool state, byte action, bool &flag, void (*onPress)(byte)) {
  if (state && !flag) {
    onPress(action);
    flag = true;
  }
  if (!state && flag) {
    flag = false;
  }
}

// Simpit functions that require a press and release action, such as for a toggle switch.
static void setSimpitAction_Switch(bool state, byte action, bool &flag, void (*onPress)(byte), void (*onRelease)(byte)) {
  if (state && !flag) {
    onPress(action);
    flag = true;
  }
  if (!state && flag) {
    onRelease(action);
    flag = false;
  }
}

// Wrapper so setSimpitAction can call activateAction (member functions can't be passed as pointers).
static void simpitActivateAction(byte action) {
  mySimpit.activateAction(action);
}
// Wrapper so setSimpitAction can call deactivateAction (member functions can't be passed as pointers).
static void simpitDeactivateAction(byte action) {
  mySimpit.deactivateAction(action);
}
// Wrapper so setSimpitAction can call toggleAction (member functions can't be passed as pointers).
static void simpitToggleAction(byte action) {
  mySimpit.toggleAction(action);
}
// Wrapper so setSimpitAction can call setSASMode (member functions can't be passed as pointers).
static void simpitSetSASMode(byte action) {
  mySimpit.setSASMode(action);
}
// Helper function to send a timewarp message to Simpit.
static void simpitSendTimewarp(byte) {
  timewarpToMessage TIMEW(TW_type, TW_delay);
  mySimpit.send(TIMEWARP_TO_MESSAGE, TIMEW);
}
// Wrapper so setSimpitAction can call activateCAG (member functions can't be passed as pointers).
static void simpitActivateCAG(byte action) {
  mySimpit.activateCAG(action);
}
// Wrapper so setSimpitAction can call deactivateCAG (member functions can't be passed as pointers).
static void simpitDeactivateCAG(byte action) {
  mySimpit.deactivateCAG(action);
}
// Wrapper so setSimpitAction can call toggleCAG (member functions can't be passed as pointers).
static void simpitToggleCAG(byte action) {
  mySimpit.toggleCAG(action);
}
// Wrapper so setSimpitAction can call cycleNavBallMode (member functions can't be passed as pointers).
static void simpitCycleNavBallMode(byte action) {
  mySimpit.cycleNavBallMode();
}


// Helper function to send a handshake message to a module.
byte sendHandshake(int address) {
  Wire.requestFrom(address, 1);
  byte connection = Wire.read();
  return connection;
}

// Check if the modules are connected.
void checkConnections() {
  Action_Ctrl_Con = (sendHandshake(Action_Ctrl_)!=255);
  Action2_Ctrl_Con = (sendHandshake(Action2_Ctrl_)!=255);
  Action3_Ctrl_Con = (sendHandshake(Action3_Ctrl_)!=255);
  Ctrl_Sys_Ctrl_Con = (sendHandshake(Ctrl_Sys_Ctrl_)!=255);
  Navigation_Con = (sendHandshake(Navigation_)!=255);
  Time_Ctrl_Con = (sendHandshake(Time_Ctrl_)!=255);
  Stage_Con = (sendHandshake(Stage_)!=255);
  Abort_Con = (sendHandshake(Abort_)!=255);
  Executive_Actions_Con = (sendHandshake(Executive_Actions_)!=255);
  Rotation_Throttle_Con = (sendHandshake(Rotation_Throttle_)!=255);
  Rotation_Con = (sendHandshake(Rotation_)!=255);
  Translation_Con = (sendHandshake(Translation_)!=255);
  Camera_Con = (sendHandshake(Camera_)!=255);
  EVA_Ctrl_Con = (sendHandshake(EVA_Ctrl_)!=255);
  LCD_Con = (sendHandshake(LCD_)!=255);
  Throttle_Con = (sendHandshake(Throttle_)!=255);
  Navigation_Time_Ctrl_Con = (sendHandshake(Navigation_Time_Ctrl_)!=255);
  EA_Groups_Ctrl_Con = (sendHandshake(EA_Groups_Ctrl_)!=255);
  Analog_Con = (sendHandshake(Analog_)!=255);
  Analog_Throttle_Con = (sendHandshake(Analog_Throttle_)!=255);
  EA_Control_Ctrl_Con = (sendHandshake(EA_Control_Ctrl_)!=255);
  Util_Nav_Ctrl_Con = (sendHandshake(Util_Nav_Ctrl_)!=255);
  Util_Time_Ctrl_Con = (sendHandshake(Util_Time_Ctrl_)!=255);
}

void transmissions() {
  /*
    This function is used to handle the transmissions to and from the modules.
    It takes the control address, the control array, the number of input bytes,
    the display address, the display array, and the number of output bytes.
    It then sends the control array to the control address and the display array
    to the display address.

    ctrl_address, ctrl[], in_bytes, dspl_address, dspl[], out_bytes
  */
// --- h1 ---
if (Action_Ctrl_Con) {module_transmission(Action_Ctrl_, Action_Ctrl, TWO_BYTES, Action_Dspl_, Action_Dspl, TWO_BYTES);}
// --- c1 ---
if (LCD_Con) {LCD_transmission(LCD_, LCD_data);}
// --- g1 ---
if (Ctrl_Sys_Ctrl_Con) {module_transmission(Ctrl_Sys_Ctrl_, Ctrl_Sys_Ctrl, TWO_BYTES, Ctrl_Sys_Dspl_, Ctrl_Sys_Dspl, ONE_BYTE);}
// --- b1 ---
if (Time_Ctrl_Con) {module_transmission(Time_Ctrl_, Time_Ctrl, ONE_BYTE, Time_Dspl_, Time_Dspl, ONE_BYTE);}
// --- f3 ---
if (Rotation_Con) {module_transmission(Rotation_, Rotation, 7, NO_DISPLAY, NO_DISPLAY, NO_DISPLAY);}
// --- f5 ---
if (Translation_Con) {module_transmission(Translation_, Translation, 9, NO_DISPLAY, NO_DISPLAY, NO_DISPLAY);}
// --- f6 ---
if (Throttle_Con) {module_transmission(Throttle_, Throttle, 3, NO_DISPLAY, NO_DISPLAY, NO_DISPLAY);}
// --- a3 ---
if (Executive_Actions_Con) {module_transmission(Executive_Actions_, Executive_Actions, ONE_BYTE, NO_DISPLAY, NO_DISPLAY, NO_DISPLAY);}
  
}

// Handle actions in response to changes detected by modules.
void actions() {
// --- h1 ---
if (Action_Ctrl_Con){Action_Action();}
// --- g1 ---
if (Ctrl_Sys_Ctrl_Con){Ctrl_Sys_Action();}
// --- b1 ---
if (Time_Ctrl_Con){Time_Action();}
// --- f3 ---
if (Rotation_Con){Rotation_Action();}
// --- f5 ---
if (Translation_Con){Translation_Action();}
// --- f6 ---
if (Throttle_Con){Throttle_Action();}
// --- a3 ---
if (Executive_Actions_Con){Executive_Actions_Action();}
  
}
