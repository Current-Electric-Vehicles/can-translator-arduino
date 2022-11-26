#include <Arduino.h>
#include <mcp_can.h>
#include <AsyncTimer.h>

#include "LedManager.h"
#include "aem_vcu.h"
#include "Chrysler_300c.h"

#define DEBUG_ENABLED true

#define CAN1_CHIP_SELECT_PIN 3
#define CAN1_LISTEN_ENABLED true
#define CAN1_EMIT_ENABLED false
#define CAN1_BAUD_RATE CAN_500KBPS
#define CAN1_EMIT_FREQUENCE_MILLIS 20

#define CAN2_CHIP_SELECT_PIN 4
#define CAN2_LISTEN_ENABLED false
#define CAN2_EMIT_ENABLED true
#define CAN2_BAUD_RATE CAN_500KBPS
#define CAN2_EMIT_FREQUENCE_MILLIS 20

#define EV_MOTOR_MAX_RPM 12000
#define EMULATOED_MOTOR_MAX_RPM 6200
#define EMULATOED_MOTOR_IDLE_RPM 500

#define ERROR_CODE_CAN_SETUP_FAILED 3


#ifdef DEBUG_ENABLED
#define debug_print(...) Serial.print(__VA_ARGS__)
#define debug_println(...) Serial.println(__VA_ARGS__)
#else
#define debug_print(...)
#define debug_println(...)
#endif

#define CLAMP(v, min, max) (v < min ? min : (v > max ? max : v))

bool reverseSwitch = false;
bool driveSwitch = false;
bool neutralSwitch = false;
bool parkSwitch = false;
float acceleratorPedalPct = 0;
uint16_t targetEmulatedRpm = 0;
uint16_t rpmRamp = 50;
PACKET_Chrysler_300c_0x0308 outputPacket;

AsyncTimer timer;
LedManager led;

MCP_CAN can1(CAN1_CHIP_SELECT_PIN);
bool logCan1 = false;
uint64_t can1MessageCount = 0;
uint64_t can1MessageSentCount = 0;

MCP_CAN can2(CAN2_CHIP_SELECT_PIN);
bool logCan2 = false;
uint64_t can2MessageCount = 0;
uint64_t can2MessageSentCount = 0;

bool logParsedCanMessages = false;
bool logEmittedCanMessages = false;

uint8_t inBufferr[64];
uint8_t inBufferrLen = 0;

#define CLEAR_CMD_BUFFER() memset(&cmdBuffer, '\0', 128); cmdBufferLen = 0
uint8_t cmdBuffer[129];
uint8_t cmdBufferLen;

void emitTranslatedMessages(MCP_CAN* can, uint64_t* counter);
void processCan(const char* name, uint64_t* counter, bool logEnabled, MCP_CAN* can);
void processSerial();

/**
 * Called by the android subsystem to set things up.
 */
void setup() {

  Serial.begin(9600);
  delay(5000);
  CLEAR_CMD_BUFFER();
  debug_println("Starting CAN Translator");

  // setup can1
  if (CAN1_LISTEN_ENABLED || CAN1_EMIT_ENABLED) {
    debug_println("Configuring CAN1");
    if (can1.begin(MCP_ANY, CAN1_BAUD_RATE, MCP_16MHZ) != CAN_OK) {
      debug_println("Configuring CAN1 failed");
      while (true) {
        led.flashError(ERROR_CODE_CAN_SETUP_FAILED);
      }
    }
    can1.setMode(MCP_NORMAL);
    can1.enOneShotTX();
    debug_println("CAN1 started");
  }

  // setup can2
  if (CAN2_LISTEN_ENABLED || CAN2_EMIT_ENABLED) {
    debug_println("Configuring CAN2");
    if (can2.begin(MCP_ANY, CAN2_BAUD_RATE, MCP_16MHZ) != CAN_OK) {
      debug_println("Configuring CAN2 failed");
      while (true) {
        led.flashError(ERROR_CODE_CAN_SETUP_FAILED);
      }
    }
    can2.setMode(MCP_NORMAL);
    can2.enOneShotTX();
    debug_println("CAN2 started");
  }

  // setup emitters
  if (CAN1_EMIT_ENABLED) {
    debug_println("Emitting messages on CAN1");
    timer.setInterval([]() { emitTranslatedMessages(&can1, &can1MessageSentCount); }, CAN1_EMIT_FREQUENCE_MILLIS);
  }
  if (CAN1_LISTEN_ENABLED) {
    timer.setInterval([]() {
      debug_print("Listening for messages on CAN1 (in: ");
      debug_print(can1MessageCount, DEC);
      debug_print(", out: ");
      debug_print(can1MessageSentCount, DEC);
      debug_println(")");
    }, 10000);
  }

  if (CAN2_EMIT_ENABLED) {
    debug_println("Emitting messages on CAN2");
    timer.setInterval([]() { emitTranslatedMessages(&can2, &can2MessageSentCount); }, CAN2_EMIT_FREQUENCE_MILLIS);
  }
  if (CAN2_LISTEN_ENABLED) {
    timer.setInterval([]() {
      debug_print("Listening for messages on CAN2 (in: ");
      debug_print(can2MessageCount, DEC);
      debug_print(", out: ");
      debug_print(can2MessageSentCount, DEC);
      debug_println(")");
    }, 10000);
  }

  timer.setInterval([]() {
    if (targetEmulatedRpm == outputPacket.Engine_RPM) {
      return;
    }
    if (targetEmulatedRpm > outputPacket.Engine_RPM) {
      outputPacket.Engine_RPM += rpmRamp;
    } else {
      outputPacket.Engine_RPM -= rpmRamp;
    }
    // outputPacket.Engine_RPM += 1;
  }, 5);
}

/**
 * Called by the android subsystem at some frequency.
 */
void loop() {
  timer.handle();
  if (CAN1_LISTEN_ENABLED) {
    processCan("CAN1", &can1MessageCount, logCan1, &can1);
  }
  if (CAN2_LISTEN_ENABLED) {
    processCan("CAN2", &can2MessageCount, logCan2, &can2);
  }
  processSerial();
}

/**
 * Processes CAN messages.
 */
void processCan(const char* name, uint64_t* counter, bool logEnabled, MCP_CAN* can) {

  if (can->checkReceive() != CAN_MSGAVAIL) {
    return;
  }

  uint32_t msgId = -1;
  byte msgIsExtended = 0;
  if (can->readMsgBuf(&msgId, &msgIsExtended, &inBufferrLen, inBufferr) != CAN_OK) {
    debug_println("Unable to can.readMsgBuf");
    return;
  }

  (*counter)++;

  if (logEnabled) {
    Serial.print(name); Serial.print(" incoming message with ID: "); Serial.println(msgId, HEX); 
    for (uint8_t i = 0; i<inBufferrLen; i++) {
      Serial.print("  Byte "); Serial.print(i); Serial.print(": "); Serial.println(inBufferr[i], HEX); 
    }
  }

  // we ignore messages that aren't extended
  if (msgIsExtended != 1) {
    return;
  }

  // led.flashOn(5);

  switch (msgId) {

    case PACKET_ID_M106_DriverInputs1: {

      PACKET_M106_DriverInputs1* inPacket = (PACKET_M106_DriverInputs1*)&inBufferr[0];
      acceleratorPedalPct = static_cast<float>(inPacket->AccelPedal) * 0.392157f;

      if (logParsedCanMessages) {
        Serial.println("PACKET_M106_DriverInputs1:");
        Serial.print("  Manual_Regen2: "); Serial.println(inPacket->Manual_Regen2, DEC);
        Serial.print("  Manual_Regen1: "); Serial.println(inPacket->Manual_Regen1, DEC);
        Serial.print("  Manual_Regen: "); Serial.println(inPacket->Manual_Regen, DEC);
        Serial.print("  AccelPedalValid: "); Serial.println(inPacket->AccelPedalValid == 0 ? "false" : "true");
        Serial.print("  AccelPedal1Valid: "); Serial.println(inPacket->AccelPedal1Valid == 0 ? "false" : "true");
        Serial.print("  AccelPedal2Valid: "); Serial.println(inPacket->AccelPedal2Valid == 0 ? "false" : "true");
        Serial.print("  Brake_Switch: "); Serial.println(inPacket->Brake_Switch == 0 ? "false" : "true");
        Serial.print("  Brake_Switch1: "); Serial.println(inPacket->Brake_Switch1 == 0 ? "false" : "true");
        Serial.print("  Brake_Switch2: "); Serial.println(inPacket->Brake_Switch2 == 0 ? "false" : "true");
        Serial.print("  Ignition_Switch: "); Serial.println(inPacket->Ignition_Switch == 0 ? "false" : "true");
        Serial.print("  Start_Switch: "); Serial.println(inPacket->Start_Switch == 0 ? "false" : "true");
        Serial.print("  AccelPedalXCheckDiff: "); Serial.println(inPacket->AccelPedalXCheckDiff, DEC);
        Serial.print("  AccelPedal2: "); Serial.println(inPacket->AccelPedal2, DEC);
        Serial.print("  AccelPedal1: "); Serial.println(inPacket->AccelPedal1, DEC);
        Serial.print("  AccelPedal: "); Serial.println(inPacket->AccelPedal, DEC);
        Serial.print("  acceleratorPedalPct:"); Serial.println(acceleratorPedalPct);
      }
      break;
    }

    case PACKET_ID_M108_DriverInputs2: {

      PACKET_M108_DriverInputs2* inPacket = (PACKET_M108_DriverInputs2*)&inBufferr[0];
      reverseSwitch = inPacket->Reverse_Switch;
      driveSwitch = inPacket->Drive_Switch;
      neutralSwitch = inPacket->Neutral_Switch;
      parkSwitch = inPacket->Park_Switch;

      if (logParsedCanMessages) {
        Serial.println("PACKET_M108_DriverInputs2:");
        Serial.print("  ManRegen_XCheckDiff: "); Serial.println(inPacket->ManRegen_XCheckDiff, DEC);
        Serial.print("  ParkLamp_Switch: "); Serial.println(inPacket->ParkLamp_Switch ? "true" : "false");
        Serial.print("  Head_LampSwitch: "); Serial.println(inPacket->Head_LampSwitch ? "true" : "false");
        Serial.print("  Reverse_Switch: "); Serial.println(inPacket->Reverse_Switch ? "true" : "false");
        Serial.print("  Drive_Switch: "); Serial.println(inPacket->Drive_Switch ? "true" : "false");
        Serial.print("  Neutral_Switch: "); Serial.println(inPacket->Neutral_Switch ? "true" : "false");
        Serial.print("  Park_Switch: "); Serial.println(inPacket->Park_Switch ? "true" : "false");
        Serial.print("  Manual_Regen2Valid: "); Serial.println(inPacket->Manual_Regen2Valid ? "true" : "false");
        Serial.print("  Manual_Regen1Valid: "); Serial.println(inPacket->Manual_Regen1Valid ? "true" : "false");
        Serial.print("  Logging_Switch: "); Serial.println(inPacket->Logging_Switch ? "true" : "false");
        Serial.print("  Wake_Switch: "); Serial.println(inPacket->Wake_Switch ? "true" : "false");
        Serial.print("  Acc_LightSwitch: "); Serial.println(inPacket->Acc_LightSwitch ? "true" : "false");
        Serial.print("  ACSwitch: "); Serial.println(inPacket->ACSwitch ? "true" : "false");
        Serial.print("  HeaterSwitch: "); Serial.println(inPacket->HeaterSwitch ? "true" : "false");
        Serial.print("  Cooling_PumpOrdSwitch: "); Serial.println(inPacket->Cooling_PumpOrdSwitch ? "true" : "false");
        Serial.print("  Cooling_FanOrdSwitch: "); Serial.println(inPacket->Cooling_FanOrdSwitch ? "true" : "false");
        Serial.print("  Enable_Switch: "); Serial.println(inPacket->Enable_Switch ? "true" : "false");
        Serial.print("  ignored1: "); Serial.println(inPacket->ignored1, HEX);
        Serial.print("  CCSetCst: "); Serial.println(inPacket->CCSetCst ? "true" : "false");
        Serial.print("  CCRsmAcc: "); Serial.println(inPacket->CCRsmAcc ? "true" : "false");
        Serial.print("  CCOn: "); Serial.println(inPacket->CCOn ? "true" : "false");
        Serial.print("  CCCancel: "); Serial.println(inPacket->CCCancel ? "true" : "false");
      }
      break;
    }

    case PACKET_ID_M116_VehicleInputs3: {

      PACKET_M116_VehicleInputs3* inPacket = (PACKET_M116_VehicleInputs3*)&inBufferr[0];

      if (!driveSwitch && !reverseSwitch) {
      }

      if (driveSwitch || reverseSwitch) {
        Serial.println("PACKET_M116_VehicleInputs3:");
        Serial.print("  BrakeVacPressure: "); Serial.println(inPacket->BrakeVacPressure, DEC);
        Serial.print("  Vehicle_Speed: "); Serial.println(inPacket->Vehicle_Speed, DEC);
        Serial.print("  DriveShaft_Speed: "); Serial.println(inPacket->DriveShaft_Speed, DEC);
        Serial.print("  DriveWheel_Speed: "); Serial.println(inPacket->DriveWheel_Speed, DEC);
        Serial.print("  Ground_WheelSpeed: "); Serial.println(inPacket->Ground_WheelSpeed, DEC);
        Serial.print("  TC_Slip_Measured: "); Serial.println(inPacket->TC_Slip_Measured, DEC);

        float pct = static_cast<float>(inPacket->Vehicle_Speed) / static_cast<float>(100);
        targetEmulatedRpm = static_cast<float>(EMULATOED_MOTOR_MAX_RPM) * pct;
      } else {
        targetEmulatedRpm = static_cast<float>(EMULATOED_MOTOR_MAX_RPM) * (acceleratorPedalPct / static_cast<float>(100));
      }

      targetEmulatedRpm += EMULATOED_MOTOR_IDLE_RPM;

      if (logParsedCanMessages) {
        Serial.println("PACKET_M116_VehicleInputs3:");
        Serial.print("  BrakeVacPressure: "); Serial.println(inPacket->BrakeVacPressure, DEC);
        Serial.print("  Vehicle_Speed: "); Serial.println(inPacket->Vehicle_Speed, DEC);
        Serial.print("  DriveShaft_Speed: "); Serial.println(inPacket->DriveShaft_Speed, DEC);
        Serial.print("  DriveWheel_Speed: "); Serial.println(inPacket->DriveWheel_Speed, DEC);
        Serial.print("  Ground_WheelSpeed: "); Serial.println(inPacket->Ground_WheelSpeed, DEC);
        Serial.print("  TC_Slip_Measured: "); Serial.println(inPacket->TC_Slip_Measured, DEC);
      }
      break;
    }
  }
}

/**
 * Emits translated messages
 */
void emitTranslatedMessages(MCP_CAN* can, uint64_t* counter) {

  (*counter)++;

  if (logEmittedCanMessages) {
    Serial.println("EMITTING PACKET_Chrysler_300c_0x0308:");
    Serial.print("  Engine_RPM: "); Serial.println(outputPacket.Engine_RPM, DEC);
  }

  INT8U balls[8] = {
    0,
    outputPacket.Engine_RPM >> 8,
    outputPacket.Engine_RPM,
    0,
    0,
    0,
    0,
    0
  };
  
  // can->sendMsgBuf(Chrysler_300c_0x0308, Chrysler_300c_0x0308_Size, (INT8U*)&outputPacket);
  can->sendMsgBuf(Chrysler_300c_0x0308, Chrysler_300c_0x0308_Size, balls);
}

/**
 * Processes stuff from the serial port.
 */
void processSerial() {
  if (!Serial.available()) {
    return;
  }

  const char* cmd = (const char*)&cmdBuffer;

  bool commandFound = false;
  while (Serial.available()) {
    if (cmdBufferLen >= 128) {
      Serial.println("Serial command buffer overflow");
      CLEAR_CMD_BUFFER();
      return;
    }
    char val = Serial.read();

    if (val == '\r' || val == '\n') {
      commandFound = strlen(cmd) > 0;
      break;

    } else if (val == 8) {
      if (cmdBufferLen > 0) {
        cmdBuffer[cmdBufferLen - 1] = '\0';
        cmdBufferLen--;
      }
      Serial.print(val); 
      Serial.print(" ");
      Serial.print(val); 

    } else {
      Serial.print(val);
      cmdBuffer[cmdBufferLen] = val;
      cmdBufferLen++;
    }
  }

  if (!commandFound) {
    return;
  }
  Serial.println("");

  if (strcmp(cmd, "help") == 0) {
    Serial.println("log-can1: enable can1 logging to serial");
    Serial.println("log-can2: enable can2 logging to serial");
    Serial.println("log-can-parsed: enable logging of parsed can messages on all can networks");
    Serial.println("log-can-emitted: enable logging of emitted can messages on all can networks");

  } else if (strcmp(cmd, "log-can1") == 0) {
    logCan1 = !logCan1;
    Serial.print("CAN1 Logging: "); Serial.println(logCan1 ? "ON" : "OFF");

  } else if (strcmp(cmd, "log-can2") == 0) {
    logCan2 = !logCan2;
    Serial.print("CAN2 Logging: "); Serial.println(logCan2 ? "ON" : "OFF");

  } else if (strcmp(cmd, "log-can-parsed") == 0) {
    logParsedCanMessages = !logParsedCanMessages;
    Serial.print("Parsed CAN messages logging: "); Serial.println(logParsedCanMessages ? "ON" : "OFF");

  } else if (strcmp(cmd, "log-can-emitted") == 0) {
    logEmittedCanMessages = !logEmittedCanMessages;
    Serial.print("Emitted CAN messages logging: "); Serial.println(logEmittedCanMessages ? "ON" : "OFF");
    
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    
  }
  
  CLEAR_CMD_BUFFER();
  cmdBufferLen = 0;
}
