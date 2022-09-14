#include <Arduino.h>
#include <mcp_can.h>
#include <AsyncTimer.h>

#include "LedManager.h"
#include "aem_vcu.h"
#include "j1939.h"

#define DEBUG_ENABLED true

#define CAN1_CHIP_SELECT_PIN 3
#define CAN1_LISTEN_ENABLED true
#define CAN1_EMIT_ENABLED false
#define CAN1_BAUD_RATE CAN_500KBPS
#define CAN1_EMIT_FREQUENCE_MILLIS 20

#define CAN2_CHIP_SELECT_PIN 4
#define CAN2_LISTEN_ENABLED true
#define CAN2_EMIT_ENABLED true
#define CAN2_BAUD_RATE CAN_250KBPS
#define CAN2_EMIT_FREQUENCE_MILLIS 20

#define MAX_MOTOR_TORQUE 500

#define ERROR_CODE_CAN_SETUP_FAILED 3


#ifdef DEBUG_ENABLED
#define debug_print(...) Serial.print(__VA_ARGS__)
#define debug_println(...) Serial.println(__VA_ARGS__)
#else
#define debug_print(...)
#define debug_println(...)
#endif

#define CLAMP(v, min, max) (v < min ? min : (v > max ? max : v))

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

PACKET_PGN_61443_ElectricEngineController2 pgn61443;

byte sendJ1939(MCP_CAN* can, uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen);
void emitTranslatedMessages(MCP_CAN* can, uint64_t* counter);
void processCan(const char* name, uint64_t* counter, bool logEnabled, MCP_CAN* can);
void processSerial();

/**
 * Called by the android subsystem to set things up
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
}

/**
 * Called by the android subsystem at some frequency
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
    Serial.println("packet: show current packet");
    Serial.println("log-can1: enable can1 logging to serial");
    Serial.println("log-can2: enable can2 logging to serial");
    Serial.println("log-can-parsed: enable logging of parsed can messages on all can networks");
    Serial.println("log-can-emitted: enable logging of emitted can messages on all can networks");

  } else if (strcmp(cmd, "packet") == 0) {
    Serial.println("PACKET_PGN_61443_ElectricEngineController2:");
    Serial.print("  NotDefined1: "); Serial.println(pgn61443.NotDefined1, DEC);
    Serial.print("  Road_Speed_Limit_Status: "); Serial.println(pgn61443.Road_Speed_Limit_Status, DEC);
    Serial.print("  Accelerator_Pedal_Kickdown_Switch: "); Serial.println(pgn61443.Accelerator_Pedal_Kickdown_Switch, DEC);
    Serial.print("  Accelerator_Pedal_1_Low_Idle_Switch: "); Serial.println(pgn61443.Accelerator_Pedal_1_Low_Idle_Switch, DEC);
    Serial.print("  Accelerator_Pedal_Position_1: "); Serial.println(pgn61443.Accelerator_Pedal_Position_1, DEC);
    Serial.print("  Percent_Load_At_Current_Speed: "); Serial.println(pgn61443.Percent_Load_At_Current_Speed, DEC);
    Serial.print("  Remote_Accelerator_Pedal_Position:"); Serial.println(pgn61443.Remote_Accelerator_Pedal_Position, DEC);

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

  } else if (strcmp(cmd, "send-can1") == 0) {
    Serial.println("sent");
    logEmittedCanMessages = !logEmittedCanMessages;
    static uint8_t val = 0;
    val++;
    can1.sendMsgBuf(1, 1, &val);

  } else if (strcmp(cmd, "send-can2") == 0) {
    Serial.println("sent");
    logEmittedCanMessages = !logEmittedCanMessages;
    static uint8_t val = 0;
    val++;
    can2.sendMsgBuf(1, 1, &val);
    
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    
  }
  
  CLEAR_CMD_BUFFER();
  cmdBufferLen = 0;
}

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
      uint8_t AccelPedalPct = static_cast<float>(inPacket->AccelPedal) * 0.392157f;

      if (logParsedCanMessages && false) {
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
        Serial.print("  AccelPedalPct:"); Serial.println(AccelPedalPct);
      }

      pgn61443.Road_Speed_Limit_Status = 1;
      pgn61443.Accelerator_Pedal_Kickdown_Switch = (AccelPedalPct >= 95) ? 1 : 0;
      pgn61443.Accelerator_Pedal_1_Low_Idle_Switch = 0b11;
      pgn61443.Accelerator_Pedal_Position_1 = inPacket->AccelPedal;
      pgn61443.Remote_Accelerator_Pedal_Position = 0;

      break;
    }

    case PACKET_ID_M116_VehicleInputs3: {

      PACKET_M116_VehicleInputs3* inPacket = (PACKET_M116_VehicleInputs3*)&inBufferr[0];
      uint8_t BrakeVacPressurePct = static_cast<uint8_t>((static_cast<float>(inPacket->BrakeVacPressure) * 0.14504f) - 14.696f);

      if (logParsedCanMessages && false) {
        Serial.println("PACKET_ID_M116_VehicleInputs3:");
        Serial.print("  TC_Slip_Measured: "); Serial.println(inPacket->TC_Slip_Measured, DEC);
        Serial.print("  Ground_WheelSpeed: "); Serial.println(inPacket->Ground_WheelSpeed, DEC);
        Serial.print("  DriveWheel_Speed: "); Serial.println(inPacket->DriveWheel_Speed, DEC);
        Serial.print("  DriveShaft_Speed: "); Serial.println(inPacket->DriveShaft_Speed, DEC);
        Serial.print("  Vehicle_Speed: "); Serial.println(inPacket->Vehicle_Speed, DEC);
        Serial.print("  BrakeVacPressure: "); Serial.println(inPacket->BrakeVacPressure, DEC);
        Serial.print("  BrakeVacPressurePct:"); Serial.println(BrakeVacPressurePct);
      }

      break;
    }

    case PACKET_ID_M138_MotorSpeedData2: {

      PACKET_M138_MotorSpeedData2* inPacket = (PACKET_M138_MotorSpeedData2*)&inBufferr[0];
      uint8_t LaunchTarget_Speed = (static_cast<float>(inPacket->LaunchTarget_Speed) * 0.25f);
      uint8_t Motor_TargetSpeed = (static_cast<float>(inPacket->Motor_TargetSpeed) * 0.25f);
      uint8_t SpeedControl_PID = (static_cast<float>(inPacket->SpeedControl_PID) * 0.5f);
      uint8_t SpeedControl_PID_Error = (static_cast<float>(inPacket->SpeedControl_PID_Error) * 0.5f);

      if (logParsedCanMessages && false) {
        Serial.println("PACKET_M138_MotorSpeedData2:");
        Serial.print("  LaunchTarget_Speed: "); Serial.println(inPacket->LaunchTarget_Speed, DEC);
        Serial.print("  Motor_TargetSpeed: "); Serial.println(inPacket->Motor_TargetSpeed, DEC);
        Serial.print("  SpeedControl_PID: "); Serial.println(inPacket->SpeedControl_PID, DEC);
        Serial.print("  SpeedControl_PID_Error: "); Serial.println(inPacket->SpeedControl_PID_Error, DEC);
        Serial.print("  LaunchTarget_Speed: "); Serial.println(LaunchTarget_Speed, DEC);
        Serial.print("  Motor_TargetSpeed: "); Serial.println(Motor_TargetSpeed, DEC);
        Serial.print("  SpeedControl_PID: "); Serial.println(SpeedControl_PID, DEC);
        Serial.print("  SpeedControl_PID_Error: "); Serial.println(SpeedControl_PID_Error, DEC);
      }

      break;
    }

    case PACKET_ID_M120_MotorTorqueData1: {

      PACKET_M120_MotorTorqueData1* inPacket = (PACKET_M120_MotorTorqueData1*)&inBufferr[0];
      float Motor1_Torque_RequestPct = (static_cast<float>(inPacket->Motor1_Torque_Request) * 0.5);
      Motor1_Torque_RequestPct /= static_cast<float>(MAX_MOTOR_TORQUE);
      Motor1_Torque_RequestPct *= static_cast<float>(100);

      if (logParsedCanMessages && true) {
        Serial.println("PACKET_M120_MotorTorqueData1:");
        Serial.print("  Motor1_Torque_Request: "); Serial.println(inPacket->Motor1_Torque_Request, DEC);
        Serial.print("  Motor1_TqLimHi: "); Serial.println(inPacket->Motor1_TqLimHi, DEC);
        Serial.print("  Motor1_TqLimLo: "); Serial.println(inPacket->Motor1_TqLimLo, DEC);
        Serial.print("  Motor1_TqTable: "); Serial.println(inPacket->Motor1_TqTable, DEC);
        Serial.print("  Motor1_TqLimMultHi: "); Serial.println(inPacket->Motor1_TqLimMultHi, DEC);
        Serial.print("  Motor1_TqLimMultLo: "); Serial.println(inPacket->Motor1_TqLimMultLo, DEC);
        Serial.print("  Motor1_Torque_RequestPct: "); Serial.println(Motor1_Torque_RequestPct);
      }

      pgn61443.Percent_Load_At_Current_Speed = static_cast<uint8_t>(Motor1_Torque_RequestPct);

      /*
      Serial.print(name); Serial.print(" incoming message with ID: "); Serial.println(msgId, HEX); 
      for (uint8_t i = 0; i<inBufferrLen; i++) {
        Serial.print("  Byte "); Serial.print(i); Serial.print(": "); Serial.println(inBufferr[i], HEX); 
      }
      */

      break;
    }
  }
}

/**
 * Emits translated messages
 */
void emitTranslatedMessages(MCP_CAN* can, uint64_t* counter) {

  (*counter)++;

  // send PGN 61443
  // pgn61443.Accelerator_Pedal_Position_1 = 50 / 0.4;
  // pgn61443.Percent_Load_At_Current_Speed = 200;

  if (logEmittedCanMessages) {
    Serial.println("EMITTING PACKET_PGN_61443_ElectricEngineController2:");
    Serial.print("  NotDefined1: "); Serial.println(pgn61443.NotDefined1, DEC);
    Serial.print("  Road_Speed_Limit_Status: "); Serial.println(pgn61443.Road_Speed_Limit_Status, DEC);
    Serial.print("  Accelerator_Pedal_Kickdown_Switch: "); Serial.println(pgn61443.Accelerator_Pedal_Kickdown_Switch, DEC);
    Serial.print("  Accelerator_Pedal_1_Low_Idle_Switch: "); Serial.println(pgn61443.Accelerator_Pedal_1_Low_Idle_Switch, DEC);
    Serial.print("  Accelerator_Pedal_Position_1: "); Serial.println(pgn61443.Accelerator_Pedal_Position_1, DEC);
    Serial.print("  Percent_Load_At_Current_Speed: "); Serial.println(pgn61443.Percent_Load_At_Current_Speed, DEC);
    Serial.print("  Remote_Accelerator_Pedal_Position:"); Serial.println(pgn61443.Remote_Accelerator_Pedal_Position, DEC);
  }
  
  sendJ1939(can, PGN_61443_ElectricEngineController2, 3, 0, 0, (uint8_t*)&pgn61443, PGN_61443_ElectricEngineController2_Size);
}

/**
 * Sends a J1939 CAN message
 **/
byte sendJ1939(MCP_CAN* can, uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen) {

  // check for a p2p packet
  bool isPeerToPeer = false;
  if (lPGN > 0 && lPGN <= 0xEFFF) {
    isPeerToPeer = true;
  }
  if (lPGN > 0x10000 && lPGN <= 0x1EFFF) {
    isPeerToPeer = true;
  }

  uint32_t lID = static_cast<uint32_t>(nPriority)<< 26 | static_cast<uint32_t>(lPGN << 8) | static_cast<uint32_t>(nSrcAddr);

  if (isPeerToPeer) {
    lID = lID & 0xFFFF00FF;
    lID = lID | (static_cast<uint32_t>(nDestAddr) << 8);
  }

  return can->sendMsgBuf(lID, CAN_EXTID, nDataLen, nData);
}
