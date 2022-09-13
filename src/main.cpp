#include <Arduino.h>
#include <mcp_can.h>
#include <AsyncTimer.h>

#include "LedManager.h"
#include "aem_vcu.h"
#include "j1939.h"

#define DEBUG_ENABLED true

#define CAN1_CHIP_SELECT_PIN 3
#define CAN1_LISTEN_ENABLED true
#define CAN1_EMIT_ENABLED true
#define CAN1_BAUD_RATE CAN_500KBPS
#define CAN1_EMIT_FREQUENCE_MILLIS 50

#define CAN2_CHIP_SELECT_PIN 4
#define CAN2_LISTEN_ENABLED true
#define CAN2_EMIT_ENABLED true
#define CAN2_BAUD_RATE CAN_250KBPS
#define CAN2_EMIT_FREQUENCE_MILLIS 50

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
MCP_CAN can2(CAN2_CHIP_SELECT_PIN);

uint8_t inBufferr[64];
uint8_t inBufferrLen = 0;

PACKET_PGN_61443_ElectricEngineController2 pgn61443;
bool pgn61443_valid = false;

byte sendJ1939(MCP_CAN* can, uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen);
void emitTranslatedMessages(MCP_CAN* can);
void processCan(MCP_CAN* can);

/**
 * Called by the android subsystem to set things up
 */
void setup() {

  Serial.begin(9600);
  delay(5000);
  debug_println("Starting CAN Translator");

  // setup can1
  if (can1.begin(MCP_STDEXT, CAN1_BAUD_RATE, MCP_16MHZ) != CAN_OK) {
    debug_println("Configuring CAN1 failed");
    while (true) {
      led.flashError(ERROR_CODE_CAN_SETUP_FAILED);
    }
  }
  can1.setMode(MCP_NORMAL);
  debug_println("CAN1 started");

  // setup can2
  if (can2.begin(MCP_STDEXT, CAN2_BAUD_RATE, MCP_16MHZ) != CAN_OK) {
    debug_println("Configuring CAN2 failed");
    while (true) {
      led.flashError(ERROR_CODE_CAN_SETUP_FAILED);
    }
  }
  can2.setMode(MCP_NORMAL);
  debug_println("CAN2 started");

  // setup emitters
  if (CAN1_EMIT_ENABLED) {
    debug_println("Listening for messages on CAN1");
    timer.setInterval([]() { emitTranslatedMessages(&can1); }, CAN1_EMIT_FREQUENCE_MILLIS);
    timer.setInterval([]() { debug_println("- Listening for messages on CAN1"); led.flashOn(5); }, 10000);
  }
  if (CAN2_EMIT_ENABLED) {
    debug_println("Listening for messages on CAN2");
    timer.setInterval([]() { emitTranslatedMessages(&can2); }, CAN2_EMIT_FREQUENCE_MILLIS);
    timer.setInterval([]() { debug_println("- Listening for messages on CAN2"); led.flashOn(5); }, 10000);
  }
}

/**
 * Called by the android subsystem at some frequency
 */
void loop() {
  timer.handle();

  if (CAN1_LISTEN_ENABLED) {
    processCan(&can1);
  }
  if (CAN2_LISTEN_ENABLED) {
    processCan(&can2);
  }
}

void processCan(MCP_CAN* can) {

  if (can->checkReceive() != CAN_MSGAVAIL) {
    return;
  }

  uint32_t msgId = -1;
  byte msgIsExtended = 0;
  if (can->readMsgBuf(&msgId, &msgIsExtended, &inBufferrLen, inBufferr) != CAN_OK) {
    debug_println("Unable to can.readMsgBuf");
    return;
  }

  debug_print("Incoming can message with iud: "); debug_println(msgId, DEC);

  // we ignore messages that aren't extended
  if (msgIsExtended != 1) {
    return;
  }

  led.flashOn(5);

  switch (msgId) {

    case PACKET_ID_M106_DriverInputs1: {

      PACKET_M106_DriverInputs1* inPacket = (PACKET_M106_DriverInputs1*)&inBufferr[0];
      uint8_t AccelPedalPct = static_cast<float>(inPacket->AccelPedal) * 0.392157f;

      debug_println("PACKET_M106_DriverInputs1:");
      debug_print("  Manual_Regen2: "); debug_println(inPacket->Manual_Regen2, DEC);
      debug_print("  Manual_Regen1: "); debug_println(inPacket->Manual_Regen1, DEC);
      debug_print("  Manual_Regen: "); debug_println(inPacket->Manual_Regen, DEC);
      debug_print("  AccelPedalValid: "); debug_println(inPacket->AccelPedalValid == 0 ? "false" : "true");
      debug_print("  AccelPedal1Valid: "); debug_println(inPacket->AccelPedal1Valid == 0 ? "false" : "true");
      debug_print("  AccelPedal2Valid: "); debug_println(inPacket->AccelPedal2Valid == 0 ? "false" : "true");
      debug_print("  Brake_Switch: "); debug_println(inPacket->Brake_Switch == 0 ? "false" : "true");
      debug_print("  Brake_Switch1: "); debug_println(inPacket->Brake_Switch1 == 0 ? "false" : "true");
      debug_print("  Brake_Switch2: "); debug_println(inPacket->Brake_Switch2 == 0 ? "false" : "true");
      debug_print("  Ignition_Switch: "); debug_println(inPacket->Ignition_Switch == 0 ? "false" : "true");
      debug_print("  Start_Switch: "); debug_println(inPacket->Start_Switch == 0 ? "false" : "true");
      debug_print("  AccelPedalXCheckDiff: "); debug_println(inPacket->AccelPedalXCheckDiff, DEC);
      debug_print("  AccelPedal2: "); debug_println(inPacket->AccelPedal2, DEC);
      debug_print("  AccelPedal1: "); debug_println(inPacket->AccelPedal1, DEC);
      debug_print("  AccelPedal: "); debug_println(inPacket->AccelPedal, DEC);
      debug_print("  AccelPedalPct:"); debug_println(AccelPedalPct, 4);

      pgn61443_valid = true;
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

      debug_println("PACKET_M106_DriverInputs1:");
      debug_print("  TC_Slip_Measured: "); debug_println(inPacket->TC_Slip_Measured, DEC);
      debug_print("  Ground_WheelSpeed: "); debug_println(inPacket->Ground_WheelSpeed, DEC);
      debug_print("  DriveWheel_Speed: "); debug_println(inPacket->DriveWheel_Speed, DEC);
      debug_print("  DriveShaft_Speed: "); debug_println(inPacket->DriveShaft_Speed, DEC);
      debug_print("  Vehicle_Speed: "); debug_println(inPacket->Vehicle_Speed, DEC);
      debug_print("  BrakeVacPressure: "); debug_println(inPacket->BrakeVacPressure, DEC);
      debug_print("  BrakeVacPressurePct:"); debug_println(BrakeVacPressurePct, 4);

      break;
    }

    case PACKET_ID_M138_MotorSpeedData2: {

      PACKET_M138_MotorSpeedData2* inPacket = (PACKET_M138_MotorSpeedData2*)&inBufferr[0];
      uint8_t LaunchTarget_Speed = (static_cast<float>(inPacket->LaunchTarget_Speed) * 0.25f);
      uint8_t Motor_TargetSpeed = (static_cast<float>(inPacket->Motor_TargetSpeed) * 0.25f);
      uint8_t SpeedControl_PID = (static_cast<float>(inPacket->SpeedControl_PID) * 0.5f);
      uint8_t SpeedControl_PID_Error = (static_cast<float>(inPacket->SpeedControl_PID_Error) * 0.5f);

      debug_println("PACKET_M138_MotorSpeedData2:");
      debug_print("  LaunchTarget_Speed: "); debug_println(inPacket->LaunchTarget_Speed, DEC);
      debug_print("  Motor_TargetSpeed: "); debug_println(inPacket->Motor_TargetSpeed, DEC);
      debug_print("  SpeedControl_PID: "); debug_println(inPacket->SpeedControl_PID, DEC);
      debug_print("  SpeedControl_PID_Error: "); debug_println(inPacket->SpeedControl_PID_Error, DEC);
      debug_print("  LaunchTarget_Speed: "); debug_println(LaunchTarget_Speed, DEC);
      debug_print("  Motor_TargetSpeed: "); debug_println(Motor_TargetSpeed, DEC);
      debug_print("  SpeedControl_PID: "); debug_println(SpeedControl_PID, DEC);
      debug_print("  SpeedControl_PID_Error: "); debug_println(SpeedControl_PID_Error, DEC);

      break;
    }

    case PACKET_ID_M120_MotorTorqueData1: {

      PACKET_M120_MotorTorqueData1* inPacket = (PACKET_M120_MotorTorqueData1*)&inBufferr[0];
      uint8_t Motor1_Torque_RequestPct = CLAMP((abs(inPacket->Motor1_Torque_Request) / MAX_MOTOR_TORQUE) * 100, 0, 250);

      debug_println("PACKET_M120_MotorTorqueData1:");
      debug_print("  Motor1_Torque_Request: "); debug_println(inPacket->Motor1_Torque_Request, DEC);
      debug_print("  Motor1_TqLimHi: "); debug_println(inPacket->Motor1_TqLimHi, DEC);
      debug_print("  Motor1_TqLimLo: "); debug_println(inPacket->Motor1_TqLimLo, DEC);
      debug_print("  Motor1_TqTable: "); debug_println(inPacket->Motor1_TqTable, DEC);
      debug_print("  Motor1_TqLimMultHi: "); debug_println(inPacket->Motor1_TqLimMultHi, DEC);
      debug_print("  Motor1_TqLimMultLo: "); debug_println(inPacket->Motor1_TqLimMultLo, DEC);
      debug_print("  Motor1_Torque_RequestPct: "); debug_println(Motor1_Torque_RequestPct, DEC);

      pgn61443_valid = true;
      pgn61443.Percent_Load_At_Current_Speed = Motor1_Torque_RequestPct;

      break;
    }
  }
}

/**
 * Emits translated messages
 */
void emitTranslatedMessages(MCP_CAN* can) {

  // send PGN 61443
  if (pgn61443_valid) {
    debug_println("EMITTING PACKET_PGN_61443_ElectricEngineController2:");
    debug_print("  NotDefined1: "); debug_println(pgn61443.NotDefined1, DEC);
    debug_print("  Road_Speed_Limit_Status: "); debug_println(pgn61443.Road_Speed_Limit_Status, DEC);
    debug_print("  Accelerator_Pedal_Kickdown_Switch: "); debug_println(pgn61443.Accelerator_Pedal_Kickdown_Switch, DEC);
    debug_print("  Accelerator_Pedal_1_Low_Idle_Switch: "); debug_println(pgn61443.Accelerator_Pedal_1_Low_Idle_Switch, DEC);
    debug_print("  Accelerator_Pedal_Position_1: "); debug_println(pgn61443.Accelerator_Pedal_Position_1, DEC);
    debug_print("  Percent_Load_At_Current_Speed: "); debug_println(pgn61443.Percent_Load_At_Current_Speed, DEC);
    debug_print("  Remote_Accelerator_Pedal_Position:"); debug_println(pgn61443.Remote_Accelerator_Pedal_Position, DEC);
    sendJ1939(can, PGN_61443_ElectricEngineController2, 3, 0, 0, (uint8_t*)&pgn61443, PGN_61443_ElectricEngineController2_Size);
  }
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
