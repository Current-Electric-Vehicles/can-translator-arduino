#include <Arduino.h>
#include <mcp_can.h>

#include"LedManager.h"
#include"aem_vcu.h"
#include"j1939.h"

#define DEBUG_PRINT 1

#define CAN_CHIP_SELECT_PIN 3
#define CAN_BAUD_RATE 500E3

#define ERROR_CODE_CAN_SETUP_FAILED 3

LedManager led;
MCP_CAN can(CAN_CHIP_SELECT_PIN);

uint8_t inBufferr[64];
uint8_t inBufferrLen = 0;

byte sendJ1939(uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen);

void setup() {

  led.off();

  Serial.begin(9600);
  while (!Serial) {
    led.flashOn(250);
  }

  if (can.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    while (true) {
      led.flashError(ERROR_CODE_CAN_SETUP_FAILED);
    }
  }
  can.setMode(MCP_NORMAL);
}

void loop() {
  if (can.checkReceive() != CAN_MSGAVAIL) {
    return;
  }

  uint32_t msgId = -1;
  byte msgIsExtended = 0;
  if (can.readMsgBuf(&msgId, &msgIsExtended, &inBufferrLen, inBufferr) != CAN_OK) {
    Serial.println("Unable to can.readMsgBuf");
    return;
  }

  // we ignore messages that aren't extended
  if (msgIsExtended != 1) {
    return;
  }

  led.flashOn(2);

  switch (msgId) {

    case PACKET_ID_M106_DriverInputs1: {

      PACKET_M106_DriverInputs1* inPacket = (PACKET_M106_DriverInputs1*)&inBufferr[0];
      float AccelPedalPct = static_cast<float>(inPacket->AccelPedal) * 0.392157f;

      #ifdef DEBUG_PRINT
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
        Serial.print("  AccelPedal: "); Serial.println(inPacket->AccelPedal, DEC);
        Serial.print("  AccelPedalPct:"); Serial.println(AccelPedalPct, 4);
      #endif


      PACKET_PGN_61443_ElectricEngineController2 outPacket;
      outPacket.Road_Speed_Limit_Status = 0;
      outPacket.Accelerator_Pedal_Kickdown_Switch = 0;
      outPacket.Accelerator_Pedal_1_Low_Idle_Switch = 0;
      outPacket.Accelerator_Pedal_Position_1 = inPacket->AccelPedal;
      outPacket.Percent_Load_At_Current_Speed = 1;
      outPacket.Remote_Accelerator_Pedal_Position = 0;
      sendJ1939(PGN_61443_ElectricEngineController2, 3, 0, 0, (uint8_t*)&outPacket, PGN_61443_ElectricEngineController2_Size);

      break;
    }

    case PACKET_ID__M116_VehicleInputs3: {

      PACKET_M116_VehicleInputs3* inPacket = (PACKET_M116_VehicleInputs3*)&inBufferr[0];
      float BrakeVacPressurePct = (static_cast<float>(inPacket->BrakeVacPressure) * 0.14504f) - 14.696f;

      #ifdef DEBUG_PRINT
        Serial.println("PACKET_M106_DriverInputs1:");
        Serial.print("  TC_Slip_Measured: "); Serial.println(inPacket->TC_Slip_Measured, DEC);
        Serial.print("  Ground_WheelSpeed: "); Serial.println(inPacket->Ground_WheelSpeed, DEC);
        Serial.print("  DriveWheel_Speed: "); Serial.println(inPacket->DriveWheel_Speed, DEC);
        Serial.print("  DriveShaft_Speed: "); Serial.println(inPacket->DriveShaft_Speed, DEC);
        Serial.print("  Vehicle_Speed: "); Serial.println(inPacket->Vehicle_Speed, DEC);
        Serial.print("  BrakeVacPressure: "); Serial.println(inPacket->BrakeVacPressure, DEC);
        Serial.print("  BrakeVacPressurePct:"); Serial.println(BrakeVacPressurePct, 4);
      #endif

      break;
    }
  }
}

byte sendJ1939(uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen) {

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

  return can.sendMsgBuf(lID, CAN_EXTID, nDataLen, nData);
}
