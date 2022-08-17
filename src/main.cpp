#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#include"LedManager.h"

#define CAN_CHIP_SELECT_PIN 3
#define CAN_BAUD_RATE 500E3

#define CAN_PACKET_M106_DriverInputs1 0x2F0A006
#define CAN_PGN_61443 0xF00316

#define ERROR_CODE_CAN_SETUP_FAILED 3

LedManager led;
MCP_CAN can(CAN_CHIP_SELECT_PIN);

unsigned long msgId = -1;
unsigned char msgBuffer[64];
byte msgBufferLen = 0;
byte msgIsExtended = 0;

byte sendJ1939(long lPGN, byte nPriority, byte nSrcAddr, byte nDestAddr, byte* nData, int nDataLen);

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

  if (can.readMsgBuf(&msgId, &msgIsExtended, &msgBufferLen, msgBuffer) != CAN_OK) {
    Serial.println("Unable to can.readMsgBuf");
    return;
  }

  // we ignore messages that aren't extended
  if (msgIsExtended != 1) {
    return;
  }

  led.flashOn(5);

  switch (msgId) {

    case CAN_PACKET_M106_DriverInputs1: {

      unsigned int accelPedal = msgBuffer[1];
      float accelPedalPct = ((float)accelPedal) * 0.392157f;

      Serial.print("AccelPedal %");
      Serial.println(accelPedalPct, 4);

      msgBuffer[0] = 0;  // 558, 559, 1437
      msgBuffer[1] = accelPedal;  // 91
      msgBuffer[2] = 0;  // 92
      msgBuffer[3] = 0;  // 974
      msgBuffer[4] = 0;  // unused
      msgBuffer[5] = 0;  // unused
      msgBuffer[6] = 0;  // unused
      msgBuffer[7] = 0;  // unused

      sendJ1939(61443, 0, 0, 0, msgBuffer, 8);
      
      break;
    }
  }
}

byte sendJ1939(long lPGN, byte nPriority, byte nSrcAddr, byte nDestAddr, byte* nData, int nDataLen) {

  // check for a p2p packet
  bool isPeerToPeer = false;
  if (lPGN > 0 && lPGN <= 0xEFFF) {
    isPeerToPeer = true;
  }
  if (lPGN > 0x10000 && lPGN <= 0x1EFFF) {
    isPeerToPeer = true;
  }

  long lID = static_cast<long>(nPriority)<< 26 | static_cast<long>(lPGN << 8) | static_cast<long>(nSrcAddr);

  if (isPeerToPeer) {
    lID = lID & 0xFFFF00FF;
    lID = lID | (static_cast<long>(nDestAddr) << 8);
  }

  return can.sendMsgBuf(lID, CAN_EXTID, nDataLen, nData);
}
