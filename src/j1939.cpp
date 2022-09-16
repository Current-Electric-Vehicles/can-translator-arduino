
#include "j1939.h"

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

