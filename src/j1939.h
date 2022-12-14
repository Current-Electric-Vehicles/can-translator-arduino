
#ifndef J1939_h
#define J1939_h

#include <Arduino.h>
#include <mcp_can.h>

byte sendJ1939(MCP_CAN* can, uint32_t lPGN, uint8_t nPriority, uint8_t nSrcAddr, uint8_t nDestAddr, uint8_t* nData, uint8_t nDataLen);

// https://powertraincontrolsolutions.com/download/Released/Public/Developer_Files/PCS%20J1939%20Messages%20v2_1.pdf

#define PGN_61443_ElectricEngineController2 61443
#define PGN_61443_ElectricEngineController2_Size 4

struct PACKET_PGN_61443_ElectricEngineController2 {
    unsigned NotDefined1 : 2;
    unsigned Road_Speed_Limit_Status : 2;
    unsigned Accelerator_Pedal_Kickdown_Switch : 2;
    unsigned Accelerator_Pedal_1_Low_Idle_Switch : 2;
    unsigned Accelerator_Pedal_Position_1;
    unsigned Percent_Load_At_Current_Speed;
    unsigned Remote_Accelerator_Pedal_Position;
};

#endif
