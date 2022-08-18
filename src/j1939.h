
#ifndef J1939_h
#define J1939_h

#include <Arduino.h>

#define PGN_61443_ElectricEngineController2 61443
#define PGN_61443_ElectricEngineController2_Size 4

struct PACKET_PGN_61443_ElectricEngineController2 {
    uint8_t NotDefined1 : 2;
    uint8_t Road_Speed_Limit_Status : 2;
    uint8_t Accelerator_Pedal_Kickdown_Switch : 2;
    uint8_t Accelerator_Pedal_1_Low_Idle_Switch : 2;
    uint8_t Accelerator_Pedal_Position_1;
    uint8_t Percent_Load_At_Current_Speed;
    uint8_t Remote_Accelerator_Pedal_Position;
};


#endif
