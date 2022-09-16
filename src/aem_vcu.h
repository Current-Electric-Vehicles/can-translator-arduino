
#ifndef AEMVCU_h
#define AEMVCU_h

#include <Arduino.h>

// https://www.aemev.com/files/instructions/AEMEV_VCU200_VCU300_CAN_Transmit_DB_RevD.pdf

#define PACKET_ID_M106_DriverInputs1 0x2F0A006

struct PACKET_M106_DriverInputs1 {
    uint8_t AccelPedal;
    uint8_t AccelPedal1;
    uint8_t AccelPedal2;
    uint8_t AccelPedalXCheckDiff;
    bool Start_Switch : 1;
    bool Ignition_Switch : 1;
    bool Brake_Switch2 : 1;
    bool Brake_Switch1 : 1;
    bool Brake_Switch : 1;
    bool AccelPedal2Valid : 1;
    bool AccelPedal1Valid : 1;
    bool AccelPedalValid : 1;
    uint8_t Manual_Regen;
    uint8_t Manual_Regen1;
    unsigned Manual_Regen2;
};

#define PACKET_ID_M108_DriverInputs2 0x2F0A008

struct PACKET_M108_DriverInputs2 {
    uint8_t ManRegen_XCheckDiff : 8;
    bool ParkLamp_Switch : 1;
    bool Head_LampSwitch : 1;
    bool Reverse_Switch : 1;
    bool Drive_Switch : 1;
    bool Neutral_Switch : 1;
    bool Park_Switch : 1;
    bool Manual_Regen2Valid : 1;
    bool Manual_Regen1Valid : 1;
    bool Logging_Switch : 1;
    bool Wake_Switch : 1;
    bool Acc_LightSwitch : 1;
    bool ACSwitch : 1;
    bool HeaterSwitch : 1;
    bool Cooling_PumpOrdSwitch : 1;
    bool Cooling_FanOrdSwitch : 1;
    bool Enable_Switch : 1;
    uint8_t ignored1 : 4;
    bool CCSetCst : 1;
    bool CCRsmAcc : 1;
    bool CCOn : 1;
    bool CCCancel : 1;
};


#define PACKET_ID_M116_VehicleInputs3 0x2F0A016

struct PACKET_M116_VehicleInputs3 {
    uint8_t BrakeVacPressure;
    uint8_t Vehicle_Speed;
    uint16_t DriveShaft_Speed;
    uint8_t DriveWheel_Speed;
    uint8_t Ground_WheelSpeed;
    uint8_t TC_Slip_Measured;
};


#define PACKET_ID_M138_MotorSpeedData2 0x2F0A038

struct PACKET_M138_MotorSpeedData2 {
    uint16_t LaunchTarget_Speed;
    uint16_t Motor_TargetSpeed;
    uint16_t SpeedControl_PID : 12;
    uint16_t SpeedControl_PID_Error : 12;
};

#endif
