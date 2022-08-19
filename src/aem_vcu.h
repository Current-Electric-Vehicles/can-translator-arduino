
#ifndef AEMVCU_h
#define AEMVCU_h

#include <Arduino.h>

// https://www.aemev.com/files/instructions/AEMEV_VCU200_VCU300_CAN_Transmit_DB_RevD.pdf

#define PACKET_ID_M106_DriverInputs1 0x2F0A006

struct PACKET_M106_DriverInputs1 {
    uint8_t Manual_Regen2;
    uint8_t Manual_Regen1;
    uint8_t Manual_Regen;
    int8_t AccelPedalValid : 1;
    int8_t AccelPedal1Valid : 1;
    int8_t AccelPedal2Valid : 1;
    int8_t Brake_Switch : 1;
    int8_t Brake_Switch1 : 1;
    int8_t Brake_Switch2 : 1;
    int8_t Ignition_Switch : 1;
    int8_t Start_Switch : 1;
    uint8_t AccelPedalXCheckDiff;
    uint8_t AccelPedal2;
    uint8_t AccelPedal1;
    uint8_t AccelPedal;
};


#define PACKET_ID_M116_VehicleInputs3 0x2F0A016

struct PACKET_M116_VehicleInputs3 {
    uint8_t TC_Slip_Measured;
    uint8_t Ground_WheelSpeed;
    uint8_t DriveWheel_Speed;
    uint16_t DriveShaft_Speed;
    uint8_t Vehicle_Speed;
    uint8_t BrakeVacPressure;
};


#define PACKET_ID_M138_MotorSpeedData2 0x2F0A038

struct PACKET_M138_MotorSpeedData2 {
    uint16_t SpeedControl_PID_Error : 12;
    uint16_t SpeedControl_PID : 12;
    uint16_t Motor_TargetSpeed;
    uint16_t LaunchTarget_Speed;
    uint8_t Undefined;
};

#endif
