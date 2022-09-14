
#ifndef AEMVCU_h
#define AEMVCU_h

#include <Arduino.h>

// https://www.aemev.com/files/instructions/AEMEV_VCU200_VCU300_CAN_Transmit_DB_RevD.pdf

#define PACKET_ID_M106_DriverInputs1 0x2F0A006

struct PACKET_M106_DriverInputs1 {
    unsigned AccelPedal;
    unsigned AccelPedal1;
    unsigned AccelPedal2;
    unsigned AccelPedalXCheckDiff;
    signed Start_Switch : 1;
    signed Ignition_Switch : 1;
    signed Brake_Switch2 : 1;
    signed Brake_Switch1 : 1;
    signed Brake_Switch : 1;
    signed AccelPedal2Valid : 1;
    signed AccelPedal1Valid : 1;
    signed AccelPedalValid : 1;
    unsigned Manual_Regen;
    unsigned Manual_Regen1;
    unsigned Manual_Regen2;
};


#define PACKET_ID_M116_VehicleInputs3 0x2F0A016

struct PACKET_M116_VehicleInputs3 {
    unsigned BrakeVacPressure;
    unsigned Vehicle_Speed;
    unsigned DriveShaft_Speed;
    unsigned DriveWheel_Speed;
    unsigned Ground_WheelSpeed;
    unsigned TC_Slip_Measured;
};


#define PACKET_ID_M138_MotorSpeedData2 0x2F0A038

struct PACKET_M138_MotorSpeedData2 {
    unsigned LaunchTarget_Speed;
    unsigned Motor_TargetSpeed;
    signed SpeedControl_PID : 12;
    signed SpeedControl_PID_Error : 12;
};


#define PACKET_ID_M120_MotorTorqueData1 0x2F0A020

struct PACKET_M120_MotorTorqueData1 {
    signed Motor1_Torque_Request : 12;
    unsigned Motor1_TqLimHi : 12;
    signed Motor1_TqLimLo : 12;
    unsigned Motor1_TqTable : 12;
    unsigned Motor1_TqLimMultHi : 8;
    unsigned Motor1_TqLimMultLo : 8;
};

#endif
