
#ifndef Chrysler_300c_h
#define Chrysler_300c_h

#include <Arduino.h>

// https://www.crossfireforum.org/forum/audio-video-electronics/84837-can-bus-hacking-crossfire.html

#define Chrysler_300c_0x0308 0x0308
#define Chrysler_300c_0x0308_Size 8

/*
0x0308, 0, 1, clutch pressed
0x0308, 8, 16, engine RPM               // this seems wrong
0x0308, 30, 1, check engine lamp on
0x0308, 29, 1, oil lamp on
0x0308, 39, 1, coolant lamp on
0x0308, 32, 1, oil temp high
0x0308, 40, 8, oil temp
0x0308, 48, 8, oil level
0x0308, 56, 8, oil quality
*/

struct PACKET_Chrysler_300c_0x0308 {
    uint8_t ignored1;
    uint16_t Engine_RPM;
    uint8_t ignored2;
    uint8_t ignored3;
    uint8_t ignored4;
    uint8_t ignored5;
    uint8_t ignored6;
};


#endif
