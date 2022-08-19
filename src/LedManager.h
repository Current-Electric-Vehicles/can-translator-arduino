
#ifndef LedManager_h
#define LedManager_h

class LedManager {
public:
    LedManager();
    void on();
    void off();
    bool isOn();
    bool isOff();
    void toggle();
    void flashOn(unsigned long milliseconds);
    void flashOff(unsigned long milliseconds);
    void flashOnNum(unsigned int count, unsigned long milliseconds = 250);
    void flashOffNum(unsigned int count, unsigned long milliseconds = 250);
    void flashError(unsigned int errorCode);
};

#endif