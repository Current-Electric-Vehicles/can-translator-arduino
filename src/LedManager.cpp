
#include <Arduino.h>
#include "LedManager.h"

void LedManager::on() {
    digitalWrite(LED_BUILTIN, HIGH);
}

void LedManager::off() {
    digitalWrite(LED_BUILTIN, LOW);
}

bool LedManager::isOn() {
    PinStatus status = digitalRead(LED_BUILTIN);
    return  (status == RISING || status == HIGH);
}

bool LedManager::isOff() {
    PinStatus status = digitalRead(LED_BUILTIN);
    return  (status == FALLING || status == LOW);
}

void LedManager::flashOn(unsigned long milliseconds) {
    this->on();
    delay(milliseconds);
    this->off();
}

void LedManager::flashOff(unsigned long milliseconds) {
    this->off();
    delay(milliseconds);
    this->on();
}

void LedManager::flashOnNum(unsigned int count, unsigned long milliseconds) {
    for (unsigned int i=0; i<count; i++) {
        this->flashOn(milliseconds);
    }
}

void LedManager::flashOffNum(unsigned int count, unsigned long milliseconds) {
    for (unsigned int i=0; i<count; i++) {
        this->flashOff(milliseconds);
    }
}

void LedManager::flashError(unsigned int errorCode) {
    this->flashOnNum(errorCode);
    delay(2000);
}
