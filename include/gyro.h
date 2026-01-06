#ifndef GYRO_H
#define GYRO_H
#include <Arduino.h>

extern bool gyro_ok;
extern float currentYaw; 
extern float currentPitch;

void setupGyro();
bool updateGyro();
void flushGyro();
void dmpDataReady();  // Interrupt handler

#endif
