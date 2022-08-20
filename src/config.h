#pragma once

#ifndef ARDUINO_ARCH_RP2040

#include <SoftwareSerial.h>
extern SoftwareSerial softwareSerial;
#define driverSerial Serial
#define debugSerial softwareSerial

#else

#define driverSerial Serial1
#define debugSerial Serial

#endif