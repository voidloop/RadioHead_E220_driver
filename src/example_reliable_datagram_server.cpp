#include <Arduino.h>
#include <RHReliableDatagram.h>

#include "RH_E220_Serial.h"

#define SERVER_ADDRESS 2

#ifndef ARDUINO_ARCH_RP2040
    #include <SoftwareSerial.h>
    SoftwareSerial softwareSerial(10, 11);
    #define driverSerial Serial
    #define debugSerial softwareSerial
#else
    #define driverSerial Serial1
    #define debugSerial Serial
#endif

RH_E220_Serial driver(driverSerial, M0_PIN, M1_PIN, AUX_PIN); // NOLINT(cppcoreguidelines-interfaces-global-init)

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

void activityIsr() {
    digitalWrite(LED_BUILTIN, !digitalRead(AUX_PIN));
}

void setup() {
    debugSerial.begin(115200);
    driverSerial.begin(9600);
    delay(3000);

    debugSerial.println("Initializing...");

    if (!manager.init())
        debugSerial.println("Failed");
    else
        debugSerial.println("Done");

    manager.setTimeout(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(AUX_PIN), activityIsr, CHANGE);
}

uint8_t data[] = "PONG!";
// Don't put this on the stack
uint8_t buf[RH_SERIAL_MAX_MESSAGE_LEN];

void loop() {
    // Wait for a message addressed to us from the client
    manager.waitAvailable();

    uint8_t len = sizeof(buf);
    uint8_t from;

    if (manager.recvfromAck(buf, &len, &from)) {
        debugSerial.print("got request from : 0x");
        debugSerial.print(from, HEX);
        debugSerial.print(": ");
        debugSerial.println((char *) buf);

        // Send a reply back to the originator client
        if (!manager.sendtoWait(data, sizeof(data), from)) {
            debugSerial.println("sendtoWait failed");
        }
    }
}

