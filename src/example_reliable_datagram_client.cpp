#include <Arduino.h>
#include <RHReliableDatagram.h>

#include "config.h"
#include "RH_E220_Serial.h"

#define SERVER_ADDRESS 2
#define CLIENT_ADDRESS 1

#ifndef ARDUINO_ARCH_RP2040
#include <SoftwareSerial.h>
SoftwareSerial softwareSerial(10, 11);
#endif

RH_E220_Serial driver(driverSerial, M0_PIN, M1_PIN, AUX_PIN);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);


void activityIsr() {
    digitalWrite(LED_BUILTIN, !digitalRead(AUX_PIN));
}

void setup() {
    driverSerial.begin(9600);
    debugSerial.begin(9600);

    delay(3000);

    debugSerial.println("Initializing...");

    if (!manager.init()) {
        debugSerial.println("Failed");
    } else {
        debugSerial.println("Done");
    }

    // It seems a good timeout for this experiment
    manager.setTimeout(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(AUX_PIN), activityIsr, CHANGE);
}

uint8_t data[RH_SERIAL_MAX_MESSAGE_LEN] = "PING";

// Don't put this on the stack
uint8_t buf[RH_SERIAL_MAX_MESSAGE_LEN];

void loop() {
    debugSerial.println("Sending to example_reliable_datagram_server");

    // Send a message to manager_server
    if (manager.sendtoWait(data, RH_SERIAL_MAX_MESSAGE_LEN, SERVER_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (manager.recvfromAckTimeout(buf, &len, 6000, &from)) {
            debugSerial.print("got reply from : 0x");
            debugSerial.print(from, HEX);
            debugSerial.print(": ");
            debugSerial.println((char *) buf);
        } else {
            debugSerial.println("No reply, is example_reliable_datagram_server running?");
        }
    } else {
        debugSerial.println("sendtoWait failed");
    }

    delay(10000);
}



