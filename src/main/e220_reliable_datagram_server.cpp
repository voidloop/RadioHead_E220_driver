// serial_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_E220 driver to
// communicate using packets over a stream port (or a radio connected to a
// stream port, such as the 3DR Telemetry radio V1 and others).
// It is designed to work with the other example serial_reliable_datagram_client
// Tested on Arduino Mega and ChipKit Uno32 (normal Arduinos only have one
// stream port and so it's not possible to test on them and still have debug
// output)
// Tested with Arduino Mega, Teensy 3.1, Moteino, Arduino Due
// Also works on Linux an OSX. Build and test with:
//  tools/simBuild examples/stream/serial_reliable_datagram_server/serial_reliable_datagram_server.pde
//  RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB0 ./serial_reliable_datagram_server

#include "RHReliableDatagram.h"
#include "RH_E220.h"

#include "Arduino.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define AUX_PIN 6
#define M0_PIN 2
#define M1_PIN 3

RH_E220 driver(Serial1, M0_PIN, M1_PIN, AUX_PIN); // NOLINT(cppcoreguidelines-interfaces-global-init)

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

void setup()
{
    Serial.begin(115200);
    delay(3000);

    Serial.println("Initializing...");
    Serial1.begin(RH_E220_CONFIG_BAUD_RATE);

//    driver.setPower(RH_E220::Power22dBm);
//    driver.setAddress(0x03, 0x03);
    driver.setChannel(0x16);
//    driver.setBaudRate();

    driver.setTarget(0x1,0x1,0x15);

    if (!manager.init())
        Serial.println("Failed");

//    Serial.println("loop");
//    while (true) {
//        while (Serial1.available()) {
//            Serial.print(Serial1.read());
//            Serial.print(',');
//        }
//    }

    manager.setTimeout(2000);
    Serial.println("Done");
}

uint8_t data[] = "And hello back to you";
// Don't put this on the stack
uint8_t buf[RH_E220_MAX_MESSAGE_LEN];

void loop()
{
    // Wait for a message addressed to us from the client
    manager.waitAvailable();

    uint8_t len = sizeof(buf);
    uint8_t from;

    if (manager.recvfromAck(buf, &len, &from))
    {
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);

        // Send a reply back to the originator client
        if (!manager.sendtoWait(data, sizeof(data), from))
            Serial.println("sendtoWait failed");
    }
}

