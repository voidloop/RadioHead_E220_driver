#include <Arduino.h>
#include <RHReliableDatagram.h>

#include "RH_E220.h"

#define M0_PIN 2
#define M1_PIN 3
#define AUX_PIN 6

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

RH_E220 driver(Serial1, M0_PIN, M1_PIN, AUX_PIN); // NOLINT(cppcoreguidelines-interfaces-global-init)

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

uint8_t data[RH_E220_MAX_MESSAGE_LEN] = {};

void setup() {
    Serial.begin(115200);

    // Remember to set serial baud rate before call initialise
    // the driver or any other driver.set* function
    Serial1.begin(RH_E220_CONFIG_UART_BAUD);

    delay(3000);
    Serial.println("Initializing...");

    driver.setTarget(0xFF, 0xFF, 0x16);

    driver.setChannel(0x15);


    if (!manager.init())
        Serial.println("Failed");

    for (uint8_t &i: data)
        i = 'A';

    Serial.println("Done");

    // Remember to set serial baud rate after driver initialisation
    Serial1.begin(9600);

    // It seems a good timeout for this experiment
    manager.setTimeout(2000);
}

// Don't put this on the stack
uint8_t buf[RH_E220_MAX_MESSAGE_LEN];

void loop() {
    Serial.println("Sending to serial_reliable_datagram_server");

    // Send a message to manager_server
    if (manager.sendtoWait(data, RH_E220_MAX_MESSAGE_LEN, SERVER_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (manager.recvfromAckTimeout(buf, &len, 6000, &from)) {
            Serial.print("got reply from : 0x");
            Serial.print(from, HEX);
            Serial.print(": ");
            Serial.println((char *) buf);
        } else {
            Serial.println("No reply, is serial_reliable_datagram_server running?");
        }
    } else
        Serial.println("sendtoWait failed");
}



