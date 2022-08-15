#include <Arduino.h>
#include "RHReliableDatagram.h"
#include "RH_E220.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define AUX_PIN 6
#define M0_PIN 2
#define M1_PIN 3

RH_E220 driver(Serial1, M0_PIN, M1_PIN, AUX_PIN);

//// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

uint8_t data[RH_E220_MAX_MESSAGE_LEN] = {};

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);

    delay(3000);
    Serial.println("Initializing...");

    driver.setChannel(0x15);

    driver.setTarget(0xFF, 0xFF, 0x17);
//    driver.setPower(RH_E220::Power22dBm);

    manager.setTimeout(2000);

    if (!manager.init())
        Serial.println("init failed");

//    while(true) {
//        Serial1.write(0x02);
//        Serial1.write(0x02);
//        Serial1.write(0x17);
//        for (size_t i = 0; i < 196; ++i) {
//            Serial1.write("a");
//        }
////        driver.printConfig();
//        delay(2000);
//    }

    Serial.println("Done");
    for (uint8_t &i: data)
        i = 'A';

    //manager.setTimeout(2000); // Might need this at slow data rates or if a radio is involved
}


// Dont put this on the stack:
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
    delay(200);
}



