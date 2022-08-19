#include <Arduino.h>
#include <RHReliableDatagram.h>

#include "RH_E220_Serial.h"

#define SERVER_ADDRESS 2

RH_E220_Serial driver(Serial1, M0_PIN, M1_PIN, AUX_PIN); // NOLINT(cppcoreguidelines-interfaces-global-init)

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);


void activityIsr() {
    digitalWrite(LED_BUILTIN, !digitalRead(AUX_PIN));
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(RH_E220_CONFIG_UART_BAUD);

    delay(3000);

    Serial.println("Initializing...");

    if (!manager.init()) {
        Serial.println("Failed");
    } else {
        Serial.println("Done");
    }

    driver.setChannel(0x16);

    // It seems a good timeout for this experiment
    manager.setTimeout(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(AUX_PIN), activityIsr, CHANGE);

}

uint8_t data[RH_SERIAL_MAX_MESSAGE_LEN] = "PING";

// Don't put this on the stack
uint8_t buf[RH_SERIAL_MAX_MESSAGE_LEN];

void loop() {
    Serial.println("Sending to example_reliable_datagram_server");

    // Send a message to manager_server
    if (manager.sendtoWait(data, RH_SERIAL_MAX_MESSAGE_LEN, SERVER_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (manager.recvfromAckTimeout(buf, &len, 6000, &from)) {
            Serial.print("got reply from : 0x");
            Serial.print(from, HEX);
            Serial.print(": ");
            Serial.println((char *) buf);
        } else {
            Serial.println("No reply, is example_reliable_datagram_server running?");
        }
    } else {
        Serial.println("sendtoWait failed");
    }
}



