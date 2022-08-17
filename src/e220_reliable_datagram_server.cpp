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

    // Remember to set serial baud rate before call initialise
    // the driver or any other driver.set* function
    Serial1.begin(RH_E220_CONFIG_UART_BAUD);

    driver.setChannel(0x16);
    driver.setTarget(0xFF,0xFF,0x14);

    if (!manager.init())
        Serial.println("Failed");
    else
        Serial.println("Done");

    // Remember to set serial baud rate after driver initialisation
    Serial1.begin(9600);

    manager.setTimeout(2000);
}

uint8_t data[] = "PONG!";
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
        if (!manager.sendtoWait(data, sizeof(data), from)) {
            Serial.println("sendtoWait failed");
        }
    }
}

