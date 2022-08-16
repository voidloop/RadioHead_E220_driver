#include "RHReliableDatagram.h"
#include "RH_E220.h"

#include "Arduino.h"

#define AUX_PIN 6
#define M0_PIN 2
#define M1_PIN 3

#define LOCAL_ADDRESS 1
#define REMOTE_ADDRESS 2

#define LOCAL_CHAN 0x15
#define REMOTE_CHAN 0x16

RH_E220 driver(Serial1, M0_PIN, M1_PIN, AUX_PIN); // NOLINT(cppcoreguidelines-interfaces-global-init)
RHReliableDatagram manager(driver, LOCAL_ADDRESS);

void setup()
{
    Serial.begin(115200);
    delay(3000);

    Serial.println("Initializing...");
    Serial1.begin(RH_E220_CONFIG_UART_BAUD);

    if (!manager.init())
        Serial.println("Failed");

    driver.setChannel(LOCAL_CHAN);
    driver.setTarget(0xFF,0xFF,REMOTE_CHAN);
    Serial1.begin(9600);

    manager.setTimeout(2000);
    Serial.println("Done");

    Serial.print("> ");
}

char data[RH_E220_MAX_MESSAGE_LEN];
uint8_t lenTo = 0;

void sendMessage() {
    Serial.println();
    Serial.print("Sending... ");

    if (manager.sendtoWait((uint8_t *)data, lenTo, REMOTE_ADDRESS)) {
        Serial.println("OK");
    } else {
        Serial.println("Failed");
    }
}

void loop() {
    if (Serial.available()) {
        char inChar = Serial.read();
        if ((inChar == '\r' || inChar == '\n') && lenTo > 0) {
            data[lenTo] = '\0';
            sendMessage();
            Serial.print("> ");
            lenTo = 0;
        } else if (lenTo == RH_MAX_MESSAGE_LEN-1) {
            Serial.println();
            Serial.println("Message is too long!");
            Serial.print("> ");
        } else {
            Serial.print(inChar);
            data[lenTo++] = inChar;
        }
    }

    if (manager.available()) {
        uint8_t from, lenFrom;
        manager.recvfromAck((uint8_t *)data, &lenFrom, &from);

        Serial.println();
        Serial.print("Message from ");
        Serial.print(from);
        Serial.print(": ");
        Serial.write(data, lenFrom);
        Serial.println();

        // Echo user input:
        Serial.print("> ");
        Serial.write(data, lenTo);
    }
}

