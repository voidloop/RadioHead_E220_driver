//
// Created by exp on 19/08/22.
//

#include "RH_E220_Serial.h"

RH_E220_Serial::RH_E220_Serial(HardwareSerial &serial, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin) :
        RH_Serial(serial),
        _m0Pin(m0Pin),
        _m1Pin(m1Pin),
        _auxPin(auxPin) {
    pinMode(_auxPin, INPUT);
    pinMode(_m0Pin, OUTPUT);
    pinMode(_m1Pin, OUTPUT);
    digitalWrite(_m0Pin, HIGH);
    digitalWrite(_m1Pin, HIGH);
}

bool RH_E220_Serial::init() {
    if (!RH_Serial::init())
        return false;

    Parameters currentParams;
    if (!readParameters(currentParams))
        return false;

    Parameters defaultParams;
    defaultParams.addh = RH_E220_DEFAULT_ADDRESS_HIGH;
    defaultParams.addl = RH_E220_DEFAULT_ADDRESS_LOW;
    defaultParams.chan = RH_E220_DEFAULT_CHANNEL;
    defaultParams.sped = RH_E220_DEFAULT_UART_BAUD |
                         RH_E220_DEFAULT_UART_MODE |
                         RH_E220_DEFAULT_DATA_RATE;
    defaultParams.opt1 = RH_E220_DEFAULT_TX_POWER;
    defaultParams.opt2 = RH_E220_DEFAULT_WOR_CYCLE;

#ifdef RH_E220_RSSI_BYTE_ENABLED
    defaultParams.opt2 |= RH_E220_PARAM_OPT2_RSSI_BYTE_ENABLE;
#endif

    if (memcmp(&defaultParams, &currentParams, sizeof(Parameters)) != 0) {
        if (!writeParameters(defaultParams, true))
            return false;
    }

    setMode(RHModeRx);
    return true;
}

bool RH_E220_Serial::recv(uint8_t *buf, uint8_t *len) {
    bool copied = RH_Serial::recv(buf, len);

#ifdef RH_E220_RSSI_BYTE_ENABLED
    // Retrieve RSSI from the serial after a successful read
    if (copied && _serial.available()) {
        uint8_t rssiByte = _serial.read();
        _lastRssi = (int16_t) (-256 + rssiByte);
    }
#endif

    return copied;
}

bool RH_E220_Serial::send(const uint8_t *data, uint8_t len) {
    bool sent = RH_Serial::send(data, len);
    if (sent)
        setMode(RHModeTx);
    return sent;
}

bool RH_E220_Serial::waitPacketSent() {
    if (_mode == RHModeTx)
        waitAuxHigh();
    setMode(RHModeRx);
    return true;
}

bool RH_E220_Serial::readParameters(Parameters &params) {
    setOperatingMode(ModeSleep);

    uint8_t readParamsCommand[] = {RH_E220_COMMAND_READ_PARAMS, 0x00, sizeof(params)};
    _serial.write(readParamsCommand, sizeof(readParamsCommand));

    size_t result = _serial.readBytes(readParamsCommand, sizeof(readParamsCommand));

    if (result != sizeof(readParamsCommand) || (
            readParamsCommand[0] == 0xFF &&
            readParamsCommand[1] == 0xFF &&
            readParamsCommand[2] == 0xFF)) {
        return false;
    }

    result = _serial.readBytes((uint8_t *) &params, sizeof(params));
    setOperatingMode(ModeNormal);
    return (result == sizeof(Parameters));
}

bool RH_E220_Serial::writeParameters(Parameters &params, bool save) {
    setOperatingMode(ModeSleep);

    uint8_t head = save ? RH_E220_COMMAND_WRITE_PARAMS_SAVE : RH_E220_COMMAND_WRITE_PARAMS_NOSAVE;
    uint8_t writeParamsCommand[] = {head, 0x00, sizeof(params)};

    size_t result = _serial.write(writeParamsCommand, sizeof(writeParamsCommand));
    if (result != sizeof(writeParamsCommand))
        return false;

    result = _serial.write((uint8_t *) &params, sizeof(params));
    if (result != sizeof(params))
        return false;

    // Now we expect to get the same data back
    result = _serial.readBytes(writeParamsCommand, sizeof(writeParamsCommand));
    if (result != sizeof(writeParamsCommand) || (
            writeParamsCommand[0] == 0xFF &&
            writeParamsCommand[1] == 0xFF &&
            writeParamsCommand[2] == 0xFF)) {
        return false;
    }

    result = _serial.readBytes((uint8_t *) &params, sizeof(params));
    if (result != sizeof(params))
        return false;

    setOperatingMode(ModeNormal);
    return true;
}

void RH_E220_Serial::setOperatingMode(OperatingMode mode) const {
    waitAuxHigh();
    delay(10);

    switch (mode) {
        case ModeNormal:
            digitalWrite(_m0Pin, LOW);
            digitalWrite(_m1Pin, LOW);
            break;

        case ModeWakeUp:
            digitalWrite(_m0Pin, HIGH);
            digitalWrite(_m1Pin, LOW);
            break;

        case ModePowerSaving:
            digitalWrite(_m0Pin, LOW);
            digitalWrite(_m1Pin, HIGH);
            break;

        case ModeSleep:
            digitalWrite(_m0Pin, HIGH);
            digitalWrite(_m1Pin, HIGH);
            break;
    }

    waitAuxHigh();
    delay(10); // Takes a little while to start its response
}

void RH_E220_Serial::waitAuxHigh() const {
    // REVISIT: timeout needed?
    while (digitalRead(_auxPin) == LOW);
}

bool RH_E220_Serial::setChannel(uint8_t chan) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.chan = chan;
    return writeParameters(params);
}