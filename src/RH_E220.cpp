#include <RHCRC.h>

#include "RH_E220.h"

#ifdef RH_HAVE_SERIAL

// Detect aux rising with an interrupt?
//volatile bool auxRising = false;
//
//void auxIsr() {
//    auxRising = true;
//}

RH_E220::RH_E220(Stream &stream, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin)
        : _stream(stream),
          _rxState(RxStateInitialising),
          _auxPin(auxPin),
          _m0Pin(m0Pin),
          _m1Pin(m1Pin),
          _targetAddh(0xFF),
          _targetAddl(0xFF),
          _targetChan(0x00) {
    pinMode(_auxPin, INPUT_PULLUP);
    pinMode(_m0Pin, OUTPUT);
    pinMode(_m1Pin, OUTPUT);
    digitalWrite(_m0Pin, HIGH);
    digitalWrite(_m1Pin, HIGH);
    //attachInterrupt(digitalPinToInterrupt(_auxPin), auxIsr, RISING);
}

bool RH_E220::init() {
    if (!RHGenericDriver::init())
        return false;

    _rxState = RxStateIdle;
    clearRxBuf();

    Parameters params;
    readParameters(params);
    //printBuffer("PARAMS", (uint8_t *) &params, sizeof(params));

    setCADTimeout(1000);
    return true;
}

bool RH_E220::setBaudRate(BaudRate rate, Parity parity) {
    Parameters params;
    if (!readParameters(params))
        return false;
    // The DataRate enums are the same values as the register bitmasks
    params.sped &= ~RH_E220_PARAM_SPED_UART_BAUD_MASK;
    params.sped |= (rate & RH_E220_PARAM_SPED_UART_BAUD_MASK);

    // Also set the parity
    params.sped &= ~RH_E220_PARAM_SPED_UART_MODE_MASK;
    params.sped |= (parity & RH_E220_PARAM_SPED_UART_MODE_MASK);

    return writeParameters(params);
}

bool RH_E220::setDataRate(DataRate rate) {
    Parameters params;
    if (!readParameters(params))
        return false;
    // The DataRate enums are the same values as the register bitmasks
    params.sped &= ~RH_E220_PARAM_SPED_DATARATE_MASK;
    params.sped |= (rate & RH_E220_PARAM_SPED_DATARATE_MASK);
    return writeParameters(params);
}

// Call this often
bool RH_E220::available() {
    while (!_rxBufValid && _stream.available()) {
        handleRx(_stream.read());
    }
    return _rxBufValid;
}

void RH_E220::waitAuxHigh() const {
    // REVISIT: timeout needed?
    while (digitalRead(_auxPin) == LOW);
//    while (!auxRising) {
//        Serial.print(_auxPin); Serial.print(": ");
//        Serial.println(digitalRead(_auxPin));
//    }
}

void RH_E220::handleRx(uint8_t ch) {
    //static int i = 0;

    // State machine for receiving chars
    switch (_rxState) {
        case RxStateIdle: {
            if (ch == DLE) {
                //Serial.print('D');
                _rxState = RxStateDLE;
            }
        }
            break;

        case RxStateDLE: {
            if (ch == STX) {
                //Serial.print('S'); i = 0;
                clearRxBuf();
                _rxState = RxStateData;
            } else {
                _rxState = RxStateIdle;
            }
        }
            break;

        case RxStateData: {
            if (ch == DLE) {
                //Serial.print('D');
                _rxState = RxStateEscape;
            } else {
                //if (i == 2)
                //  Serial.print((uint8_t) ch);
                //else {
                //  Serial.print('x'); i++;
                //}
                appendRxBuf(ch);
            }
        }
            break;

        case RxStateEscape: {
            if (ch == ETX) {
                //Serial.print('E');
                // add fcs for DLE, ETX
                _rxFcs = RHcrc_ccitt_update(_rxFcs, DLE);
                _rxFcs = RHcrc_ccitt_update(_rxFcs, ETX);
                _rxState = RxStateWaitFCS1; // End frame
            } else if (ch == DLE) {
                //Serial.print('x');
                appendRxBuf(ch);
                _rxState = RxStateData;
            } else {
                //Serial.print('U');
                _rxState = RxStateIdle; // Unexpected
            }
        }
            break;

        case RxStateWaitFCS1: {
            //Serial.print('C');
            _rxRecdFcs = ch << 8;
            _rxState = RxStateWaitFCS2;
        }
            break;

        case RxStateWaitFCS2: {
            //Serial.println('C');
            _rxRecdFcs |= ch;
            _rxState = RxStateIdle;
            validateRxBuf();
        }
            break;

        default: // Else some compilers complain
            break;
    }
}

void RH_E220::clearRxBuf() {
    _rxBufValid = false;
    _rxFcs = 0xffff;
    _rxBufLen = 0;
}

void RH_E220::appendRxBuf(uint8_t ch) {
    if (_rxBufLen < RH_E220_MAX_PAYLOAD_LEN) {
        // Normal data, save and add to FCS
        _rxBuf[_rxBufLen++] = ch;
        _rxFcs = RHcrc_ccitt_update(_rxFcs, ch);
    }
    // If the buffer overflows, we don't record the trailing data, and the FCS will be wrong,
    // causing the message to be dropped when the FCS is received
}

// Check whether the latest received message is complete and uncorrupted
void RH_E220::validateRxBuf() {
    if (_rxRecdFcs != _rxFcs) {
        _rxBad++;
        return;
    }

    // Extract the 4 headers
    _rxHeaderTo = _rxBuf[0];
    _rxHeaderFrom = _rxBuf[1];
    _rxHeaderId = _rxBuf[2];
    _rxHeaderFlags = _rxBuf[3];
    if (_promiscuous ||
        _rxHeaderTo == _thisAddress ||
        _rxHeaderTo == RH_BROADCAST_ADDRESS) {
        _rxGood++;
        _rxBufValid = true;
    }
}

bool RH_E220::recv(uint8_t *buf, uint8_t *len) {
    if (!available()) {
        return false;
    }

    if (buf && len) {
        // Skip the 4 headers that are at the beginning of the rxBuf
        if (*len > _rxBufLen - RH_E220_HEADER_LEN)
            *len = _rxBufLen - RH_E220_HEADER_LEN;
        memcpy(buf, _rxBuf + RH_E220_HEADER_LEN, *len);
    }

    clearRxBuf(); // This message accepted and cleared
    return true;
}

// Caution: this may block
bool RH_E220::send(const uint8_t *data, uint8_t len) {
    if (len > RH_E220_MAX_MESSAGE_LEN)
        return false;

    if (!waitCAD())
        return false;  // Check channel activity

    // Write the target
    _stream.write(_targetAddh);
    _stream.write(_targetAddl);
    _stream.write(_targetChan);

    _txFcs = 0xffff;    // Initial value
    _stream.write(DLE); // Not in FCS
    _stream.write(STX); // Not in FCS

    // First the 4 headers
    txData(_txHeaderTo);
    txData(_txHeaderFrom);
    txData(_txHeaderId);
    txData(_txHeaderFlags);

    //Serial.print(_targetAddh, HEX); Serial.print(' ');
    //Serial.print(_targetAddl, HEX); Serial.print(' ');
    //Serial.print(_targetChan, HEX); Serial.print(' ');
    //Serial.print('D');
    //Serial.print('S');
    //Serial.print('H');
    //Serial.print('H');
    //Serial.print(_txHeaderId);
    //Serial.print('H');

    // Now the payload
    while (len) {
        txData(*data++);
        len--;
        //Serial.print('x');
    }

    // End of message
    //Serial.print('D');
    //Serial.print('E');
    //Serial.print('C');
    //Serial.println('C');

    _stream.write(DLE);
    _txFcs = RHcrc_ccitt_update(_txFcs, DLE);
    _stream.write(ETX);
    _txFcs = RHcrc_ccitt_update(_txFcs, ETX);

    // Now send the calculated FCS for this message
    _stream.write((_txFcs >> 8) & 0xff);
    _stream.write(_txFcs & 0xff);

    return true;
}

void RH_E220::txData(uint8_t ch) {
    if (ch == DLE)    // DLE stuffing required?
        _stream.write(DLE); // Not in FCS
    _stream.write(ch);
    _txFcs = RHcrc_ccitt_update(_txFcs, ch);
}

uint8_t RH_E220::maxMessageLength() {
    return RH_E220_MAX_MESSAGE_LEN;
}

void RH_E220::setOperatingMode(OperatingMode mode) {
    //noInterrupts();
    //auxRising = false;
    //interrupts();
    PinStatus m0Status = digitalRead(_m0Pin);
    PinStatus m1Status = digitalRead(_m1Pin);
    switch (mode) {
        case ModeNormal:
            if (m0Status == LOW && m1Status == LOW)
                return;
            digitalWrite(_m0Pin, LOW);
            digitalWrite(_m1Pin, LOW);
            break;

        case ModeWakeUp:
            if (m0Status == HIGH && m1Status == LOW)
                return;
            digitalWrite(_m0Pin, HIGH);
            digitalWrite(_m1Pin, LOW);
            break;

        case ModePowerSaving:
            if (m0Status == LOW && m1Status == HIGH)
                return;
            digitalWrite(_m0Pin, LOW);
            digitalWrite(_m1Pin, HIGH);
            break;

        case ModeSleep:
            if (m0Status == HIGH && m1Status == HIGH)
                return;
            digitalWrite(_m0Pin, HIGH);
            digitalWrite(_m1Pin, HIGH);
            break;
    }
    waitAuxHigh();
    delay(10); // Takes a little while to start its response
}

bool RH_E220::readParameters(Parameters &params) {
    setOperatingMode(ModeSleep);

    uint8_t readParamsCommand[] = {RH_E220_COMMAND_READ_PARAMS, 0x00, sizeof(params)};
    _stream.write(readParamsCommand, sizeof(readParamsCommand));
    //printBuffer("COMMAND", readParamsCommand, sizeof(readParamsCommand));

    size_t result = _stream.readBytes(readParamsCommand, sizeof(readParamsCommand));
    //printBuffer("RESPONSE", readParamsCommand, sizeof(readParamsCommand));

    if (result != sizeof(readParamsCommand) || (
            readParamsCommand[0] == 0xFF &&
            readParamsCommand[1] == 0xFF &&
            readParamsCommand[2] == 0xFF)) {
        return false;
    }

    result = _stream.readBytes((uint8_t *) &params, sizeof(params));
    setOperatingMode(ModeNormal);
    //printBuffer("PARAMS", (uint8_t *) &params, sizeof(params));
    return (result == sizeof(Parameters));
}

bool RH_E220::writeParameters(Parameters &params, bool save) {
    setOperatingMode(ModeSleep);

    uint8_t head = save ? RH_E220_COMMAND_WRITE_PARAMS_SAVE : RH_E220_COMMAND_WRITE_PARAMS_NOSAVE;
    uint8_t writeParamsCommand[] = {head, 0x00, sizeof(params)};
    //printBuffer("writing now", (uint8_t*)&params, sizeof(params));

    size_t result = _stream.write(writeParamsCommand, sizeof(writeParamsCommand));
    if (result != sizeof(writeParamsCommand))
        return false;

    result = _stream.write((uint8_t *) &params, sizeof(params));
    if (result != sizeof(params))
        return false;

    // Now we expect to get the same data back
    result = _stream.readBytes(writeParamsCommand, sizeof(writeParamsCommand));
    if (result != sizeof(writeParamsCommand) || (
            writeParamsCommand[0] == 0xFF &&
            writeParamsCommand[1] == 0xFF &&
            writeParamsCommand[2] == 0xFF)) {
        return false;
    }

    result = _stream.readBytes((uint8_t *) &params, sizeof(params));
    if (result != sizeof(params))
        return false;

    //printBuffer("additional read", (uint8_t*)&params, sizeof(params));

    // Without a little delay here, writing params often fails
    //delay(20);

    setOperatingMode(ModeNormal);
    return true;
}

bool RH_E220::isChannelActive() {
    return digitalRead(_auxPin) == LOW;
}

void RH_E220::setTarget(uint8_t addh, uint8_t addl, uint8_t chan) {
    _targetAddh = addh;
    _targetAddl = addl;
    _targetChan = chan;
}

bool RH_E220::setPower(PowerLevel level) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.opt1 &= ~RH_E220_PARAM_OPT1_POWER_MASK;
    params.opt1 |= (level & RH_E220_PARAM_OPT1_POWER_MASK);
    return writeParameters(params);
}

bool RH_E220::setAddress(uint8_t addh, uint8_t addl) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.addh = addh;
    params.addl = addl;
    return writeParameters(params);
}

bool RH_E220::setChannel(uint8_t chan) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.chan = chan;
    return writeParameters(params);
}

bool RH_E220::setSubPacket(SubPacketLen len) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.opt1 &= ~RH_E220_PARAM_OPT1_SUBPKT_MASK;
    params.opt1 |= (len & RH_E220_PARAM_OPT1_SUBPKT_MASK);
    return writeParameters(params);
}

#endif // RH_HAVE_SERIAL
