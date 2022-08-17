#include <RHCRC.h>

#include "RH_E220.h"

#ifdef RH_HAVE_SERIAL

RH_E220::RH_E220(Stream &stream, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin)
        : _stream(stream),
          _rxState(RxStateInitialising),
          _auxPin(auxPin),
          _m0Pin(m0Pin),
          _m1Pin(m1Pin),
          _target({0xFF, 0xFF, 0x00}) {
    pinMode(_auxPin, INPUT);
    pinMode(_m0Pin, OUTPUT);
    pinMode(_m1Pin, OUTPUT);
    digitalWrite(_m0Pin, HIGH);
    digitalWrite(_m1Pin, HIGH);
}

bool RH_E220::init() {
    if (!RHGenericDriver::init())
        return false;

    _rxState = RxStateIdle;

    Parameters params;
    if (!readParameters(params))
        return false;

    uint8_t sped = RH_E220_DEFAULT_UART_BAUD |
                   RH_E220_DEFAULT_UART_MODE |
                   RH_E220_DEFAULT_DATA_RATE;

    uint8_t opt1 = RH_E220_DEFAULT_POWER;

    uint8_t opt2 = RH_E220_PARAM_OPT2_TX_METHOD_FIXED |
                   RH_E220_PARAM_OPT2_WOR_CYCLE_2000;

#ifdef RH_E220_RSSI_BYTE_ENABLED
    opt2 |= RH_E220_PARAM_OPT2_RSSI_BYTE_ENABLE;
#endif

    bool write = params.sped != sped ||
                 params.opt1 != opt1 ||
                 params.opt2 != opt2;

    if (write) {
        params.sped = sped;
        params.opt1 = opt1;
        params.opt2 = opt2;
        if (!writeParameters(params, true))
            return false;
    }

    setMode(RHModeRx);
    return true;
}

// Call this often
bool RH_E220::available() {
    while (!_rxBufValid && _stream.available()) {
        uint8_t ch = _stream.read();
        //Serial.print(ch);
        handleRx(ch);
    }
    return _rxBufValid;
}

void RH_E220::waitAuxHigh() const {
    // REVISIT: timeout needed?
    while (digitalRead(_auxPin) == LOW);
}

void RH_E220::handleRx(uint8_t ch) {
    static uint8_t dataLen = 0;

    // State machine for receiving chars
    switch (_rxState) {
        case RxStateIdle: {
            if (ch == PREAMBLE) {
                _rxState = RxStatePreamble1;
            }
        }
            break;

        case RxStatePreamble1: {
            if (ch == PREAMBLE) {
                _rxState = RxStatePreamble2;
            } else {
                _rxState = RxStateIdle;
            }
        }
            break;

        case RxStatePreamble2: {
            if (ch == PREAMBLE) {
                _rxState = RxStateLength;
            } else {
                _rxState = RxStateIdle;
            }
        }
            break;

        case RxStateLength: {
            dataLen = ch;
            if (dataLen < RH_E220_HEADER_LEN || dataLen > RH_E220_MAX_PAYLOAD_LEN) {
                // Broken packet or junk?
                _rxState = RxStateIdle;
            } else {
                clearRxBuf();
                _rxState = RxStateData;
            }
        }
            break;

        case RxStateData: {
            dataLen--;
            appendRxBuf(ch);
            if (dataLen == 0)
                _rxState = RxStateWaitFCS1;
        }
            break;

        case RxStateWaitFCS1: {
            _rxRecdFcs = ch << 8;
            _rxState = RxStateWaitFCS2;
        }
            break;

        case RxStateWaitFCS2: {
            _rxRecdFcs |= ch;
#ifdef RH_E220_RSSI_BYTE_ENABLED
            _rxState = RxStateWaitRSSI;
#else
            _rxState = RxStateIdle;
            validateRxBuf();
#endif
        }
            break;

#ifdef RH_E220_RSSI_BYTE_ENABLED
        case RxStateWaitRSSI: {
            _lastRssi = ch;
            _rxState = RxStateIdle;
            validateRxBuf();
        }
            break;
#endif

        default: // Else some compilers complain
            break;
    }
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

void RH_E220::appendRxBuf(uint8_t ch) {
    // Normal data, save and add to FCS
    _rxBuf[_rxBufLen++] = ch;
    _rxFcs = RHcrc_ccitt_update(_rxFcs, ch);
}

void RH_E220::clearRxBuf() {
    _rxBufValid = false;
    _rxFcs = 0xffff;
    _rxBufLen = 0;
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

    waitPacketSent();

    // Write the target
    _stream.write((uint8_t *) &_target, sizeof(_target));

    _txFcs = 0xffff;    // Initial value
    _stream.write(PREAMBLE); // Not in FCS
    _stream.write(PREAMBLE); // Not in FCS
    _stream.write(PREAMBLE); // Not in FCS
    _stream.write(len + RH_E220_HEADER_LEN); // Not in FCS

    // First the 4 headers
    txData(_txHeaderTo);
    txData(_txHeaderFrom);
    txData(_txHeaderId);
    txData(_txHeaderFlags);

    // Now the payload
    while (len) {
        txData(*data++);
        len--;
    }

    // Now send the calculated FCS for this message
    _stream.write((_txFcs >> 8) & 0xff);
    _stream.write(_txFcs & 0xff);

    setMode(RHModeTx);
    _txGood++;

    return true;
}

bool RH_E220::waitPacketSent() {
    if (_mode == RHModeTx)
        waitAuxHigh();
    setMode(RHModeRx);
    return true;
}

void RH_E220::txData(uint8_t ch) {
    _stream.write(ch);
    _txFcs = RHcrc_ccitt_update(_txFcs, ch);
}

uint8_t RH_E220::maxMessageLength() {
    return RH_E220_MAX_MESSAGE_LEN;
}

void RH_E220::setOperatingMode(OperatingMode mode) {
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
    // I don't understand this delay...
    delay(10); // Takes a little while to start its response
}

bool RH_E220::readParameters(Parameters &params) {
    setOperatingMode(ModeSleep);

    uint8_t readParamsCommand[] = {RH_E220_COMMAND_READ_PARAMS, 0x00, sizeof(params)};
    _stream.write(readParamsCommand, sizeof(readParamsCommand));

    size_t result = _stream.readBytes(readParamsCommand, sizeof(readParamsCommand));

    if (result != sizeof(readParamsCommand) || (
            readParamsCommand[0] == 0xFF &&
            readParamsCommand[1] == 0xFF &&
            readParamsCommand[2] == 0xFF)) {
        return false;
    }

    result = _stream.readBytes((uint8_t *) &params, sizeof(params));
    setOperatingMode(ModeNormal);
    return (result == sizeof(Parameters));
}

bool RH_E220::writeParameters(Parameters &params, bool save) {
    setOperatingMode(ModeSleep);

    uint8_t head = save ? RH_E220_COMMAND_WRITE_PARAMS_SAVE : RH_E220_COMMAND_WRITE_PARAMS_NOSAVE;
    uint8_t writeParamsCommand[] = {head, 0x00, sizeof(params)};

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

    setOperatingMode(ModeNormal);
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

void RH_E220::setTarget(uint8_t addh, uint8_t addl, uint8_t chan) {
    _target.addh = addh;
    _target.addl = addl;
    _target.chan = chan;
}

bool RH_E220::setTxPower(TxPowerLevel level) {
    Parameters params;
    if (!readParameters(params))
        return false;
    params.opt1 &= ~RH_E220_PARAM_OPT1_TX_POWER_MASK;
    params.opt1 |= (level & RH_E220_PARAM_OPT1_TX_POWER_MASK);
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

bool RH_E220::setDataRate(DataRate rate) {
    Parameters params;
    if (!readParameters(params))
        return false;
    // The DataRate enums are the same values as the register bitmasks
    params.sped &= ~RH_E220_PARAM_SPED_DATA_RATE_MASK;
    params.sped |= (rate & RH_E220_PARAM_SPED_DATA_RATE_MASK);
    return writeParameters(params);
}

#endif // RH_HAVE_SERIAL
