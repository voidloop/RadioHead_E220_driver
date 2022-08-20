#pragma once

#include <RH_Serial.h>

// Commands to alter module behaviour
#define RH_E220_COMMAND_WRITE_PARAMS_SAVE         0xC0
#define RH_E220_COMMAND_READ_PARAMS               0xC1
#define RH_E220_COMMAND_WRITE_PARAMS_NOSAVE       0xC2
#define RH_E220_COMMAND_READ_VERSION              0xC3
#define RH_E220_COMMAND_RESET                     0xC4

// Various flags and masks for param bytes
#define RH_E220_PARAM_SPED_UART_BAUD_MASK         0xE0
#define RH_E220_PARAM_SPED_UART_BAUD_1200         0x00
#define RH_E220_PARAM_SPED_UART_BAUD_2400         0x20
#define RH_E220_PARAM_SPED_UART_BAUD_4800         0x40
#define RH_E220_PARAM_SPED_UART_BAUD_9600         0x60
#define RH_E220_PARAM_SPED_UART_BAUD_19200        0x80
#define RH_E220_PARAM_SPED_UART_BAUD_38400        0xA0
#define RH_E220_PARAM_SPED_UART_BAUD_57600        0xC0
#define RH_E220_PARAM_SPED_UART_BAUD_115200       0xE0

#define RH_E220_PARAM_SPED_UART_MODE_MASK         0x18
#define RH_E220_PARAM_SPED_UART_MODE_8N1          0x00
#define RH_E220_PARAM_SPED_UART_MODE_8O1          0x08
#define RH_E220_PARAM_SPED_UART_MODE_8E1          0x18

#define RH_E220_PARAM_SPED_DATA_RATE_MASK         0x07
#define RH_E220_PARAM_SPED_DATA_RATE_2400         0x02
#define RH_E220_PARAM_SPED_DATA_RATE_4800         0x03
#define RH_E220_PARAM_SPED_DATA_RATE_9600         0x04
#define RH_E220_PARAM_SPED_DATA_RATE_19200        0x05
#define RH_E220_PARAM_SPED_DATA_RATE_38400        0x06
#define RH_E220_PARAM_SPED_DATA_RATE_62500        0x07

#define RH_E220_PARAM_OPT1_PACKET_LEN_MASK        0xC0
#define RH_E220_PARAM_OPT1_PACKET_LEN_200         0x00
#define RH_E220_PARAM_OPT1_PACKET_LEN_128         0x40
#define RH_E220_PARAM_OPT1_PACKET_LEN_64          0x80
#define RH_E220_PARAM_OPT1_PACKET_LEN_32          0xC0

#define RH_E220_PARAM_OPT1_RSSI_NOISE_MASK        0x20
#define RH_E220_PARAM_OPT1_RSSI_NOISE_DISABLE     0x00
#define RH_E220_PARAM_OPT1_RSSI_NOISE_ENABLE      0x20

#define RH_E220_PARAM_OPT1_TX_POWER_MASK          0x03
#define RH_E220_PARAM_OPT1_TX_POWER_22            0x00
#define RH_E220_PARAM_OPT1_TX_POWER_17            0x01
#define RH_E220_PARAM_OPT1_TX_POWER_13            0x02
#define RH_E220_PARAM_OPT1_TX_POWER_10            0x03

#define RH_E220_PARAM_OPT2_RSSI_BYTE_MASK         0x80
#define RH_E220_PARAM_OPT2_RSSI_BYTE_DISABLE      0x00
#define RH_E220_PARAM_OPT2_RSSI_BYTE_ENABLE       0x80

#define RH_E220_PARAM_OPT2_TX_METHOD_MASK         0x40
#define RH_E220_PARAM_OPT2_TX_METHOD_TRANSPARENT  0x00
#define RH_E220_PARAM_OPT2_TX_METHOD_FIXED        0x40

#define RH_E220_PARAM_OPT2_LTB_MASK               0x10
#define RH_E220_PARAM_OPT2_LTB_DISABLE            0x00
#define RH_E220_PARAM_OPT2_LTB_ENABLE             0x10

#define RH_E220_PARAM_OPT2_WOR_CYCLE_MASK         0x07
#define RH_E220_PARAM_OPT2_WOR_CYCLE_500          0x00
#define RH_E220_PARAM_OPT2_WOR_CYCLE_1000         0x01
#define RH_E220_PARAM_OPT2_WOR_CYCLE_1500         0x02
#define RH_E220_PARAM_OPT2_WOR_CYCLE_2000         0x03
#define RH_E220_PARAM_OPT2_WOR_CYCLE_2500         0x04
#define RH_E220_PARAM_OPT2_WOR_CYCLE_3000         0x05
#define RH_E220_PARAM_OPT2_WOR_CYCLE_3500         0x06
#define RH_E220_PARAM_OPT2_WOR_CYCLE_4000         0x07

// Defaults
#define RH_E220_DEFAULT_ADDRESS_HIGH    0xFF
#define RH_E220_DEFAULT_ADDRESS_LOW     0xFF
#define RH_E220_DEFAULT_CHANNEL         0x17
#define RH_E220_DEFAULT_WOR_CYCLE       RH_E220_PARAM_OPT2_WOR_CYCLE_2000
#define RH_E220_DEFAULT_TX_POWER        RH_E220_PARAM_OPT1_TX_POWER_10
#define RH_E220_DEFAULT_DATA_RATE       RH_E220_PARAM_SPED_DATA_RATE_2400
#define RH_E220_DEFAULT_UART_MODE       RH_E220_PARAM_SPED_UART_MODE_8N1
#define RH_E220_DEFAULT_UART_BAUD       RH_E220_PARAM_SPED_UART_BAUD_9600

#define RH_E220_CONFIG_UART_BAUD        9600

// This for activate the RSSI byte
// (define? or let this option editable at runtime?)
#define RH_E220_RSSI_BYTE_ENABLED

class RH_E220_Serial : public RH_Serial {
public:
    /// Constructor
    /// \param[in] serial Reference to the HardwareSerial port which will be used by this instance.
    /// \param[in] m0Pin Pin number of the Arduino pin that connects to the radio M0 input
    /// \param[in] m1Pin Pin number of the Arduino pin that connects to the radio M1 input
    /// \param[in] auxPin Pin number of the Arduino pin that connects to the radio AUX output
    RH_E220_Serial(HardwareSerial &serial, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool init() override;

    /// Override the recv function to extract RSSI byte (If enabled)
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool recv(uint8_t *buf, uint8_t *len) override;

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent.
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    bool send(const uint8_t* data, uint8_t len) override;

    /// Sets the tarnsmitter and receiver channel.
    /// \param[in] frequency Desired channel (See EBYTE documentation)
    /// \return true if successful
    bool setChannel(uint8_t chan);

    /// Waits for any currently transmitting packet to be completely sent
    /// Returns true if successful
    bool waitPacketSent() override;

protected:
    /// Defines values to be passed to setOperatingMode
    /// For internal driver user only
    typedef enum {
        ModeNormal = 0,    ///< Normal mode for sending and receiving messages
        ModeWakeUp,        ///< Adds a long preamble to transmission to allow destination receivers to wake up
        ModePowerSaving,   ///< Receiver sleeps until a message is received
        ModeSleep          ///< Use during parameter setting
    } OperatingMode;

    /// Sets the operating mode of the radio.
    /// For internal use only
    void setOperatingMode(OperatingMode mode) const;

    /// Waits for the AUX pin to go high
    /// For internal use only
    void waitAuxHigh() const;

    /// Structure for reading and writing radio control parameters
    /// For internal driver user only
    typedef struct {
        uint8_t addh;      ///< High address byte (not used by this driver)
        uint8_t addl;      ///< Low address byte (not used by this driver)
        uint8_t sped;      ///< Data and baud rate parameters
        uint8_t opt1;      ///< Various control options
        uint8_t chan;      ///< Radio channel
        uint8_t opt2;      ///< Various control options
    } Parameters;

    /// Read the radio configuration parameters into local memory
    /// \param[in] params Reference to a Parameter structure which will be filled if successful
    /// \return true if successful
    bool readParameters(Parameters &params);

    /// Write radio configuration parameters from local memory
    /// to the radio. You can choose whether the parameter will be saved across power down or not
    /// \param[in] params Reference to a Parameter structure containing the radio configuration parameters
    /// to be written to the radio.
    /// \param[in] save If true, the parameters will be saved across power down in the radio
    /// \return true if successful
    bool writeParameters(Parameters &params, bool save = false);

private:
    uint8_t _m0Pin;
    uint8_t _m1Pin;
    uint8_t _auxPin;
};