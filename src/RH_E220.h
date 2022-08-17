#ifndef RH_E220_H
#define RH_E220_H

#include <RHGenericDriver.h>
#include <Stream.h>

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

// Reserved for what?
#define RH_E220_PARAM_OPT1_RESERVED_MASK          0x1C
#define RH_E220_PARAM_OPT2_RESERVED_MASK          0x28


// Defaults, set in init()
#define RH_E220_DEFAULT_POWER       RH_E220_PARAM_OPT1_TX_POWER_10
#define RH_E220_DEFAULT_DATA_RATE   RH_E220_PARAM_SPED_DATA_RATE_2400
#define RH_E220_DEFAULT_UART_MODE   RH_E220_PARAM_SPED_UART_MODE_8N1
#define RH_E220_DEFAULT_UART_BAUD   RH_E220_PARAM_SPED_UART_BAUD_9600

#define RH_E220_CONFIG_UART_BAUD    9600


// The length of the headers we add.
// The headers are inside the payload and are therefore protected by the FCS
#define RH_E220_HEADER_LEN 4

// This is the preamble octet.
#define PREAMBLE 0xAA

// Maximum message length of the packet that can be supported by this driver.
// +----------------+-------------------+-------+
// | FRAME          | PAYLOAD           | FRAME |
// +----------+-----+---------+---------+-------+
// | PREAMBLE | LEN | HEADER  | MESSAGE | FCS   |
// +----------+-----+---------+---------+-------+
// | 3        | 1   | 4       | 187     | 2     |
// +----------+-----+---------+---------+-------+
// The first 3 octets (before PREAMBLE) are the target.

#define RH_E220_MAX_PAYLOAD_LEN   191 // HEADER + MESSAGE

// This is the maximum message length that can be supported by this library.
// It is an arbitrary limit.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrypted
#ifndef RH_E220_MAX_MESSAGE_LEN
#define RH_E220_MAX_MESSAGE_LEN (RH_E220_MAX_PAYLOAD_LEN - RH_E220_HEADER_LEN)
#endif

/////////////////////////////////////////////////////////////////////
/// \class RH_E220 RH_E220.h <RH_E220.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a stream connection
///
/// \par Packet Format
///
/// All messages sent and received by this Driver conform to this packet format:
///
/// - 5 octets HEADER: (LENGTH,  TO, FROM, ID, FLAGS)
/// - 0 to 53 octets DATA
///

class RH_E220 : public RHGenericDriver {
public:
    /// Constructor
    /// \param[in] stream Reference to the Stream which will be used by this instance.
    RH_E220(Stream &stream, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool init() override;

    /// Tests whether a new message is available
    /// This can be called multiple times in a timeout loop.
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    bool available() override;

    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool recv(uint8_t *buf, uint8_t *len) override;

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    bool send(const uint8_t *data, uint8_t len) override;

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    uint8_t maxMessageLength() override;

    /// Determine if the currently selected radio channel is active.
    /// This is expected to be subclassed by specific radios to implement their Channel Activity Detection
    /// if supported. If the radio does not support CAD, returns true immediately. If a RadioHead radio
    /// supports isChannelActive() it will be documented in the radio specific documentation.
    /// This is called automatically by waitCAD().
    /// \return true if the radio-specific CAD (as returned by override of isChannelActive()) shows the
    /// current radio channel as active, else false. If there is no radio-specific CAD, returns false.
    bool isChannelActive() override;

    /// \brief Values to be passed to setDataRate() to control the on-air data rate
    ///
    /// This is NOT to be used to control the baud rate of the stream connection to the radio
    typedef enum {
        DataRate2400bps = RH_E220_PARAM_SPED_DATA_RATE_2400,
        DataRate4800bps = RH_E220_PARAM_SPED_DATA_RATE_4800,
        DataRate9600bps = RH_E220_PARAM_SPED_DATA_RATE_9600,
        DataRate19200bps = RH_E220_PARAM_SPED_DATA_RATE_19200,
        DataRate38400bps = RH_E220_PARAM_SPED_DATA_RATE_38400,
        DataRate62500bps = RH_E220_PARAM_SPED_DATA_RATE_62500,
    } DataRate;

    /// Sets the on-air data rate to be used by the transmitter and receiver
    /// \param[in] rate A valid data rate from the DataRate enum
    /// \return true if successful
    bool setDataRate(DataRate rate);

    /// \brief Values to be passed to setBaudRate() to control the radio stream connection baud rate
    ///
    /// This is NOT to be used to control the on-air data rate the radio transmits and receives at
    typedef enum {
        BaudRate1200 = RH_E220_PARAM_SPED_UART_BAUD_1200,
        BaudRate2400 = RH_E220_PARAM_SPED_UART_BAUD_2400,
        BaudRate4800 = RH_E220_PARAM_SPED_UART_BAUD_4800,
        BaudRate9600 = RH_E220_PARAM_SPED_UART_BAUD_9600,
        BaudRate19200 = RH_E220_PARAM_SPED_UART_BAUD_19200,
        BaudRate38400 = RH_E220_PARAM_SPED_UART_BAUD_38400,
        BaudRate57600 = RH_E220_PARAM_SPED_UART_BAUD_57600,
        BaudRate115200 = RH_E220_PARAM_SPED_UART_BAUD_115200,
    } BaudRate;

    /// \brief Values to be passed to setBaudRate() to control the parity of the stream connection to the radio
    typedef enum {
        Parity8N1 = RH_E220_PARAM_SPED_UART_MODE_8N1,
        Parity8O1 = RH_E220_PARAM_SPED_UART_MODE_8O1,
        Parity8E1 = RH_E220_PARAM_SPED_UART_MODE_8E1,
    } Parity;

    /// Sets the radio stream port baud rate and parity (not the on-air data rate)
    /// Set also the Arduino rate or parity
    /// \param[in] rate A valid baud rate from the BaudRate enum
    /// \param[in] parity A valid parity from the PArity enum
    /// \return true if successful
    bool setBaudRate(BaudRate rate = BaudRate9600, Parity parity = Parity8N1);

    void setTarget(uint8_t addh, uint8_t addl, uint8_t chan);

    /// \brief Values to be passed to setPower() to control the transmitter power
    ///
    typedef enum {
        Power22dBm = RH_E220_PARAM_OPT1_TX_POWER_22,
        Power17dBm = RH_E220_PARAM_OPT1_TX_POWER_17,
        Power13dBm = RH_E220_PARAM_OPT1_TX_POWER_13,
        Power10dBm = RH_E220_PARAM_OPT1_TX_POWER_10,
    } PowerLevel;

    /// Sets the transmitter power output
    /// \param[in] level A valid power setting from the Power enum
    /// \return true if successful
    bool setPower(PowerLevel level);

    bool setAddress(uint8_t addh, uint8_t addl);

    bool setChannel(uint8_t chan);

protected:
    /// \brief Defines different receiver states in teh receiver state machine
    typedef enum {
        RxStateInitialising = 0,  ///< Before init() is called
        RxStateIdle,              ///< Waiting for an first PREAMBLE
        RxStatePreamble1,         ///< Waiting for an second PREAMBLE
        RxStatePreamble2,         ///< Waiting for an third PREAMBLE
        RxStateLength,            ///< Got the length of receiving data
        RxStateData,              ///< Receiving data
        RxStateWaitFCS1,          ///< Got DLE ETX, waiting for first FCS octet
        RxStateWaitFCS2           ///< Waiting for second FCS octet
    } RxState;

    /// \brief Defines values to be passed to setOperatingMode
    ///
    /// For internal driver user only
    typedef enum {
        ModeNormal = 0,    ///< Normal mode for sending and receiving messages
        ModeWakeUp,        ///< Adds a long preamble to transmission to allow destination receivers to wake up
        ModePowerSaving,   ///< Receiver sleeps until a message is received
        ModeSleep          ///< Use during parameter setting
    } OperatingMode;

    /// Sets the operating mode of the radio.
    /// For internal use only
    void setOperatingMode(OperatingMode mode);

    /// \brief Structure for reading and writing radio control parameters
    ///
    /// For internal driver user only
    typedef struct {
        uint8_t addh;      ///< High address byte (not used by this driver)
        uint8_t addl;      ///< Low address byte (not used by this driver)
        uint8_t sped;      ///< Data and baud rate parameters
        uint8_t opt1;      ///< Various control options
        uint8_t chan;      ///< Radio channel
        uint8_t opt2;      ///< Various control options
    } Parameters;

    /// Read the radio configuration parameters into
    /// local memory
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

    /// Waits for the AUX pin to go high
    /// For internal use only
    void waitAuxHigh() const;

    /// Handle a character received from the stream port. Implements the receiver state machine
    void handleRx(uint8_t ch);

    /// Empties the Rx buffer
    void clearRxBuf();

    /// Adds a character to the Rx buffer
    void appendRxBuf(uint8_t ch);

    /// Checks whether the Rx buffer contains valid data that is complete and uncorrupted
    /// Check the FCS, the TO address, and extracts the headers
    void validateRxBuf();

    /// Sends a single data octet to the stream port.
    /// Implements DLE stuffing and keeps track of the senders FCS
    void txData(uint8_t ch);

private:

    /// Reference to the Stream we will use
    Stream &_stream;

    /// The current state of the Rx state machine
    RxState _rxState;

    /// Progressive FCS calc (CCITT CRC-16 covering all received data (but not stuffed DLEs), plus trailing DLE, ETX)
    uint16_t _rxFcs;

    /// The received FCS at the end of the current message
    uint16_t _rxRecdFcs;

    /// The Rx buffer
    uint8_t _rxBuf[RH_E220_MAX_PAYLOAD_LEN];

    /// Current length of data in the Rx buffer
    uint8_t _rxBufLen;

    /// True if the data in the Rx buffer is value and uncorrupted and complete message is available for collection
    bool _rxBufValid;

    /// FCS for transmitted data
    uint16_t _txFcs;

    uint8_t _auxPin;

    uint8_t _m0Pin;

    uint8_t _m1Pin;

    typedef struct {
        uint8_t addh;
        uint8_t addl;
        uint8_t chan;
    } Target;

    Target _target;
};

#endif
