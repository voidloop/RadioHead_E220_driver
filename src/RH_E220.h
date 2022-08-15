#ifndef RH_E220_H
#define RH_E220_H

#include <RHGenericDriver.h>

#if (RH_PLATFORM == RH_PLATFORM_STM32F2)
#define HardwareSerial USARTSerial
#elif defined (ARDUINO_ARCH_STM32F4)
#include <libmaple/HardwareSerial.h>
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
#include <UART.h>
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(ARDUINO_attinyxy6)
// AT Tiny Mega 3216 etc
#define HardwareSerial UartClass
#else

#include <HardwareSerial.h>

#endif

// Commands to alter module behaviour
#define RH_E220_COMMAND_WRITE_PARAMS_SAVE         0xC0
#define RH_E220_COMMAND_READ_PARAMS               0xC1
#define RH_E220_COMMAND_WRITE_PARAMS_NOSAVE       0xC2
#define RH_E220_COMMAND_READ_VERSION              0xC3
#define RH_E220_COMMAND_RESET                     0xC4

// Various flags and masks for param bytes
#define RH_E220_PARAM_SPED_UART_MODE_MASK         0x18
#define RH_E220_PARAM_SPED_UART_MODE_8N1          0x08
#define RH_E220_PARAM_SPED_UART_MODE_8O1          0x10
#define RH_E220_PARAM_SPED_UART_MODE_8E1          0x18

#define RH_E220_PARAM_SPED_UART_BAUD_MASK         0xE0
#define RH_E220_PARAM_SPED_UART_BAUD_1200         0x00
#define RH_E220_PARAM_SPED_UART_BAUD_2400         0x20
#define RH_E220_PARAM_SPED_UART_BAUD_4800         0x40
#define RH_E220_PARAM_SPED_UART_BAUD_9600         0x60
#define RH_E220_PARAM_SPED_UART_BAUD_19200        0x80
#define RH_E220_PARAM_SPED_UART_BAUD_38400        0xA0
#define RH_E220_PARAM_SPED_UART_BAUD_57600        0xC0
#define RH_E220_PARAM_SPED_UART_BAUD_115200       0xE0

#define RH_E220_PARAM_SPED_DATARATE_MASK          0x07
#define RH_E220_PARAM_SPED_DATARATE_2400BPS       0x02
#define RH_E220_PARAM_SPED_DATARATE_4800BPS       0x03
#define RH_E220_PARAM_SPED_DATARATE_9600BPS       0x04
#define RH_E220_PARAM_SPED_DATARATE_19200BPS      0x05
#define RH_E220_PARAM_SPED_DATARATE_38400BPS      0x06
#define RH_E220_PARAM_SPED_DATARATE_62500BPS      0x07

#define RH_E220_PARAM_OPT1_SUB_PKT_MASK           0xC0
#define RH_E220_PARAM_OPT1_SUB_PKT_200B           0x00
#define RH_E220_PARAM_OPT1_SUB_PKT_128B           0x40
#define RH_E220_PARAM_OPT1_SUB_PKT_64B            0x80
#define RH_E220_PARAM_OPT1_SUB_PKT_32B            0xC0

#define RH_E220_PARAM_OPT1_POWER_MASK             0x03
#define RH_E220_PARAM_OPT1_POWER_22DBM            0x00
#define RH_E220_PARAM_OPT1_POWER_17DBM            0x01
#define RH_E220_PARAM_OPT1_POWER_13DBM            0x02
#define RH_E220_PARAM_OPT1_POWER_10DBM            0x03

#define RH_E220_CONFIG_BAUD_RATE 9600

// Special characters
#define STX 0x02
#define ETX 0x03
#define DLE 0x10
#define SYN 0x16

// Maximum message length (including the headers) we are willing to support
#define RH_E220_MAX_PAYLOAD_LEN 200

// The length of the headers we add.
// The headers are inside the payload and are therefore protected by the FCS
#define RH_E220_HEADER_LEN 4

#define RH_E220_TARGET_LEN 3
#define RH_E220_DLESTX_LEN 2
#define RH_E220_DLEETX_LEN 2
#define RH_E220_FCS_LEN 2

// This is the maximum message length that can be supported by this library. 
// It is an arbitrary limit.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrpyted
#ifndef RH_E220_MAX_MESSAGE_LEN
#define RH_E220_MAX_MESSAGE_LEN (RH_E220_MAX_PAYLOAD_LEN - RH_E220_TARGET_LEN -\
    RH_E220_DLESTX_LEN - RH_E220_HEADER_LEN - RH_E220_DLEETX_LEN - RH_E220_FCS_LEN)
#endif


/////////////////////////////////////////////////////////////////////
/// \class RH_E220 RH_E220.h <RH_E220.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a stream connection
///
/// This class sends and received packetized messages over a stream connection.
/// It can be used for point-to-point or multidrop, RS232, RS488 or other stream connections as
/// supported by your controller hardware.
/// It can also be used to communicate via radios with stream interfaces such as:
/// - APC220 Radio Data Module http://www.dfrobot.com/image/data/TEL0005/APC220_Datasheet.pdf
///   http://www.dfrobot.com/image/data/TEL0005/APC220_Datasheet.pdf
/// - 3DR Telemetry Radio https://store.3drobotics.com/products/3dr-radio
/// - HopeRF HM-TR module http://www.hoperf.com/upload/rf_app/HM-TRS.pdf
/// - Others
///
/// Compiles and runs on Linux, OSX and all the microprocessers and MCUs suported by
/// radiohead. On Linux and OSX, a RadioHead specific version of HardwareSerial (in RHutil/HardwareSerial.*)
/// encapsulates access to any serial port (or suported USB-stream converter)
///
/// The packetised messages include message encapsulation, headers, a message payload and a checksum.
/// It therefore can support robust binary message passing with error-detection and retransmission
/// when used with the appropriate manager. This allows reliable stream communicaitons even over very long
/// lines where noise might otherwise affect reliablity of the communications.
///
/// \par Packet Format
///
/// All messages sent and received by this RH_E220 Driver conform to this packet format:
/// \code
/// DLE 
/// STX
/// TO Header                (1 octet)
/// FROM Header              (1 octet)
/// ID Header                (1 octet)
/// FLAGS Header             (1 octet)
/// Message payload          (0 to 60 octets)
/// DLE
/// ETX
/// Frame Check Sequence FCS CCITT CRC-16 (2 octets)
/// \endcode
///
/// If any of octets from TO header through to the end of the payload are a DLE, 
/// then they are preceded by a DLE (ie DLE stuffing).
/// The FCS covers everything from the TO header to the ETX inclusive, but not any stuffed DLEs
///
/// \par Physical connection
///
/// The physical connection to your stream port will depend on the type of platform you are on.
///
/// For example, many arduinos only support a single Serial port on pins 0 and 1, 
/// which is shared with the USB host connections. On such Arduinos, it is not possible to use both 
/// RH_E220 on the Serial port as well as using the Serial port for debugand other printing or communications.
/// 
/// On Arduino Mega and Due, there are 4 stream ports:
/// - Serial: this is the stream port connected to the USB interface and the programming host.
/// - Serial1: on pins 18 (Tx) and 19 (Rx)
/// - Serial2: on pins 16 (Tx) and 17 (Rx)
/// - Serial3: on pins 14 (Tx) and 15 (Rx)
///
/// On Uno32, there are 2 stream ports:
/// - SerialUSB: this is the port for the USB host connection.
/// - Serial1: on pins 39 (Rx) and 40 (Tx) 
///
/// On Maple and Flymaple, there are 4 stream ports:
/// - SerialUSB: this is the port for the USB host connection.
/// - Serial1: on pins 7 (Tx) and 8 (Rx)
/// - Serial2: on pins 0 (Rx) and 1 (Tx)
/// - Serial3: on pins 29 (Tx) and 30 (Rx)
///
/// On Linux and OSX there can be any number of stream ports.
/// - On Linux, names like /dev/ttyUSB0 (for a FTDO USB-stream converter)
/// - On OSX, names like /dev/tty.usbserial-A501YSWL (for a FTDO USB-stream converter)
///
/// On STM32 F4 Discovery with Arduino and Arduino_STM32, there are 4 stream ports. We had success with port 2
/// (TX on pin PA2 and RX on pin PA3) and initialising the driver like this:
/// RH_E220 driver(Serial2);
///
/// Note that it is necessary for you to select which Serial port your RF_Serial will use and pass it to the 
/// contructor. On Linux you must pass an instance of HardwareSerial.
///
/// \par Testing
/// 
/// You can test this class and the RHReliableDatagram manager
/// on Unix and OSX with back-to-back connected FTDI USB-stream adapters.
/// Back-to-back means the TX of one is connected to the RX of the other and vice-versa. 
/// You should also join the ground pins.
///
/// Assume the 2 USB-stream adapters are connected by USB
/// and have been assigned device names: 
/// /dev/ttyUSB0 and /dev/ttyUSB1.
/// Build the example RHReliableDatagram client and server programs:
/// \code
/// tools/simBuild examples/stream/serial_reliable_datagram_server/serial_reliable_datagram_server.pde
/// tools/simBuild examples/stream/serial_reliable_datagram_client/serial_reliable_datagram_client.pde
/// \endcode
/// In one window run the server, specifying the device to use as an environment variable:
/// \code
/// RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB1 ./serial_reliable_datagram_server 
/// \endcode
/// And in another window run the client, specifying the other device to use as an environment variable:
/// \code
/// RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB0 ./serial_reliable_datagram_client 
/// \endcode
/// You should see the 2 programs passing messages to each other.
/// 
class RH_E220 : public RHGenericDriver {
public:
    /// Constructor
    /// \param[in] serial Reference to the HardwareSerial port which will be used by this instance.
    RH_E220(HardwareSerial &serial, uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin);

    /// Return the HardwareSerial port in use by this instance
    /// \return The current HardwareSerial as a reference
    HardwareSerial &serial();

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool init() override;

    /// Tests whether a new message is available
    /// This can be called multiple times in a timeout loop.
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    bool available() override;

    /// Wait until a new message is available from the driver.
    /// Blocks until a complete message is received as reported by available()
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
//    virtual void waitAvailable(uint16_t polldelay = 0);

    /// Wait until a new message is available from the driver or the timeout expires.
    /// Blocks until a complete message is received as reported by available() or the timeout expires.
    /// \param[in] timeout The maximum time to wait in milliseconds
    /// \param[in] polldelay Time between polling available() in milliseconds. This can be useful
    /// in multitaking environment like Linux to prevent waitAvailableTimeout
    /// using all the CPU while polling for receiver activity
    /// \return true if a message is available as reported by available(), false on timeout.
//    virtual bool waitAvailableTimeout(uint16_t timeout, uint16_t polldelay = 0);

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
    virtual uint8_t maxMessageLength();

    bool isChannelActive() override;

    /// \brief Values to be passed to setDataRate() to control the on-air data rate
    ///
    /// This is NOT to be used to control the baud rate of the stream connection to the radio
    typedef enum {
        DataRate2400bps = RH_E220_PARAM_SPED_DATARATE_2400BPS,
        DataRate4800bps = RH_E220_PARAM_SPED_DATARATE_4800BPS,
        DataRate9600bps = RH_E220_PARAM_SPED_DATARATE_9600BPS,
        DataRate19200bps = RH_E220_PARAM_SPED_DATARATE_19200BPS,
        DataRate38400bps = RH_E220_PARAM_SPED_DATARATE_38400BPS,
        DataRate62500bps = RH_E220_PARAM_SPED_DATARATE_62500BPS,
    } DataRate;

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
        Power22dBm = RH_E220_PARAM_OPT1_POWER_22DBM,
        Power17dBm = RH_E220_PARAM_OPT1_POWER_17DBM,
        Power13dBm = RH_E220_PARAM_OPT1_POWER_13DBM,
        Power10dBm = RH_E220_PARAM_OPT1_POWER_10DBM,
    } PowerLevel;

    /// Sets the transmitter power output
    /// \param[in] level A valid power setting from the Power enum
    /// \return true if successful
    bool setPower(PowerLevel level);

    bool setAddress(uint8_t addh, uint8_t addl);

    bool setChannel(uint8_t chan);

    /// \brief Values to be passed to setSubPkt() to control the sub-packet settings
    ///
    typedef enum {
        SubPacket200B = RH_E220_PARAM_OPT1_SUB_PKT_200B,
        SubPacket128B = RH_E220_PARAM_OPT1_SUB_PKT_128B,
        SubPacket64B = RH_E220_PARAM_OPT1_SUB_PKT_64B,
        SubPacket32B = RH_E220_PARAM_OPT1_SUB_PKT_32B,
    } SubPacketLen;

    bool setSubPacket(SubPacketLen len);

protected:
    /// \brief Defines different receiver states in teh receiver state machine
    typedef enum {
        RxStateInitialising = 0,  ///< Before init() is called
        RxStateIdle,              ///< Waiting for an STX
        RxStateDLE,               ///< Waiting for the DLE after STX
        RxStateData,              ///< Receiving data
        RxStateEscape,            ///< Got a DLE while receiving data.
        RxStateWaitFCS1,          ///< Got DLE ETX, waiting for first FCS octet
        RxStateWaitFCS2           ///< Waiting for second FCS octet
    } RxState;

    /// \brief Defines values to be passed to setOperatinMode
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

    /// HAndle a character received from the stream port. IMplements
    /// the receiver state machine
    void handleRx(uint8_t ch);

    /// Empties the Rx buffer
    void clearRxBuf();

    /// Waits for the AUX pin to go high
    /// For internal use only
    void waitAuxHigh() const;

    /// Adds a charater to the Rx buffer
    void appendRxBuf(uint8_t ch);

    /// Checks whether the Rx buffer contains valid data that is complete and uncorrupted
    /// Check the FCS, the TO address, and extracts the headers
    void validateRxBuf();

    /// Sends a single data octet to the stream port.
    /// Implements DLE stuffing and keeps track of the senders FCS
    void txData(uint8_t ch);

    /// Reference to the Stream we will use
    HardwareSerial &_serial;

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

    uint8_t _targetAddh;

    uint8_t _targetAddl;

    uint8_t _targetChan;
};

#endif
