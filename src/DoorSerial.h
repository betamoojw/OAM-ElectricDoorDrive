#ifndef DOOR_SERIAL_H
#define DOOR_SERIAL_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "hardware.h"

#include <deque>
#include <functional>
#include <vector>

// DoorSerial encapsulates UART communication with the door controller.
// The protocol uses DLE/STX and DLE/ETX framing with XOR checksum of the
// transmitted (stuffed) payload bytes. Incoming frames are decoded into
// payload-only messages that can be consumed via callback or polling.

class DoorSerial {
private:
    SoftwareSerial* serialPort;
    uint8_t rxPin;
    uint8_t txPin;
    unsigned long baudRate;

    enum class RxState : uint8_t {
        Idle,
        AwaitStx,
        InFrame,
        AfterDle,
        AwaitChecksum
    };

    RxState rxState;
    uint8_t computedChecksum;
    std::vector<uint8_t> rxBuffer;
    std::deque<std::vector<uint8_t>> messageQueue;

    std::function<void(const std::vector<uint8_t>&)> messageCallback;

    void resetState();
    void handleIncomingByte(uint8_t byte);
    void enqueueMessage(const std::vector<uint8_t>& message);

    static constexpr uint8_t DLE = 0x10;
    static constexpr uint8_t STX = 0x02;
    static constexpr uint8_t ETX = 0x03;
    static constexpr size_t MAX_QUEUE_DEPTH = 4;
    static constexpr size_t MAX_MESSAGE_LENGTH = 128;
    
public:
    static constexpr size_t MaxMessageLength = MAX_MESSAGE_LENGTH;

    // Constructor
    DoorSerial(uint8_t rx_pin = MAIN_DOOR_RX_PIN, uint8_t tx_pin = MAIN_DOOR_TX_PIN, unsigned long baud = MAIN_DOOR_SERIAL_BAUD);
    
    // Destructor
    ~DoorSerial();
    
    // Initialization
    bool begin();
    void end();

    // Processing
    void poll();
    bool hasMessage() const;
    size_t readMessage(uint8_t* buffer, size_t maxLength);
    
    // Communication methods
    bool sendPayload(const uint8_t* payload, size_t length);
    bool sendPayload(const std::vector<uint8_t>& payload);
    void setMessageCallback(std::function<void(const std::vector<uint8_t>&)> callback);
    
    // Legacy helpers (for compatibility)
    inline bool hasData() const { return hasMessage(); }
    inline void sendBinaryData(const uint8_t* data, size_t length) { sendPayload(data, length); }
    inline size_t readBinaryData(uint8_t* buffer, size_t maxLength) { return readMessage(buffer, maxLength); }
    
    // Utility methods
    bool isConnected() const;
    void flush();
    void clearReceiveBuffer();
    size_t availableForWrite() const;
    
    // Periodic transmission methods (removed)
    inline void enablePeriodicSend(const String&, unsigned long = 5000) {}
    inline void disablePeriodicSend() {}
    inline void updatePeriodicSend() {}
    
    // Configuration methods
    void setBaudRate(unsigned long baud);
    void setPins(uint8_t rx_pin, uint8_t tx_pin);
    
    // Status methods
    uint8_t getTxPin() const { return txPin; }
    uint8_t getRxPin() const { return rxPin; }
    unsigned long getBaudRate() const { return baudRate; }
    bool isInitialized() const { return (serialPort != nullptr); }
    
    void printStatus();
};

#endif // DOOR_SERIAL_H