#include <Arduino.h>
#include <SoftwareSerial.h>
#include "hardware.h"
#include "OpenKNX.h"

#include <deque>
#include <functional>
#include <vector>

// DoorSerial encapsulates UART communication with the door controller.
// The protocol uses DLE/STX and DLE/ETX framing with XOR checksum of the
// transmitted (stuffed) payload bytes. Incoming frames are decoded into
// payload-only messages that can be consumed via callback or polling.

class DoorSerial {
private:
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
    DoorSerial();
    
    // Destructor
    ~DoorSerial();
    
    std::string logPrefix();

    // Initialization
    void begin();
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
    void flush();
    void clearReceiveBuffer();
    
    // Periodic transmission methods (removed)
    inline void enablePeriodicSend(const String&, unsigned long = 5000) {}
    inline void disablePeriodicSend() {}
    inline void updatePeriodicSend() {}
    
    void printStatus();
};