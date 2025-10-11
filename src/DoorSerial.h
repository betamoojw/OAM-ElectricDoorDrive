#ifndef DOOR_SERIAL_H
#define DOOR_SERIAL_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "hardware.h"

class DoorSerial {
private:
    SoftwareSerial* serialPort;
    uint8_t txPin;
    uint8_t rxPin;
    unsigned long baudRate;
    unsigned long lastPeriodicSend;
    bool periodicSendEnabled;
    String periodicMessage;
    
public:
    // Constructor
    DoorSerial(uint8_t tx_pin = 4, uint8_t rx_pin = 5, unsigned long baud = 115200);
    
    // Destructor
    ~DoorSerial();
    
    // Initialization
    bool begin();
    void end();
    
    // Basic communication methods
    void sendMessage(const String& message);
    String readMessage();
    bool hasData();
    
    // Binary data methods
    void sendBinaryData(const uint8_t* data, size_t length);
    size_t readBinaryData(uint8_t* buffer, size_t maxLength);
    
    // Utility methods
    bool isConnected();
    void flush();
    void clearReceiveBuffer();
    size_t availableForWrite();
    
    // Periodic transmission methods
    void enablePeriodicSend(const String& message, unsigned long intervalMs = 5000);
    void disablePeriodicSend();
    void updatePeriodicSend(); // Call this in loop()
    
    // Configuration methods
    void setBaudRate(unsigned long baud);
    void setPins(uint8_t tx_pin, uint8_t rx_pin);
    
    // Status methods
    uint8_t getTxPin() const { return txPin; }
    uint8_t getRxPin() const { return rxPin; }
    unsigned long getBaudRate() const { return baudRate; }
    bool isInitialized() const { return (serialPort != nullptr); }
    
    void printStatus();
};

#endif // DOOR_SERIAL_H