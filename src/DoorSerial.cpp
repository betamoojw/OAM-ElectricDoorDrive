#include "DoorSerial.h"

// Constructor
DoorSerial::DoorSerial(uint8_t tx_pin, uint8_t rx_pin, unsigned long baud) 
    : txPin(tx_pin), rxPin(rx_pin), baudRate(baud), lastPeriodicSend(0), 
      periodicSendEnabled(false), periodicMessage(""), serialPort(nullptr) {
}

// Destructor
DoorSerial::~DoorSerial() {
    end();
}

// Initialize UART communication
bool DoorSerial::begin() {
    serialPort = new MAIN_DOOR_SERIAL(rxPin, txPin);
    if (serialPort != nullptr) {
        end(); // Clean up existing connection
    }
    
    if (serialPort == nullptr) {
        return false;
    }
    
    // Configure UART: 115200 baud, 8 data bits, even parity, 1 stop bit
    serialPort->begin(baudRate, SERIAL_8E1);
    
    // Wait a moment for initialization
    delay(10);
    
    if (Serial) {
        Serial.println("DoorSerial: UART initialized successfully!");
        Serial.printf("DoorSerial: TX Pin: %d, RX Pin: %d, Baud: %lu\n", 
                        txPin, rxPin, baudRate);
    }
    
    return true;
}

// End UART communication
void DoorSerial::end() {
    if (serialPort != nullptr) {
        serialPort->end();
        delete serialPort;
        serialPort = nullptr;
    }
    periodicSendEnabled = false;
}

// Send string message
void DoorSerial::sendMessage(const String& message) {
    if (serialPort == nullptr) {
        if (Serial) Serial.println("DoorSerial: Error - Not initialized!");
        return;
    }
    
    if (Serial) {
        Serial.println("DoorSerial: Sending - " + message);
    }
    
    serialPort->println(message);
}

// Read string message
String DoorSerial::readMessage() {
    if (serialPort == nullptr || !serialPort->available()) {
        return "";
    }
    
    String receivedData = serialPort->readStringUntil('\n');
    receivedData.trim(); // Remove trailing whitespace
    
    if (Serial && receivedData.length() > 0) {
        Serial.println("DoorSerial: Received - " + receivedData);
    }
    
    return receivedData;
}

// Check if data is available
bool DoorSerial::hasData() {
    return (serialPort != nullptr && serialPort->available() > 0);
}

// Send binary data
void DoorSerial::sendBinaryData(const uint8_t* data, size_t length) {
    if (serialPort == nullptr || data == nullptr) {
        if (Serial) Serial.println("DoorSerial: Error - Invalid parameters for binary send!");
        return;
    }
    
    if (Serial) {
        Serial.printf("DoorSerial: Sending binary data (%zu bytes)\n", length);
    }
    
    serialPort->write(data, length);
}

// Read binary data
size_t DoorSerial::readBinaryData(uint8_t* buffer, size_t maxLength) {
    if (serialPort == nullptr || buffer == nullptr) {
        return 0;
    }
    
    size_t bytesRead = 0;
    
    while (serialPort->available() && bytesRead < maxLength) {
        buffer[bytesRead] = serialPort->read();
        bytesRead++;
    }
    
    if (Serial && bytesRead > 0) {
        Serial.printf("DoorSerial: Read %zu binary bytes\n", bytesRead);
    }
    
    return bytesRead;
}

// Check if UART is connected (simple test)
bool DoorSerial::isConnected() {
    if (serialPort == nullptr) {
        return false;
    }
    
    // Simple connectivity test - check if we can write
    size_t initialAvailable = serialPort->availableForWrite();
    serialPort->write((uint8_t)0x00); // Send null byte
    delay(1);
    
    return (serialPort->availableForWrite() != initialAvailable);
}

// Flush UART buffers
void DoorSerial::flush() {
    if (serialPort != nullptr) {
        serialPort->flush(); // Wait for transmission to complete
    }
}

// Clear receive buffer
void DoorSerial::clearReceiveBuffer() {
    if (serialPort == nullptr) return;
    
    while (serialPort->available()) {
        serialPort->read();
    }
    
    if (Serial) {
        Serial.println("DoorSerial: Receive buffer cleared");
    }
}

// Get available space in write buffer
size_t DoorSerial::availableForWrite() {
    return (serialPort != nullptr) ? serialPort->availableForWrite() : 0;
}

// Enable periodic message sending
void DoorSerial::enablePeriodicSend(const String& message, unsigned long intervalMs) {
    periodicMessage = message;
    periodicSendEnabled = true;
    lastPeriodicSend = millis();
    
    if (Serial) {
        Serial.printf("DoorSerial: Periodic send enabled - \"%s\" every %lu ms\n", 
                     message.c_str(), intervalMs);
    }
}

// Disable periodic message sending
void DoorSerial::disablePeriodicSend() {
    periodicSendEnabled = false;
    
    if (Serial) {
        Serial.println("DoorSerial: Periodic send disabled");
    }
}

// Update periodic sending (call in main loop)
void DoorSerial::updatePeriodicSend() {
    if (!periodicSendEnabled || serialPort == nullptr) {
        return;
    }
    
    static unsigned long periodicInterval = 5000; // Default 5 seconds
    
    if (millis() - lastPeriodicSend >= periodicInterval) {
        sendMessage(periodicMessage);
        lastPeriodicSend = millis();
    }
}

// Set baud rate (requires restart)
void DoorSerial::setBaudRate(unsigned long baud) {
    baudRate = baud;
    
    if (Serial) {
        Serial.printf("DoorSerial: Baud rate set to %lu (restart required)\n", baud);
    }
    
    // If already initialized, restart with new baud rate
    if (serialPort != nullptr) {
        begin();
    }
}

// Set pins (requires restart)
void DoorSerial::setPins(uint8_t tx_pin, uint8_t rx_pin) {
    txPin = tx_pin;
    rxPin = rx_pin;
    
    if (Serial) {
        Serial.printf("DoorSerial: Pins set to TX:%d, RX:%d (restart required)\n", 
                     tx_pin, rx_pin);
    }
    
    // If already initialized, restart with new pins
    if (serialPort != nullptr) {
        begin();
    }
}

// Print status information
void DoorSerial::printStatus() {
    if (!Serial) return;
    
    Serial.println("=== DoorSerial Status ===");
    Serial.printf("Initialized: %s\n", isInitialized() ? "Yes" : "No");
    Serial.printf("TX Pin: %d\n", txPin);
    Serial.printf("RX Pin: %d\n", rxPin);
    Serial.printf("Baud Rate: %lu\n", baudRate);
    
    if (serialPort != nullptr) {
        Serial.printf("Data Available: %d bytes\n", serialPort->available());
        Serial.printf("Write Buffer: %zu bytes available\n", serialPort->availableForWrite());
        Serial.printf("Connected: %s\n", isConnected() ? "Yes" : "No");
    }
    
    Serial.printf("Periodic Send: %s\n", periodicSendEnabled ? "Enabled" : "Disabled");
    if (periodicSendEnabled) {
        Serial.printf("Periodic Message: \"%s\"\n", periodicMessage.c_str());
    }
    Serial.println("========================");
}