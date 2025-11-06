#include "DoorSerial.h"

#include <algorithm>
#include <utility>

DoorSerial::DoorSerial(uint8_t rx_pin, uint8_t tx_pin, unsigned long baud)
    : serialPort(nullptr),
      rxPin(rx_pin),
      txPin(tx_pin),
      baudRate(baud),
      rxState(RxState::Idle),
      computedChecksum(0) {}

DoorSerial::~DoorSerial() {
    end();
}

bool DoorSerial::begin() {
    if (serialPort != nullptr) {
        end();
    }

    serialPort = new MAIN_DOOR_SERIAL(rxPin, txPin);
    if (serialPort == nullptr) {
        return false;
    }

    serialPort->begin(baudRate, MAIN_DOOR_SERIAL_CONFIG);
    delay(10);

    resetState();
    clearReceiveBuffer();

    if (Serial) {
        Serial.println("DoorSerial: UART initialized successfully");
        Serial.printf("DoorSerial: RX Pin: %d, TX Pin: %d, Baud: %lu\n", rxPin, txPin, baudRate);
    }

    return true;
}

void DoorSerial::end() {
    if (serialPort != nullptr) {
        serialPort->end();
        delete serialPort;
        serialPort = nullptr;
    }

    messageQueue.clear();
    resetState();
}

void DoorSerial::poll() {
    if (serialPort == nullptr) {
        return;
    }

    while (serialPort->available()) {
        const uint8_t byte = serialPort->read();
        handleIncomingByte(byte);
    }
}

bool DoorSerial::hasMessage() const {
    return !messageQueue.empty();
}

size_t DoorSerial::readMessage(uint8_t* buffer, size_t maxLength) {
    if (buffer == nullptr || maxLength == 0) {
        return 0;
    }

    poll();

    if (messageQueue.empty()) {
        return 0;
    }

    const std::vector<uint8_t>& next = messageQueue.front();
    if (next.size() > maxLength) {
        if (Serial) {
            Serial.printf("DoorSerial: Message too large for buffer (%zu > %zu)\n", next.size(), maxLength);
        }
        return 0;
    }

    std::copy(next.begin(), next.end(), buffer);
    const size_t length = next.size();
    messageQueue.pop_front();

    return length;
}

bool DoorSerial::sendPayload(const uint8_t* payload, size_t length) {
    if (serialPort == nullptr || payload == nullptr) {
        if (Serial) {
            Serial.println("DoorSerial: Cannot send payload (serial not initialized or payload null)");
        }
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(length * 2 + 5);

    frame.push_back(DLE);
    frame.push_back(STX);

    uint8_t checksum = 0x00;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t byte = payload[i];
        if (byte == DLE) {
            frame.push_back(DLE);
            frame.push_back(DLE);
            checksum ^= DLE;
            checksum ^= DLE;
        } else {
            frame.push_back(byte);
            checksum ^= byte;
        }
    }

    frame.push_back(DLE);
    frame.push_back(ETX);
    frame.push_back(checksum);

    const size_t written = serialPort->write(frame.data(), frame.size());
    serialPort->flush();

    // if (Serial) {
    //     Serial.printf("DoorSerial: Sent framed payload (%zu bytes payload, %zu bytes frame)\n", length, frame.size());
    // }

    return written == frame.size();
}

bool DoorSerial::sendPayload(const std::vector<uint8_t>& payload) {
    return sendPayload(payload.data(), payload.size());
}

void DoorSerial::setMessageCallback(std::function<void(const std::vector<uint8_t>&)> callback) {
    messageCallback = std::move(callback);
}

bool DoorSerial::isConnected() const {
    return serialPort != nullptr;
}

void DoorSerial::flush() {
    if (serialPort != nullptr) {
        serialPort->flush();
    }
}

void DoorSerial::clearReceiveBuffer() {
    if (serialPort == nullptr) {
        return;
    }

    while (serialPort->available()) {
        serialPort->read();
    }

    resetState();
}

size_t DoorSerial::availableForWrite() const {
    return (serialPort != nullptr) ? serialPort->availableForWrite() : 0;
}

void DoorSerial::setBaudRate(unsigned long baud) {
    baudRate = baud;

    if (Serial) {
        Serial.printf("DoorSerial: Baud rate set to %lu\n", baudRate);
    }

    if (serialPort != nullptr) {
        begin();
    }
}

void DoorSerial::setPins(uint8_t rx_pin, uint8_t tx_pin) {
    rxPin = rx_pin;
    txPin = tx_pin;

    if (Serial) {
        Serial.printf("DoorSerial: Pins set to RX:%d, TX:%d\n", rxPin, txPin);
    }

    if (serialPort != nullptr) {
        begin();
    }
}

void DoorSerial::printStatus() {
    if (!Serial) {
        return;
    }

    Serial.println("=== DoorSerial Status ===");
    Serial.printf("Initialized: %s\n", isInitialized() ? "Yes" : "No");
    Serial.printf("RX Pin: %d\n", rxPin);
    Serial.printf("TX Pin: %d\n", txPin);
    Serial.printf("Baud Rate: %lu\n", baudRate);
    Serial.printf("Queued Messages: %zu\n", messageQueue.size());

    if (serialPort != nullptr) {
        Serial.printf("Data Available: %d\n", serialPort->available());
        Serial.printf("Write Buffer Available: %zu\n", serialPort->availableForWrite());
    }

    Serial.println("========================");
}

void DoorSerial::resetState() {
    rxState = RxState::Idle;
    computedChecksum = 0x00;
    rxBuffer.clear();
}

void DoorSerial::handleIncomingByte(uint8_t byte) {
    switch (rxState) {
        case RxState::Idle:
            if (byte == DLE) {
                rxState = RxState::AwaitStx;
            }
            break;

        case RxState::AwaitStx:
            if (byte == STX) {
                rxBuffer.clear();
                computedChecksum = 0x00;
                rxState = RxState::InFrame;
            } else if (byte != DLE) {
                rxState = RxState::Idle;
            }
            break;

        case RxState::InFrame:
            if (byte == DLE) {
                rxState = RxState::AfterDle;
            } else {
                if (rxBuffer.size() >= MAX_MESSAGE_LENGTH) {
                    if (Serial) {
                        Serial.println("DoorSerial: Discarding message (payload too long)");
                    }
                    resetState();
                    break;
                }
                rxBuffer.push_back(byte);
                computedChecksum ^= byte;
            }
            break;

        case RxState::AfterDle:
            if (byte == DLE) {
                if (rxBuffer.size() >= MAX_MESSAGE_LENGTH) {
                    if (Serial) {
                        Serial.println("DoorSerial: Discarding message (payload too long)");
                    }
                    resetState();
                    break;
                }
                computedChecksum ^= DLE;
                computedChecksum ^= DLE;
                rxBuffer.push_back(DLE);
                rxState = RxState::InFrame;
            } else if (byte == ETX) {
                rxState = RxState::AwaitChecksum;
            } else {
                if (Serial) {
                    Serial.printf("DoorSerial: Unexpected escape sequence 0x%02X\n", byte);
                }
                resetState();
            }
            break;

        case RxState::AwaitChecksum:
            if (byte == computedChecksum) {
                enqueueMessage(rxBuffer);
            } else if (Serial) {
                Serial.printf("DoorSerial: Checksum mismatch (expected 0x%02X, received 0x%02X)\n", computedChecksum, byte);
            }
            resetState();
            break;
    }
}

void DoorSerial::enqueueMessage(const std::vector<uint8_t>& message) {
    if (messageQueue.size() >= MAX_QUEUE_DEPTH) {
        messageQueue.pop_front();
    }

    messageQueue.push_back(message);

    if (messageCallback) {
        messageCallback(messageQueue.back());
    }
}