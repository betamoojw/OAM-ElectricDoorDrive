#include "DoorSerial.h"

#include <algorithm>
#include <utility>

DoorSerial::DoorSerial(uint8_t rx_pin, uint8_t tx_pin, unsigned long baud)
    : rxPin(rx_pin),
      txPin(tx_pin),
      baudRate(baud),
      rxState(RxState::Idle),
      computedChecksum(0) {}

DoorSerial::~DoorSerial() {
    end();
}

bool DoorSerial::begin() {
    MAIN_DOOR_SERIAL.setRX(rxPin);
    MAIN_DOOR_SERIAL.setTX(txPin);
    MAIN_DOOR_SERIAL.begin(baudRate, MAIN_DOOR_SERIAL_CONFIG);
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
    MAIN_DOOR_SERIAL.end();
    messageQueue.clear();
    resetState();
}

void DoorSerial::poll() {
    while (MAIN_DOOR_SERIAL.available()) {
        const uint8_t byte = MAIN_DOOR_SERIAL.read();
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
    if (payload == nullptr) {
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

    const size_t written = MAIN_DOOR_SERIAL.write(frame.data(), frame.size());
    MAIN_DOOR_SERIAL.flush();

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

void DoorSerial::flush() {
    MAIN_DOOR_SERIAL.flush();
}

void DoorSerial::clearReceiveBuffer() {
    while (MAIN_DOOR_SERIAL.available()) {
        MAIN_DOOR_SERIAL.read();
    }

    resetState();
}

void DoorSerial::setBaudRate(unsigned long baud) {
    baudRate = baud;

    if (Serial) {
        Serial.printf("DoorSerial: Baud rate set to %lu\n", baudRate);
    }

    begin();
}

void DoorSerial::setPins(uint8_t rx_pin, uint8_t tx_pin) {
    rxPin = rx_pin;
    txPin = tx_pin;

    if (Serial) {
        Serial.printf("DoorSerial: Pins set to RX:%d, TX:%d\n", rxPin, txPin);
    }

    begin();
}

void DoorSerial::printStatus() {
    if (!Serial) {
        return;
    }

    Serial.println("=== DoorSerial Status ===");
    Serial.printf("RX Pin: %d\n", rxPin);
    Serial.printf("TX Pin: %d\n", txPin);
    Serial.printf("Baud Rate: %lu\n", baudRate);
    Serial.printf("Queued Messages: %zu\n", messageQueue.size());

    Serial.printf("Data Available: %d\n", MAIN_DOOR_SERIAL.available());
    Serial.printf("Write Buffer Available: %zu\n", MAIN_DOOR_SERIAL.availableForWrite());

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