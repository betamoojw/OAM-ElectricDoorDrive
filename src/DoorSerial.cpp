#include "DoorSerial.h"

#include <algorithm>
#include <utility>
#include <cstdio>

DoorSerial::DoorSerial()
    : rxState(RxState::Idle),
      computedChecksum(0) {}

DoorSerial::~DoorSerial() {
    end();
}

std::string DoorSerial::logPrefix()
{
    return "DoorSerial";
}

void DoorSerial::begin() {
    MAIN_DOOR_SERIAL.setFIFOSize(1024);
    MAIN_DOOR_SERIAL.setRX(MAIN_DOOR_RX_PIN);
    MAIN_DOOR_SERIAL.setTX(MAIN_DOOR_TX_PIN);
    MAIN_DOOR_SERIAL.begin(MAIN_DOOR_SERIAL_BAUD, MAIN_DOOR_SERIAL_CONFIG);
    delay(10);

    resetState();
    clearReceiveBuffer();

    logDebugP("UART initialized (RX Pin: %d, TX Pin: %d, Baud: %lu)", MAIN_DOOR_RX_PIN, MAIN_DOOR_TX_PIN, MAIN_DOOR_SERIAL_BAUD);
}

void DoorSerial::end() {
    MAIN_DOOR_SERIAL.end();
    messageQueue.clear();
    resetState();
}

void DoorSerial::poll() {
    // std::vector<uint8_t> received;
    while (MAIN_DOOR_SERIAL.available()) {
        const uint8_t byte = MAIN_DOOR_SERIAL.read();
        // received.push_back(byte);
        handleIncomingByte(byte);
    }

    // if (!received.empty()) {
    //     // Format bytes as hex string for debug output
    //     std::string hexStr;
    //     hexStr.reserve(received.size() * 3);
    //     char buf[4];
    //     for (size_t i = 0; i < received.size(); ++i) {
    //         std::snprintf(buf, sizeof(buf), "%02X", received[i]);
    //         hexStr += buf;
    //         if (i + 1 < received.size()) {
    //             hexStr += ' ';
    //         }
    //     }

    //     logDebugP("DoorSerial: Received raw bytes (%zu): %s", received.size(), hexStr.c_str());
    // }
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
        logDebugP("DoorSerial: Message too large for buffer (%zu > %zu)", next.size(), maxLength);
        return 0;
    }

    std::copy(next.begin(), next.end(), buffer);
    const size_t length = next.size();
    messageQueue.pop_front();

    return length;
}

bool DoorSerial::sendPayload(const uint8_t* payload, size_t length) {
    if (payload == nullptr) {
        logDebugP("DoorSerial: Cannot send payload (serial not initialized or payload null)");
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

    // logDebugP("DoorSerial: Sent framed payload (%zu bytes payload, %zu bytes frame)\n", length, frame.size());

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

void DoorSerial::printStatus() {
    logDebugP("DoorSerial Status:");
    logIndentUp();
    logDebugP("RX Pin: %d", MAIN_DOOR_RX_PIN);
    logDebugP("TX Pin: %d", MAIN_DOOR_TX_PIN);
    logDebugP("Baud Rate: %lu", MAIN_DOOR_SERIAL_BAUD);
    logDebugP("Queued Messages: %zu", messageQueue.size());

    logDebugP("Data Available: %d", MAIN_DOOR_SERIAL.available());
    logDebugP("Write Buffer Available: %zu", MAIN_DOOR_SERIAL.availableForWrite());
    logIndentDown();
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
                    logDebugP("DoorSerial: Discarding message (payload too long)");
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
                    logDebugP("DoorSerial: Discarding message (payload too long)");
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
                logDebugP("DoorSerial: Unexpected escape sequence 0x%02X", byte);
                resetState();
            }
            break;

        case RxState::AwaitChecksum:
            if (byte == computedChecksum) {
                enqueueMessage(rxBuffer);
            } else {
                logDebugP("DoorSerial: Checksum mismatch (expected 0x%02X, received 0x%02X)", computedChecksum, byte);
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