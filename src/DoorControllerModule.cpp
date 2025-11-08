#include "Wire.h"
#include <Arduino.h>
#include <cstring>
#include <DoorControllerModule.h>

namespace
{
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_INIT1), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_INIT2), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_INIT3), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_OPEN), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_CLOSING), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_CLOSED), "Door payload size mismatch");
static_assert(DOOR_PAYLOAD_SIZE == sizeof(PAYLOAD_OPENING), "Door payload size mismatch");

template <typename T, size_t N>
constexpr size_t arrayCount(const T (&)[N])
{
    return N;
}

// Define prefix message sequences here. Each entry is transmitted once (in order)
// before the final payload is sent continuously again.
constexpr const uint8_t *PREFIX_CLOSING[] = {PAYLOAD_CLOSING_PRE1, PAYLOAD_CLOSING_PRE2};
constexpr const uint8_t *PREFIX_OPENING[] = {PAYLOAD_OPENING_PRE1};

constexpr DoorCommandDefinition COMMAND_INIT1{PAYLOAD_INIT1, 0u, nullptr};
constexpr DoorCommandDefinition COMMAND_INIT2{PAYLOAD_INIT2, 0u, nullptr};
constexpr DoorCommandDefinition COMMAND_INIT3{PAYLOAD_INIT3, 0u, nullptr};
constexpr DoorCommandDefinition COMMAND_OPEN{PAYLOAD_OPEN, 0u, nullptr};
constexpr DoorCommandDefinition COMMAND_CLOSING{PAYLOAD_CLOSING, arrayCount(PREFIX_CLOSING), PREFIX_CLOSING};
constexpr DoorCommandDefinition COMMAND_CLOSED{PAYLOAD_CLOSED, 0u, nullptr};
constexpr DoorCommandDefinition COMMAND_OPENING{PAYLOAD_OPENING, arrayCount(PREFIX_OPENING), PREFIX_OPENING};

struct CommandLookupEntry
{
    const char *token;
    const DoorCommandDefinition *definition;
};

constexpr CommandLookupEntry COMMAND_LOOKUP[] = {
    {"it1", &COMMAND_INIT1},
    {"it2", &COMMAND_INIT2},
    {"it3", &COMMAND_INIT3},
    {"opg", &COMMAND_OPENING},
    {"opn", &COMMAND_OPEN},
    {"clg", &COMMAND_CLOSING},
    {"cls", &COMMAND_CLOSED},
};
} // namespace

const std::string DoorControllerModule::name()
{
    return "DoorController";
}

const std::string DoorControllerModule::version()
{
    return MAIN_Version;
}

void DoorControllerModule::setup()
{
    logDebugP("Setup ElectricDoorDrive");
    logIndentUp();

    logDebugP("Setup PIN modes");
    openknx.gpio.pinMode(MAIN_DOOR_ENABLE_PIN, OUTPUT, true, MAIN_DOOR_ENABLE_ACTIVE);
    openknx.gpio.pinMode(MAIN_DOOR_MASTER_PIN, OUTPUT, true, !MAIN_DOOR_MASTER_ACTIVE);
    openknx.gpio.pinMode(MAIN_PWR_PIN, INPUT);
    openknx.gpio.pinMode(LOCK_PIN, OUTPUT, true, !LOCK_ACTIVE);
    openknx.gpio.pinMode(SENSOR_TST_PIN, OUTPUT, true, !SENSOR_TST_ACTIVE);
    openknx.gpio.pinMode(SENSOR_INSIDE_RAD_PIN, INPUT_PULLUP);
    openknx.gpio.pinMode(SENSOR_INSIDE_AIR_PIN, INPUT_PULLUP);
    openknx.gpio.pinMode(SENSOR_OUTSIDE_RAD_PIN, INPUT_PULLUP);
    openknx.gpio.pinMode(SENSOR_OUTSIDE_AIR_PIN, INPUT_PULLUP);

    doorSerial.setMessageCallback([this](const std::vector<uint8_t>& payload) {
        this->doorMessageCallback(payload);
    });

    doorSerial.begin();

    logDebugP("Get initial sensor states");
    sensorInsideRadActiveNew = openknx.gpio.digitalRead(SENSOR_INSIDE_RAD_PIN) == SENSOR_RAD_ACTIVE;
    sensorInsideAirActiveNew = openknx.gpio.digitalRead(SENSOR_INSIDE_AIR_PIN) == SENSOR_AIR_ACTIVE;
    sensorOutsideRadActiveNew = openknx.gpio.digitalRead(SENSOR_OUTSIDE_RAD_PIN) == SENSOR_RAD_ACTIVE;
    sensorOutsideAirActiveNew = openknx.gpio.digitalRead(SENSOR_OUTSIDE_AIR_PIN) == SENSOR_AIR_ACTIVE;

    logDebugP("Attach interrupts");
    attachInterrupt(digitalPinToInterrupt(SENSOR_INSIDE_RAD_PIN), DoorControllerModule::interruptSensorInsideRadChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SENSOR_INSIDE_AIR_PIN), DoorControllerModule::interruptSensorInsideAirChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SENSOR_OUTSIDE_RAD_PIN), DoorControllerModule::interruptSensorOutsideRadChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SENSOR_OUTSIDE_AIR_PIN), DoorControllerModule::interruptSensorOutsideAirChange, CHANGE);

    enableExtInterface();

    logDebugP("Setup complete");
    logIndentDown();
}

void DoorControllerModule::processInputKo(GroupObject &ko)
{
    uint16_t lAsap = ko.asap();
    switch (lAsap)
    {
        case DOR_KoDoorMode:
            doorMode = static_cast<DoorMode>((byte)KoDOR_DoorMode.value(DPT_DecimalFactor));
            KoDOR_DoorModeStatus.valueNoSend((byte)doorMode, DPT_DecimalFactor);
            logDebugP("DoorMode changed: %d", doorMode);
            break;
        case DOR_KoSwitchInside:
            // switch trigger can only be used in manual door mode
            if (doorMode != MANUAL)
                break;

            // switch trigger can only be set, not reset externally
            switchInsideTrigger = KoDOR_SwitchInside.value(DPT_Switch) ? true : switchInsideTrigger;
            logDebugP("SwitchInside triggered");
            break;
        case DOR_KoSwitchOutside:
            // switch trigger can only be used in manual door mode
            if (doorMode != MANUAL)
                break;

            // switch trigger can only be set, not reset externally
            switchOutsideTrigger = KoDOR_SwitchOutside.value(DPT_Switch) ? true : switchOutsideTrigger;
            logDebugP("SwitchOutside triggered");
            break;
        case DOR_KoDoorLock:
            lockRequested = KoDOR_DoorLock.value(DPT_Switch);
            logDebugP("LockRequested: %d", lockRequested);
            break;
    }
}

uint16_t DoorControllerModule::flashSize()
{
    // Version + DoorMode
    return 1 + 1;
}

void DoorControllerModule::readFlash(const uint8_t *data, const uint16_t size)
{
    if (size == 0) // first call - without data
        return;

    byte version = openknx.flash.readByte();
    if (version != 1) // version unknown
    {
        logDebugP("Wrong version of flash data: version %d", version);
        return;
    }

    doorMode = static_cast<DoorMode>(openknx.flash.readByte());
    KoDOR_DoorMode.valueNoSend((byte)doorMode, DPT_DecimalFactor);
    KoDOR_DoorModeStatus.valueNoSend((byte)doorMode, DPT_DecimalFactor);
    logDebugP("DoorMode read from flash: %d", doorMode);
}

void DoorControllerModule::writeFlash()
{
    openknx.flash.writeByte(1); // Version
    openknx.flash.writeByte((byte)doorMode);
}

void DoorControllerModule::interruptSensorInsideRadChange()
{
    sensorInsideRadActiveNew = digitalRead(SENSOR_INSIDE_RAD_PIN) == SENSOR_RAD_ACTIVE;
}

void DoorControllerModule::interruptSensorInsideAirChange()
{
    sensorInsideAirActiveNew = digitalRead(SENSOR_INSIDE_AIR_PIN) == SENSOR_AIR_ACTIVE;
}

void DoorControllerModule::interruptSensorOutsideRadChange()
{
    sensorOutsideRadActiveNew = digitalRead(SENSOR_OUTSIDE_RAD_PIN) == SENSOR_RAD_ACTIVE;
}

void DoorControllerModule::interruptSensorOutsideAirChange()
{
    sensorOutsideAirActiveNew = digitalRead(SENSOR_OUTSIDE_AIR_PIN) == SENSOR_AIR_ACTIVE;
}

void DoorControllerModule::enableExtInterface()
{
    logDebugP("Enable EXT interface");

    openknx.gpio.pinMode(EXT_SWITCH_OUT_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SWITCH_INS_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_DOOR_OPN_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_DOOR_CLD_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_DOOR_MODE_AUT_PIN, OUTPUT, true, HIGH);
    openknx.gpio.pinMode(EXT_DOOR_MODE_MAN_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_DOOR_MODE_OPN_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_DOOR_MODE_CLD_PIN, OUTPUT, true, LOW);

    openknx.gpio.pinMode(EXT_SENSOR_INSIDE_AIR_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_INSIDE_RAD_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_LCK_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_TST_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_NSK_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_HSK_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_MLD_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_MAIN_PWR_PIN, OUTPUT, true, LOW);

    openknx.gpio.pinMode(EXT_SENSOR_NSK2_TST_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_NSK2_AIR_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_NSK1_TST_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_NSK1_AIR_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_OUTSIDE_TST_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_OUTSIDE_AIR_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_OUTSIDE_RAD_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_SENSOR_INSIDE_TST_PIN, OUTPUT, true, LOW);

    openknx.gpio.pinMode(EXT_KNX_PRG_SWITCH_PIN, INPUT_PULLUP);
    openknx.gpio.pinMode(EXT_KNX_PRG_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_KNX_INF_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_POWER_RUN_PIN, OUTPUT, true, HIGH);
    openknx.gpio.pinMode(EXT_LOCK_ACT_PIN, OUTPUT, true, LOW);
    openknx.gpio.pinMode(EXT_LOCK_RQT_PIN, OUTPUT, true, LOW);
}

void DoorControllerModule::loop()
{
    processDoorSerial();
    return;

    processSensorInsideRadChange();
    processSensorInsideAirChange();
    processSensorOutsideRadChange();
    processSensorOutsideAirChange();

    processTestSignal();
    checkProtection();
    checkDoorPower();
    updateDoorState();
    processDoorStateMachine();
    updateExtensionOutputs();
}

void DoorControllerModule::doorMessageCallback(const std::vector<uint8_t>& payload)
{
    // Only output the received command if it differs from the last one we saw.
    if (payload.size() == sizeof(lastDataDoorReceived))
    {
        if (doorDebugOutput ||
            memcmp(payload.data(), lastDataDoorReceived, sizeof(lastDataDoorReceived)) != 0)
        {
            memcpy(lastDataDoorReceived, payload.data(), sizeof(lastDataDoorReceived));
            logDebugP("Door RECEIVED command changed:");
            logIndentUp();
            logHexDebugP(lastDataDoorReceived, sizeof(lastDataDoorReceived));
            logIndentDown();
        }
    }
    else
    {
        // Different length -> treat as changed: store what we can and print payload
        size_t copyLen = std::min(payload.size(), sizeof(lastDataDoorReceived));
        memset(lastDataDoorReceived, 0, sizeof(lastDataDoorReceived));
        if (copyLen > 0)
            memcpy(lastDataDoorReceived, payload.data(), copyLen);

        logDebugP("Door RECEIVED command (len %u) differs from stored one:", static_cast<unsigned>(payload.size()));
        if (!payload.empty())
        {
            logIndentUp();
            logHexDebugP(payload.data(), payload.size());
            logIndentDown();
        }
    }
}

void DoorControllerModule::processDoorSerial()
{
    doorSerial.poll();

    if (lastDoorSent > 0 && delayCheckMillis(lastDoorSent, DOOR_SEND_INTERVAL))
    {
        lastDoorSent = millis();
        const uint8_t *payloadToSend = doorDataSending;

        if (activeDoorPrefixes != nullptr && activeDoorPrefixIndex < activeDoorPrefixCount)
        {
            payloadToSend = activeDoorPrefixes[activeDoorPrefixIndex];
            ++activeDoorPrefixIndex;
        }

        doorSerial.sendPayload(payloadToSend, DOOR_PAYLOAD_SIZE);

        if (doorDebugOutput ||
            memcmp(payloadToSend, lastDataDoorSent, DOOR_PAYLOAD_SIZE) != 0)
        {
            memcpy(lastDataDoorSent, payloadToSend, DOOR_PAYLOAD_SIZE);

            logDebugP("Door SEND command changed:");
            logIndentUp();
            logHexDebugP(lastDataDoorSent, DOOR_PAYLOAD_SIZE);
            logIndentDown();
        }
    }

    // uint8_t payload[DoorSerial::MaxMessageLength];

    // while (doorSerial.hasData())
    // {
    //     if (doorSerial.readBinaryData(payload, sizeof(payload)) == 0)
    //     {
    //         break;
    //     }
    //     // TODO: interpret door payload
    // }
}

void DoorControllerModule::processSensorInsideRadChange()
{
    if (sensorInsideRadActive != sensorInsideRadActiveNew)
    {
        sensorInsideRadActive = sensorInsideRadActiveNew;
        logDebugP("sensorInsideRadActive: %i", sensorInsideRadActive);
    }
}

void DoorControllerModule::processSensorInsideAirChange()
{
    if (sensorInsideAirActive != sensorInsideAirActiveNew)
    {
        sensorInsideAirActive = sensorInsideAirActiveNew;
        logDebugP("sensorInsideAirActive: %i", sensorInsideAirActive);
    }
}

void DoorControllerModule::processSensorOutsideRadChange()
{
    if (sensorOutsideRadActive != sensorOutsideRadActiveNew)
    {
        sensorOutsideRadActive = sensorOutsideRadActiveNew;
        logDebugP("sensorOutsideRadActive: %i", sensorOutsideRadActive);
    }
}

void DoorControllerModule::processSensorOutsideAirChange()
{
    if (sensorOutsideAirActive != sensorOutsideAirActiveNew)
    {
        sensorOutsideAirActive = sensorOutsideAirActiveNew;
        logDebugP("sensorOutsideAirActive: %i", sensorOutsideAirActive);
    }
}

void DoorControllerModule::processTestSignal()
{
    // int testSignal = analogRead(MAIN_TST_PIN);
    // if (testSignal > MAIN_TST_THRESHOLD + MAIN_TST_THRESHOLD_MARGIN)
    // {
    //     if (mainTstActive)
    //     {
    //         digitalWrite(SENSOR_TST_PIN, !SENSOR_TST_ACTIVE);
    //         sensorTstActive = false;
    //         mainTstActive = false;

    //         logDebugP("mainTstActive: %i", mainTstActive);
    //     }
    // }
    // else if (testSignal < MAIN_TST_THRESHOLD - MAIN_TST_THRESHOLD_MARGIN)
    // {
    //     if (!mainTstActive)
    //     {
    //         digitalWrite(SENSOR_TST_PIN, SENSOR_TST_ACTIVE);
    //         sensorTstActive = true;
    //         mainTstActive = true;

    //         logDebugP("mainTstActive: %i", mainTstActive);
    //     }
    // }
}

void DoorControllerModule::checkProtection()
{
    bool mainHskActiveNew = false;
    if (ParamDOR_SafetySensorHsk == 1)
        mainHskActiveNew = sensorInsideAirActive;
    else if (ParamDOR_SafetySensorHsk == 2)
        mainHskActiveNew = sensorOutsideAirActive;
    else if (ParamDOR_SafetySensorHsk == 3)
        mainHskActiveNew = sensorInsideAirActive || sensorOutsideAirActive;

    if (mainHskActive != mainHskActiveNew)
    {
        mainHskActive = mainHskActiveNew;
        //digitalWrite(MAIN_HSK_PIN, mainHskActive ? MAIN_HSK_NSK_ACTIVE : !MAIN_HSK_NSK_ACTIVE);

        logDebugP("mainHskActive: %i", mainHskActive);
    }

    bool mainNskActiveNew = false;
    if (ParamDOR_SafetySensorNsk == 1)
        mainNskActiveNew = sensorInsideAirActive;
    else if (ParamDOR_SafetySensorNsk == 2)
        mainNskActiveNew = sensorOutsideAirActive;
    else if (ParamDOR_SafetySensorNsk == 3)
        mainNskActiveNew = sensorInsideAirActive || sensorOutsideAirActive;

    if (mainNskActive != mainNskActiveNew)
    {
        mainNskActive = mainNskActiveNew;
        //digitalWrite(MAIN_NSK_PIN, mainNskActive ? MAIN_HSK_NSK_ACTIVE : !MAIN_HSK_NSK_ACTIVE);

        logDebugP("mainNskActive: %i", mainNskActive);
    }
}

void DoorControllerModule::checkDoorPower()
{
    int testSignal = analogRead(MAIN_PWR_PIN);
    if (testSignal <= MAIN_PWR_THRESHOLD - MAIN_PWR_THRESHOLD_MARGIN)
    {
        if (mainPwrActive)
        {
            mainPwrActive = false;
            logDebugP("mainPwrActive: %i", mainPwrActive);
        }
    }
    else if (testSignal > MAIN_PWR_THRESHOLD + MAIN_PWR_THRESHOLD_MARGIN)
    {
        if (!mainPwrActive)
        {
            mainPwrActive = true;
            logDebugP("mainPwrActive: %i", mainPwrActive);
        }
    }
}

void DoorControllerModule::updateDoorState()
{
    //###ToDo

    if (doorStatePrevious != doorState)
    {
        if (doorState != DoorState::UNDEFINED)
            KoDOR_DoorStatus.valueNoSend((byte)doorState, DPT_Switch_Control);

        switch (doorState)
        {
            case DoorState::UNDEFINED:
                logDebugP("DoorState: UNDEFINED");
                break;

            case DoorState::OPEN:
                KoDOR_DoorOpenClosed.valueNoSend(true, DPT_Window_Door);
                logDebugP("DoorState: OPEN");
                break;

            case DoorState::CLOSED:
                KoDOR_DoorOpenClosed.valueNoSend(false, DPT_Window_Door);
                logDebugP("DoorState: CLOSED");
                break;

            case DoorState::OPENING:
                logDebugP("DoorState: OPENING");
                break;

            case DoorState::CLOSING:
                logDebugP("DoorState: CLOSING");
                break;
        }

        doorStatePrevious = doorState;
        doorStateChanged = true;
        doorStateLastChanged = millis();
    }

    /*logDebugP("doorOpen: %d", doorOpen);
    logDebugP("doorClosed: %d", doorClosed);
    delay(500);*/
}

void DoorControllerModule::processDoorStateMachine()
{
    if (doorMode != DoorMode::AUTOMATIC &&
        doorMode != DoorMode::MANUAL)
        return;

    // if (mainLckStart > 0)
    // {
    //     if ((millis() - mainLckStart >= MAIN_SIGNAL_LENGTH))
    //     {
    //         digitalWrite(MAIN_LCK_PIN, !MAIN_LCK_ACTIVE);
    //         mainLckActive = false;
    //         mainLckStart = 0;

    //         logDebugP("mainLckActive: %i", mainLckActive);
    //     }
    // }

    // if (mainMdlStart > 0)
    // {
    //     if ((millis() - mainMdlStart >= MAIN_SIGNAL_LENGTH))
    //         sendMainMld(false);
    // }
    // else if (lockRequested != lockActive)
    // {
    //     if (doorStateMachine == DoorStateMachine::STATE_CLOSED ||
    //         doorStateMachine == DoorStateMachine::STATE_CLOSED_LOCKED)
    //     {
    //         lock(lockRequested);
    //     }
    //     else if (millis() - lastLockRequestMld >= LOCK_REQUEST_MLD_TIMEOUT)
    //     {
    //         sendMainMld(true);
    //         lastLockRequestMld = millis();
    //     }
    // }

    bool triggerMld = false;
    switch (doorStateMachine)
    {
        case DoorStateMachine::STATE_UNDEFINED:
            if (doorState == DoorState::OPEN)
                doorStateMachine = DoorStateMachine::STATE_OPEN_START;
            else if (doorState == DoorState::CLOSED)
                doorStateMachine = DoorStateMachine::STATE_CLOSED;
            break;

        case DoorStateMachine::STATE_OPEN_START:
            doorOpenSince = millis();
            doorStateMachine = DoorStateMachine::STATE_OPEN;
            break;

        case DoorStateMachine::STATE_OPEN:
            if (doorState != DoorState::OPEN)
            {
                doorStateMachine = DoorStateMachine::STATE_UNDEFINED;
                break;
            }

            if (doorMode == DoorMode::AUTOMATIC)
            {
                triggerMld =
                    !sensorInsideRadActive && !sensorOutsideRadActive &&
                    !sensorInsideAirActive && !sensorOutsideAirActive &&
                    (millis() - doorOpenSince >= DOOR_OPEN_MIN);
            }
            else
            {
                triggerMld =
                    !sensorInsideAirActive && !sensorOutsideAirActive &&
                    (switchInsideTrigger || switchOutsideTrigger);
            }

            if (triggerMld)
            {
                sendMainMld(true);
                doorStateMachine = DoorStateMachine::STATE_TRANSITION;
            }

            break;

        case DoorStateMachine::STATE_CLOSED:
            if (doorState != DoorState::CLOSED)
            {
                doorStateMachine = DoorStateMachine::STATE_UNDEFINED;
                break;
            }

            if (lockActive)
            {
                doorStateMachine = DoorStateMachine::STATE_CLOSED_LOCKED;
                break;
            }

            if (doorMode == DoorMode::AUTOMATIC)
                triggerMld = sensorInsideRadActive || sensorOutsideRadActive;
            else
                triggerMld = switchInsideTrigger || switchOutsideTrigger;

            if (triggerMld)
            {
                sendMainMld(true);
                doorStateMachine = DoorStateMachine::STATE_TRANSITION;
            }

            break;

        case DoorStateMachine::STATE_CLOSED_LOCKED:
            // this should not be possible, but just in case to avoid getting stuck
            if (doorState != DoorState::CLOSED)
            {
                doorStateMachine = DoorStateMachine::STATE_UNDEFINED;
                break;
            }

            if (!lockActive)
                doorStateMachine = DoorStateMachine::STATE_CLOSED;
            break;

        case DoorStateMachine::STATE_TRANSITION:
            if (doorStateChanged ||
                (millis() - doorStateLastChanged >= DOOR_STATE_CHANGED_TIMEOUT))
            {
                if (doorState == DoorState::OPEN)
                    doorStateMachine = DoorStateMachine::STATE_OPEN_START;
                else if (doorState == DoorState::CLOSED)
                    doorStateMachine = DoorStateMachine::STATE_CLOSED;
            }
            break;
    }

    if (doorStateMachinePrevious != doorStateMachine)
    {
        switch (doorStateMachine)
        {
            case DoorStateMachine::STATE_UNDEFINED:
                logDebugP("DoorStateMachine: STATE_UNDEFINED");
                break;

            case DoorStateMachine::STATE_OPEN_START:
                logDebugP("DoorStateMachine: STATE_OPEN_START");
                break;

            case DoorStateMachine::STATE_OPEN:
                logDebugP("DoorStateMachine: STATE_OPEN");
                break;

            case DoorStateMachine::STATE_CLOSED:
                logDebugP("DoorStateMachine: STATE_CLOSED");
                break;

            case DoorStateMachine::STATE_CLOSED_LOCKED:
                logDebugP("DoorStateMachine: STATE_CLOSED_LOCKED");
                break;

            case DoorStateMachine::STATE_TRANSITION:
                logDebugP("DoorStateMachine: STATE_TRANSITION");
                break;
        }

        doorStateMachinePrevious = doorStateMachine;
    }
}

void DoorControllerModule::sendMainMld(bool active)
{
    doorStateChanged = false;
    doorStateLastChanged = millis();

    // digitalWrite(MAIN_MLD_PIN, active ? MAIN_MLD_ACTIVE : !MAIN_MLD_ACTIVE);
    // mainMldActive = active;
    // mainMdlStart = active ? millis() : 0;
}

void DoorControllerModule::lock(bool active)
{
    if (lockActive == active)
        return;

    digitalWrite(LOCK_PIN, active ? LOCK_ACTIVE : !LOCK_ACTIVE);
    KoDOR_DoorLockStatus.valueNoSend(active, DPT_Switch);
    lockActive = active;

    logDebugP("lockActive: %i", lockActive);
}

void DoorControllerModule::setDoorCommand(const DoorCommandDefinition &definition)
{
    activeDoorPrefixes = (definition.prefixCount > 0 && definition.prefixPayloads != nullptr) ? definition.prefixPayloads : nullptr;
    activeDoorPrefixCount = (definition.prefixCount > 0 && definition.prefixPayloads != nullptr) ? definition.prefixCount : 0;
    activeDoorPrefixIndex = 0;

    if (definition.finalPayload != nullptr)
        memcpy(doorDataSending, definition.finalPayload, DOOR_PAYLOAD_SIZE);
    else
        memset(doorDataSending, 0, DOOR_PAYLOAD_SIZE);

    memset(lastDataDoorSent, 0, sizeof(lastDataDoorSent));
    lastDoorSent = 1; // trigger immediate send on next loop iteration
}

void DoorControllerModule::updateExtensionOutputs()
{
    if (lastExtMainPwr != mainPwrActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_PWR_PIN, mainPwrActive ? HIGH : LOW);
        lastExtMainPwr = mainPwrActive;
    }

    if (lastExtMainMld != mainMldActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_MLD_PIN, mainMldActive ? HIGH : LOW);
        lastExtMainMld = mainMldActive;
    }

    if (lastExtMainHsk != mainHskActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_HSK_PIN, mainHskActive ? HIGH : LOW);
        lastExtMainHsk = mainHskActive;
    }

    if (lastExtMainNsk != mainNskActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_NSK_PIN, mainNskActive ? HIGH : LOW);
        lastExtMainNsk = mainNskActive;
    }

    if (lastExtMainTst != mainTstActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_TST_PIN, mainTstActive ? HIGH : LOW);
        lastExtMainTst = mainTstActive;
    }

    if (lastExtMainNsk != mainNskActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_NSK_PIN, mainNskActive ? HIGH : LOW);
        lastExtMainNsk = mainNskActive;
    }

    if (lastExtMainTst != mainTstActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_TST_PIN, mainTstActive ? HIGH : LOW);
        lastExtMainTst = mainTstActive;
    }

    if (lastExtMainLck != mainLckActive)
    {
        openknx.gpio.digitalWrite(EXT_MAIN_LCK_PIN, mainLckActive ? HIGH : LOW);
        lastExtMainLck = mainLckActive;
    }

    bool doorClosed = doorState == DoorState::CLOSED;
    if (lastExtDoorCld != doorClosed)
    {
        openknx.gpio.digitalWrite(EXT_DOOR_CLD_PIN, doorClosed ? HIGH : LOW);
        lastExtDoorCld = doorClosed;
    }

    bool doorOpen = doorState == DoorState::OPEN;
    if (lastExtDoorOpn != doorOpen)
    {
        openknx.gpio.digitalWrite(EXT_DOOR_OPN_PIN, doorOpen ? HIGH : LOW);
        lastExtDoorOpn = doorOpen;
    }

    if (lastExtSensorInsideRad != sensorInsideRadActive)
    {
        openknx.gpio.digitalWrite(EXT_SENSOR_INSIDE_RAD_PIN, sensorInsideRadActive ? HIGH : LOW);
        lastExtSensorInsideRad = sensorInsideRadActive;
    }

    if (lastExtSensorInsideAir != sensorInsideAirActive)
    {
        openknx.gpio.digitalWrite(EXT_SENSOR_INSIDE_AIR_PIN, sensorInsideAirActive ? HIGH : LOW);
        lastExtSensorInsideAir = sensorInsideAirActive;
    }

    if (lastExtSensorTst != sensorTstActive)
    {
        openknx.gpio.digitalWrite(EXT_SENSOR_OUTSIDE_TST_PIN, sensorTstActive ? HIGH : LOW);
        openknx.gpio.digitalWrite(EXT_SENSOR_INSIDE_TST_PIN, sensorTstActive ? HIGH : LOW);
        lastExtSensorTst = sensorTstActive;
    }

    if (lastExtSensorOutsideRad != sensorOutsideRadActive)
    {
        openknx.gpio.digitalWrite(EXT_SENSOR_OUTSIDE_RAD_PIN, sensorOutsideRadActive ? HIGH : LOW);
        lastExtSensorOutsideRad = sensorOutsideRadActive;
    }

    if (lastExtSensorOutsideAir != sensorOutsideAirActive)
    {
        openknx.gpio.digitalWrite(EXT_SENSOR_OUTSIDE_AIR_PIN, sensorOutsideAirActive ? HIGH : LOW);
        lastExtSensorOutsideAir = sensorOutsideAirActive;
    }
    
    if (lastExtDoorMode != doorMode)
    {
        openknx.gpio.digitalWrite(EXT_DOOR_MODE_AUT_PIN, doorMode == DoorMode::AUTOMATIC ? HIGH : LOW);
        openknx.gpio.digitalWrite(EXT_DOOR_MODE_MAN_PIN, doorMode == DoorMode::MANUAL ? HIGH : LOW);
        openknx.gpio.digitalWrite(EXT_DOOR_MODE_OPN_PIN, doorMode == DoorMode::ALWAYS_OPEN ? HIGH : LOW);
        openknx.gpio.digitalWrite(EXT_DOOR_MODE_CLD_PIN, doorMode == DoorMode::ALWAYS_CLOSED ? HIGH : LOW);
        lastExtDoorMode = doorMode;
    }

    if (lastExtLockRqt != lockRequested)
    {
        openknx.gpio.digitalWrite(EXT_LOCK_RQT_PIN, lockRequested ? HIGH : LOW);
        lastExtLockRqt = lockRequested;
    }

    if (lastExtLockAct != lockActive)
    {
        openknx.gpio.digitalWrite(EXT_LOCK_ACT_PIN, lockActive ? HIGH : LOW);
        lastExtLockAct = lockActive;
    }

    if (delayCheckMillis(extProgSwitchLastTrigger, EXT2_KNX_PRG_SWITCH_DEBOUNCE) &&
        openknx.gpio.digitalRead(EXT_KNX_PRG_SWITCH_PIN) == EXT_KNX_PRG_SWITCH_ACTIVE)
    {
        knx.toggleProgMode();
        extProgSwitchLastTrigger = millis();
    }

    bool progMode = knx.progMode();
    if (lastExtKnxPrg != progMode)
    {
        openknx.gpio.digitalWrite(EXT_KNX_PRG_PIN, progMode ? HIGH : LOW);
        lastExtKnxPrg = progMode;
    }
}

void DoorControllerModule::showHelp()
{
    logInfo("dc send it1", "Send INIT1 command to door.");
    logInfo("dc send it2", "Send INIT2 command to door.");
    logInfo("dc send it3", "Send INIT3 command to door.");
    logInfo("dc send opg", "Send OPENING command to door.");
    logInfo("dc send opn", "Send OPEN command to door.");
    logInfo("dc send clg", "Send CLOSING command to door.");
    logInfo("dc send cls", "Send CLOSED command to door.");
    logInfo("dc status", "Print door serial status.");
    logInfo("dc debug [0/1]", "Enable or disable extensive debug output.");
}

bool DoorControllerModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    if (cmd.substr(0, 2) != "dc")
        return false;

    if (cmd.length() > 8 && cmd.compare(0, 8, "dc send ") == 0)
    {
        std::string token = cmd.substr(8);

        const DoorCommandDefinition *definition = nullptr;
        for (const auto &entry : COMMAND_LOOKUP)
        {
            if (token == entry.token)
            {
                definition = entry.definition;
                break;
            }
        }

        if (definition == nullptr)
        {
            logInfoP("dc send command with bad args");
            return false;
        }

        setDoorCommand(*definition);
        return true;
    }

    if (cmd.length() == 9 && cmd.substr(0, 9) == "dc status")
    {
        doorSerial.printStatus();
        return true;
    }

    if (cmd.length() == 10 && cmd.substr(0, 9) == "dc debug ")
    {
        if (cmd.substr(9, 1) == "0")
            doorDebugOutput = false;
        else if (cmd.substr(9, 1) == "1")
            doorDebugOutput = true;

        return true;
    }

    logInfoP("dc (DoorController) command with bad args");
    if (diagnoseKo)
        openknx.console.writeDiagenoseKo("dc: bad args");
    
    return true;
}

DoorControllerModule openknxDoorControllerModule;