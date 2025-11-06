#include <cstddef>
#include <cstdint>
#include "OpenKNX.h"
#include "hardware.h"
#include "enum-helper.h"
#include "DoorSerial.h"

#define DOOR_SEND_INTERVAL 60

constexpr uint8_t PAYLOAD_INIT1[]         = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x04, 0x06};
constexpr uint8_t PAYLOAD_INIT2[]         = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x04, 0x05};
constexpr uint8_t PAYLOAD_INIT3[]         = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x04, 0x04};
constexpr uint8_t PAYLOAD_OPEN[]          = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x10, 0x04};
constexpr uint8_t PAYLOAD_CLOSING[]       = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x00, 0x01};
constexpr uint8_t PAYLOAD_CLOSING_PRE1[]  = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x12, 0x04}; // radar?
constexpr uint8_t PAYLOAD_CLOSING_PRE2[]  = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x14, 0x04}; // infrared?
constexpr uint8_t PAYLOAD_CLOSED[]        = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x00, 0x03};
constexpr uint8_t PAYLOAD_OPENING[]       = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x10, 0x00};
constexpr uint8_t PAYLOAD_OPENING_PRE1[]  = {0x00, 0x00, 0x00, 0x52, 0x0B, 0x00, 0x12, 0x03};

constexpr size_t DOOR_PAYLOAD_SIZE = sizeof(PAYLOAD_INIT1);

struct DoorCommandDefinition
{
  const uint8_t *finalPayload;
  size_t prefixCount;
  const uint8_t *const *prefixPayloads;
};

class DoorControllerModule : public OpenKNX::Module
{
  public:
    void loop() override;
    void setup() override;
    // void processAfterStartupDelay() override;
    void processInputKo(GroupObject &ko) override;

    const std::string name() override;
    const std::string version() override;
    uint16_t flashSize() override;
    void readFlash(const uint8_t *data, const uint16_t size) override;
    void writeFlash() override;

    void showHelp() override;
    bool processCommand(const std::string cmd, bool diagnoseKo) override;

  private:
    enum DoorState
    {
        CLOSED,
        CLOSING,
        OPEN,
        OPENING,
        UNDEFINED
    };

    enum DoorMode
    {
        ALWAYS_CLOSED,
        ALWAYS_OPEN,
        MANUAL,
        AUTOMATIC
    };

    enum DoorStateMachine
    {
        STATE_UNDEFINED,
        STATE_TRANSITION,
        STATE_OPEN_START,
        STATE_OPEN,
        STATE_CLOSED,
        STATE_CLOSED_LOCKED
    };

    DoorState doorState = DoorState::UNDEFINED;
    DoorState doorStatePrevious = DoorState::UNDEFINED;
    DoorMode doorMode = DoorMode::AUTOMATIC;
    DoorStateMachine doorStateMachine = DoorStateMachine::STATE_UNDEFINED;
    DoorStateMachine doorStateMachinePrevious = DoorStateMachine::STATE_UNDEFINED;
    bool mainPwrActive = false;
    bool mainMldActive = false;
    bool mainHskActive = false;
    bool mainNskActive = false;
    bool mainLckActive = false;
    bool switchInsideTrigger = false;
    bool switchOutsideTrigger = false;
    bool lockRequested = false;
    bool lockActive = false;
    unsigned long mainMdlStart = 0;
    bool doorStateChanged = false;
    unsigned long doorStateLastChanged = 0;
    unsigned long doorOpenSince = 0;
    unsigned long mainLckStart = 0;
    unsigned long lastLockRequestMld = 0;

    uint32_t lastDoorSent = 0;
  uint8_t doorDataSending[DOOR_PAYLOAD_SIZE] = {};
  uint8_t lastDataDoorSent[DOOR_PAYLOAD_SIZE] = {};
  uint8_t lastDataDoorReceived[DOOR_PAYLOAD_SIZE] = {};
    bool doorDebugOutput = false;

  const uint8_t *const *activeDoorPrefixes = nullptr;
  size_t activeDoorPrefixCount = 0;
  size_t activeDoorPrefixIndex = 0;

    DoorSerial doorSerial = DoorSerial();

    unsigned long extProgSwitchLastTrigger = 0;

    bool lastExtSwitchOut = false;
    bool lastExtSwitchIns = false;
    bool lastExtDoorOpn = false;
    bool lastExtDoorCld = false;
    DoorMode lastExtDoorMode = DoorMode::AUTOMATIC;

    bool lastExtSensorInsideAir = false;
    bool lastExtSensorInsideRad = false;
    bool lastExtMainLck = false;
    bool lastExtMainTst = false;
    bool lastExtMainNsk = false;
    bool lastExtMainHsk = false;
    bool lastExtMainMld = false;
    bool lastExtMainPwr = false;

    bool lastExtSensorNsk2Tst = false;
    bool lastExtSensorNsk2Air = false;
    bool lastExtSensorNsk1Tst = false;
    bool lastExtSensorNsk1Air = false;
    bool lastExtSensorOutsideAir = false;
    bool lastExtSensorOutsideRad = false;
    bool lastExtSensorTst = false;

    bool lastExtKnxPrgSwitch = false;
    bool lastExtKnxPrg = false;
    bool lastExtKnxInf = false;
    bool lastExtPowerRun = false;
    bool lastExtLockAct = false;
    bool lastExtLockRqt = false;

    bool mainTstActive = false;
    bool sensorTstActive = false;
    bool sensorInsideRadActive = false;
    bool sensorInsideAirActive = false;
    bool sensorOutsideRadActive = false;
    bool sensorOutsideAirActive = false;
    inline volatile static bool sensorInsideRadActiveNew = false;
    inline volatile static bool sensorInsideAirActiveNew = false;
    inline volatile static bool sensorOutsideRadActiveNew = false;
    inline volatile static bool sensorOutsideAirActiveNew = false;

    void enableExtInterface();
    void processDoorSerial();
    void processSensorInsideRadChange();
    void processSensorInsideAirChange();
    void processSensorOutsideRadChange();
    void processSensorOutsideAirChange();
    void processTestSignal();
    void checkProtection();
    void checkDoorPower();
    void updateDoorState();
    void processDoorStateMachine();
    void processManualMachine();
    void updateExtensionOutputs();
    void sendMainMld(bool active);
    void lock(bool active);

  void setDoorCommand(const DoorCommandDefinition &definition);

    static void interruptSensorInsideRadChange();
    static void interruptSensorInsideAirChange();
    static void interruptSensorOutsideRadChange();
    static void interruptSensorOutsideAirChange();
};

extern DoorControllerModule openknxDoorControllerModule;