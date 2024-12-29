#include "OpenKNX.h"
#include "enum-helper.h"
#include <Adafruit_MCP23X17.h>

#define MAIN_24V_PIN 26
#define MAIN_TST_PIN 27
#define MAIN_TST_THRESHOLD 100
#define MAIN_TST_THRESHOLD_MARGIN 50
#define MAIN_MLD_PIN 18
#define MAIN_MLD_ACTIVE HIGH
#define MAIN_MLD_INACTIVE LOW
#define MAIN_NSK_PIN 19
#define MAIN_HSK_PIN 17
#define MAIN_HSK_NSK_ACTIVE LOW
#define MAIN_HSK_NSK_INACTIVE HIGH
#define MAIN_LCK_PIN 16
#define MAIN_LCK_ACTIVE HIGH
#define MAIN_LCK_INACTIVE LOW
#define MAIN_SIGNAL_LENGTH 100

#define DOOR_SENSOR_PWR_PIN 23
#define DOOR_OPEN_PIN 29
#define DOOR_CLOSED_PIN 28
#define DOOR_SENSOR_THRESHOLD 500 // threshold with 220 Ohm resistor
#define DOOR_SENSOR_THRESHOLD_MARGIN 10
#define DOOR_OPEN_MIN 3000
#define DOOR_STATE_CHANGED_TIMEOUT 3000

#define LOCK_PIN 25
#define LOCK_ACTIVE HIGH
#define LOCK_INACTIVE LOW
#define LOCK_REQUEST_MLD_TIMEOUT 3000

#define SENSOR_TST_PIN 24
#define SENSOR_TST_ACTIVE HIGH
#define SENSOR_TST_INACTIVE LOW
#define SENSOR_INSIDE_RAD_PIN 5
#define SENSOR_INSIDE_AIR_PIN 6
#define SENSOR_OUTSIDE_RAD_PIN 7
#define SENSOR_OUTSIDE_AIR_PIN 8
#define SENSOR_RAD_ACTIVE LOW
#define SENSOR_AIR_ACTIVE HIGH

#define EXT_I2C_BUS Wire
#define EXT_I2C_SDA 20
#define EXT_I2C_SCL 21
#define EXT1_MCP23017_ADR 0x20
#define EXT2_MCP23017_ADR 0x21
#define EXT2_KNX_PRG_SWITCH_PIN 5
#define EXT2_KNX_PRG_SWITCH_DEBOUNCE 250
#define EXT2_KNX_PRG_SWITCH_ACTIVE LOW

enum class EXT1_A : byte
{
    NONE = 0,
    LOCK_ACT = 1,
    LOCK_RQT = 2,
    SWITCH_OUT = 4,
    SWITCH_INS = 8,
    DOOR_MODE_AUT = 16,
    DOOR_MODE_MAN = 32,
    DOOR_MODE_OPN = 64,
    DOOR_MODE_CLD = 128
};

enum class EXT1_B : byte
{
    NONE = 0,
    SENSOR_INSIDE_RAD = 1,
    DOOR_OPN = 2,
    DOOR_CLD = 4,
    MAIN_LCK = 8,
    MAIN_TST = 16,
    MAIN_NSK = 32,
    MAIN_HSK = 64,
    MAIN_MLD = 128
};

enum class EXT2_A : byte
{
    NONE = 0,
    SENSOR_OUTSIDE_TST = 1,
    SENSOR_OUTSIDE_AIR = 2,
    SENSOR_OUTSIDE_RAD = 4,
    SENSOR_INSIDE_TST = 8,
    SENSOR_INSIDE_AIR = 16,
    PROG_SWITCH = 32,
    KNX_PRG = 64,
    KNX_INF = 128
};

enum class EXT2_B : byte
{
    NONE = 0,
    POWER_RUN = 1
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

    Adafruit_MCP23X17 extMcp1;
    Adafruit_MCP23X17 extMcp2;

    EXT1_A ext1A = EXT1_A::NONE;
    EXT1_B ext1B = EXT1_B::NONE;
    EXT2_A ext2A = EXT2_A::NONE;
    EXT2_B ext2B = EXT2_B::NONE;

    unsigned long extProgSwitchLastTrigger = 0;

    // initialize with max. value to enforce first update send
    EXT1_A ext1ALastSent = (EXT1_A)255u;
    EXT1_B ext1BLastSent = (EXT1_B)255u;
    EXT2_A ext2ALastSent = (EXT2_A)255u;
    EXT2_B ext2BLastSent = (EXT2_B)255u;

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
    void processSensorInsideRadChange();
    void processSensorInsideAirChange();
    void processSensorOutsideRadChange();
    void processSensorOutsideAirChange();
    void processTestSignal();
    void checkProtection();
    void updateDoorState();
    void processDoorStateMachine();
    void processManualMachine();
    void updateExtensionOutputs();
    void sendMainMld(bool active);
    void lock(bool active);

    static void interruptSensorInsideRadChange();
    static void interruptSensorInsideAirChange();
    static void interruptSensorOutsideRadChange();
    static void interruptSensorOutsideAirChange();
};

extern DoorControllerModule openknxDoorControllerModule;