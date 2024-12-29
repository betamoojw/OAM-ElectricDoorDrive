#include "DoorControllerModule.h"
#include "FileTransferModule.h"
#include "Logic.h"
#include "OpenKNX.h"
#include "VirtualButtonModule.h"
#include <Arduino.h>

void setup()
{
    const uint8_t firmwareRevision = 0;
    openknx.init(firmwareRevision);
    openknx.addModule(1, openknxLogic);
    openknx.addModule(2, openknxDoorControllerModule);
    openknx.addModule(3, openknxVirtualButtonModule);
    openknx.addModule(9, openknxFileTransferModule);
    openknx.setup();

    // call direct for testing without KNX connected
    //openknxDoorControllerModule.setup();
}

void loop()
{
    openknx.loop();

    // call direct for testing without KNX connected
    //openknxDoorControllerModule.loop();
}