#include "Particle.h"

#include "CameraHelperRK.h"


SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

int takePictureHandler(String cmd);

CameraHelper cameraHelper(Serial1, 115200);


void setup()
{
    Particle.function("takePicture", takePictureHandler);

    // Start the camera interface
    // The CameraHelperTracker turns on the CAN_PWR to power the camera.
    cameraHelper.setup();

    Particle.connect();
}

void loop()
{
    cameraHelper.loop();
}

int takePictureHandler(String cmd) 
{
    // Take a picture
    cameraHelper.takePicture();
    return 0;
}





