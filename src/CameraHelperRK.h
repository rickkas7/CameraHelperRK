#ifndef __CAMERAHELPER_RK_H
#define __CAMERAHELPER_RK_H

// Required Libraries: uCamIII and Base64RK
#include "uCamIII.h"

#include "sha1.h"

#include <deque>

class CameraHelper {
public:
    CameraHelper(USARTSerial &serial, int baud, int resetPin = -1);
    virtual ~CameraHelper();

    /**
     * @brief You must call the setup method from setup()
     */
    void setup();

    /**
     * @brief You must call the loop method from loop()
     */
    void loop();

    /**
     * @brief Call from setup() to enable cloud configuration of settings
     * 
     * Can be done before or after the setup method above, but must be
     * done after Tracker::instance().init().
     */
    void setupCloudConfig();

    /**
     * @brief Take a picture, save, and upload it when possible
     */
    void takePicture() { takePictureFlag = true; };

    /**
     * @brief The base class does not implement power control and return false
     * 
     * Subclasses like CameraHelperTracker override this method.
     */
    virtual bool hasPowerControl() const { return false; };
    
    /**
     * @brief The base class does not implement power control this is a no-op.
     * 
     * @param on true to turn power on or false to turn it off
     * 
     * Subclasses like CameraHelperTracker override this method.
     */
    virtual void powerControl(bool on) {};


    /**
     * @brief Method to create a directory on the LittleFS file system if it does not exist
     */
    static bool createDirIfNecessary(const char *path);


protected:
    int functionHandler(String command);

    void stateScanDir();
    void stateWaitToTakePicture();
    void stateStartPicture();
    void stateSavePicture();

    void stateCameraReset();
    void stateCameraWaitPowerDown();

    void statePublishWaitForFile();
    void statePublishStartWait();
    void statePublishChunk();
    void statePublishConfirmationWait();
    void statePublishRetryWait();

    void savePictureCleanup();

    static constexpr const char *CAMERA_FUNCTION_NAME = "camera";
    static constexpr const char *CAMERA_EVENT_NAME = "camera";
    static constexpr const char *CAMERA_DIR = "/camera";
    static constexpr const char *TEMP_FILE_NAME = "temp";
    static constexpr const char *FILENUM_FILE_NAME = "filenum";

    static const size_t PATH_BUF_SIZE = 64;
    static const size_t CAMERA_CHUNK_SIZE = 512;
    static const size_t JSON_BUF_SIZE = 622;
    static const size_t PUBLISH_TEMP_BUF_SIZE = 600;
    static const size_t PUBLISH_CHUNK_SIZE = 423;    

protected:
    USARTSerial &serial;
    int baud;
    uCamIII<USARTSerial> ucam;

    // Settings
    int resolution = uCamIII_320x240;
    int contrast = uCamIII_DEFAULT;
    int brightness = uCamIII_DEFAULT;
    int exposure = uCamIII_DEFAULT;

    // Camera
    std::function<void(CameraHelper&)> cameraStateHandler = 0;
    bool takePictureFlag = false;
    long imageSize = 0;
    long imageOffset = 0;
    int lastFileNum = 0;
    int saveFd = -1;
    uint8_t cameraChunkBuf[CAMERA_CHUNK_SIZE];
    SHA1_CTX imageHash;
    unsigned long cameraStateTime = 0;
    
    // Publishing
    std::function<void(CameraHelper&)> publishStateHandler = 0;
    std::deque<int> publishQueue;
    int publishFd = -1;
    int publishFileNum = 0;
    size_t publishFileSize = 0;
    int publishChunk = 0;
    int publishNumChunks = 0;
    uint8_t publishTempBuf[PUBLISH_TEMP_BUF_SIZE];
    char publishJsonBuf[JSON_BUF_SIZE];
    unsigned long publishTime = 0;
    std::deque<int> republishQueue;
    bool publishStart = false;
    bool publishRestart = false;
    bool publishDone = false;
};

/**
 * @brief Class for CameraHelper on the Tracker One
 * 
 * The main difference is that this implements power control using CAN_PWR to control the CAN_5V line.
 * 
 * If you do not want to use smart power control (turns on camera only when necessary
 * and resets the camera using a 5 second power down) just use the base CameraHelper
 * class and manually manage CAN_PWR. You might do this if you are powering other 
 * peripherals using CAN_5V.
 */
class CameraHelperTracker : public CameraHelper {
public:
    CameraHelperTracker(USARTSerial &serial = Serial1, int baud = 115200, int resetPin = -1);
    virtual ~CameraHelperTracker();

    /**
     * @brief Returns true as this class implements power control
     */
    virtual bool hasPowerControl() const { return true; };

    /**
     * @brief Turns on or off CAN_PWR as CAN_5V powers the uCam
     * 
     * @param on true to turn on power, false to turn off power
     */
    virtual void powerControl(bool on);


};

#endif /* __CAMERAHELPER_RK_H */
