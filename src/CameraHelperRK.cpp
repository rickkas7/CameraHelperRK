#include "CameraHelperRK.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "tracker.h"
#include "tracker_config.h"

// Required Libraries: uCamIII and Base64RK
#include "Base64RK.h"


static Logger _log("app.cam");


CameraHelper::CameraHelper(USARTSerial &serial, int baud, int resetPin) : 
    serial(serial), baud(baud), ucam(serial, resetPin, 500) {

}

CameraHelper::~CameraHelper() {

}


void CameraHelper::setup() {
    Particle.function(CAMERA_FUNCTION_NAME, &CameraHelper::functionHandler, this);

    cameraStateHandler = &CameraHelper::stateScanDir;
    publishStateHandler = &CameraHelper::statePublishWaitForFile;
}

void CameraHelper::loop() {
    if (cameraStateHandler) {
        cameraStateHandler(*this);
    }
    if (publishStateHandler) {
        publishStateHandler(*this);
    }
}

void CameraHelper::setupCloudConfig() {
    // Set up configuration settings
    static ConfigObject ucamDesc("ucam", {
        ConfigInt("resolution", &resolution, 1, 9), // default: 5 (uCamIII_320x240)
        ConfigInt("contrast", &contrast, 0, 4),     // default: 2
        ConfigInt("brightness", &brightness, 0, 4), // default: 2
        ConfigInt("exposure", &exposure, 0, 4),     // default: 2
    });
    Tracker::instance().configService.registerModule(ucamDesc);
}


void CameraHelper::stateScanDir() {
    bool bResult = createDirIfNecessary(CAMERA_DIR);
    if (!bResult) {
        _log.error("failed to create camera directory");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    char path[PATH_BUF_SIZE];

    snprintf(path, sizeof(path), "%s/%s", CAMERA_DIR, FILENUM_FILE_NAME);

    int fd = open(path, O_RDONLY);
    if (fd >= 0) {
        int count = read(fd, (void *)&lastFileNum, sizeof(lastFileNum));
        if (count != sizeof(lastFileNum)) {
            lastFileNum = 0;
        }
        close(fd);
    }

    DIR *dir = opendir(CAMERA_DIR);
    if (dir) {
        while(true) {
            struct dirent* ent = readdir(dir); 
            if (!ent) {
                break;
            }
            
            if (ent->d_type != DT_REG) {
                // Not a plain file
                continue;
            }
            
            int fileNum;
            if (sscanf(ent->d_name, "%06d.jpg", &fileNum) == 1) {
                // Check and make sure there is also a sha1 file. 
                snprintf(path, sizeof(path), "%s/%06d.sha1", CAMERA_DIR, fileNum);
                struct stat sb;
                int res = stat(path, &sb);
                if (res < 0) {
                    // Most likely a partially written file because the device was reset
                    // before the file was complete and the hash file written. Delete file.
                    snprintf(path, sizeof(path), "%s/%s", CAMERA_DIR, ent->d_name);
                    unlink(path);
                    _log.info("deleted partial file %s", path);
                }
                else {
                    if (fileNum > lastFileNum) {
                        lastFileNum = fileNum;
                    }
                    _log.trace("adding to publish queue %d", fileNum);
                    publishQueue.push_back(fileNum); // TODO: Enable this again
                }
            }
            else
            if (sscanf(ent->d_name, "%06d.sha1", &fileNum) == 1) {
                // Hash file - ignore as it's checked with the jpg file
                // (this has the potential for leaving a .sha1 file without a .jpg file
                // but that's unlikely as they're deleted at the same time)
            }
            else {
                if (strcmp(ent->d_name, FILENUM_FILE_NAME) != 0) {
                    snprintf(path, sizeof(path), "%s/%s", CAMERA_DIR, ent->d_name);
                    unlink(path);
                    _log.info("deleted unexpected file %s", path);
                }
            }
        }
        closedir(dir);
    }

    _log.info("lastFileNum=%d", lastFileNum);

    cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
}

void CameraHelper::stateWaitToTakePicture() {
    if (takePictureFlag) {
        takePictureFlag = false;

        cameraStateHandler = &CameraHelper::stateStartPicture;
    }
}


void CameraHelper::stateStartPicture() {
    _log.trace("camera init starting");

    // Turn on camera (ignored if no power control available)
    powerControl(true);

    long retVal = ucam.init(baud);
    if (retVal <= 0) {
        _log.error("camera init failed");
        cameraStateHandler = &CameraHelper::stateCameraReset;
        return;
    }

    _log.trace("camera init succeeded");
    
    retVal = ucam.setImageFormat(uCamIII_COMP_JPEG, (uCamIII_RES) resolution);
    if (retVal == 0) {
        _log.error("camera setImageFormat failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    retVal = ucam.setCBE((uCamIII_CBE) contrast, (uCamIII_CBE) brightness, (uCamIII_CBE) exposure);
    if (retVal == 0) {
        _log.error("camera setCBE failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    retVal = ucam.takeSnapshot(uCamIII_SNAP_JPEG);
    if (retVal == 0) {
        _log.error("camera takeSnapshot failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    retVal = ucam.setPackageSize(CAMERA_CHUNK_SIZE);
    if (retVal == 0) {
        _log.error("camera setPackageSize failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }
    _log.trace("init succeeded");


    imageSize = ucam.getPicture(uCamIII_TYPE_JPEG);
    if (imageSize <= 0) {
        _log.error("camera getPicture failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    _log.info("getPicture returned %ld", imageSize);
    imageOffset = 0;

    char path[PATH_BUF_SIZE];
    snprintf(path, sizeof(path), "%s/%06d.jpg", CAMERA_DIR, ++lastFileNum);

    saveFd = open(path, O_RDWR | O_CREAT | O_TRUNC);
    if (saveFd < 0) {
        _log.error("camera file open failed");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        return;
    }

    SHA1Init(&imageHash);    

    cameraStateHandler = &CameraHelper::stateSavePicture;
}

void CameraHelper::stateSavePicture() {
    long count = imageSize - imageOffset;
    if (count > (long) CAMERA_CHUNK_SIZE) {
        count = (long) CAMERA_CHUNK_SIZE;
    }
    _log.trace("readCamera count=%ld imageOffset=%ld imageSize=%ld", count, imageOffset, imageSize);

    long bytesRead = ucam.getJpegData(cameraChunkBuf, count);
    if (bytesRead <= 0) {
        _log.error("failed to read bytesRead=%ld count=%ld imageOffset=%ld imageSize=%ld", bytesRead, count, imageOffset, imageSize);
        cameraStateHandler = &CameraHelper::stateCameraReset;
        savePictureCleanup();
        return;
    }

    int res = write(saveFd, cameraChunkBuf, bytesRead);
    if (res < 0) {
        _log.error("failed to write file count=%ld imageOffset=%ld imageSize=%ld", count, imageOffset, imageSize);
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        savePictureCleanup();
        return;
    }

    SHA1Update(&imageHash, (const unsigned char *) cameraChunkBuf, bytesRead);    

    imageOffset += bytesRead;
    if (imageOffset >= imageSize) {
        _log.info("file downloaded!");
        cameraStateHandler = &CameraHelper::stateWaitToTakePicture;
        savePictureCleanup();

        // Turn off camera (ignored if no power control available)
        powerControl(false);

        _log.info("saved image file to fileNum=%d", lastFileNum);

        publishQueue.push_back(lastFileNum);

        unsigned char digest[20];
        SHA1Final(digest, &imageHash);    

        char path[PATH_BUF_SIZE];
        int fd;

        // Save the hash value to disk
        snprintf(path, sizeof(path), "%s/%06d.sha1", CAMERA_DIR, lastFileNum);
        fd = open(path, O_RDWR | O_CREAT | O_TRUNC);
        if (fd >= 0) {
            char buf[sizeof(digest) * 2 + 1];
            for(size_t ii = 0; ii < sizeof(digest); ii++) {
                sprintf(&buf[ii * 2], "%02x", digest[ii]);
            }
            write(fd, buf, strlen(buf));
            
            close(fd);
        }

        // Save the lastFileNum to a file so we don't reuse file numbers after deleting
        // them; this helps the server code keep track of which uploads are in
        // progress more easily.
        snprintf(path, sizeof(path), "%s/%s", CAMERA_DIR, FILENUM_FILE_NAME);
        fd = open(path, O_RDWR | O_CREAT | O_TRUNC);
        if (fd >= 0) {
            write(fd, (void *)&lastFileNum, sizeof(lastFileNum));
            
            close(fd);
        }

        return;
    }

}

void CameraHelper::savePictureCleanup() {
    if (saveFd != -1) {
        close(saveFd);
        saveFd = -1;
    }
}

void CameraHelper::stateCameraReset() {
    if (!hasPowerControl()) {
        ucam.hardReset();
        cameraStateHandler = &CameraHelper::stateStartPicture;
        return;
    }

    // Power down camera
    _log.info("powering down camera to reset");
    powerControl(false);
    cameraStateTime = millis();
    cameraStateHandler = &CameraHelper::stateCameraWaitPowerDown;
}


void CameraHelper::stateCameraWaitPowerDown() {
    if (millis() - cameraStateTime < 5000) {
        return;
    }

    // stateStartPicture will turn power back on
    cameraStateHandler = &CameraHelper::stateStartPicture;
}



int CameraHelper::functionHandler(String command) {
    // Control requests are received in JSON format in command via the "camera" function



    String op;
    int fileNum = 0;

    JSONValue outerObj = JSONValue::parseCopy(command.c_str());
    JSONObjectIterator iter(outerObj);
    std::deque<int> chunks;
    
    while(iter.next()) {
        _log.info("key=%s value=%s", (const char *) iter.name(), (const char *) iter.value().toString());

        if (iter.name() == "op") {
            op = (const char *) iter.value().toString();
        }
        else
        if (iter.name() == "file") {
            fileNum = iter.value().toInt();
        }
        else
        if (iter.name() == "chunks") {
            JSONArrayIterator iter2(iter.value());
            while(iter2.next()) {
                chunks.push_back(iter2.value().toInt());
            }
        }        
    }

    if (fileNum != publishFileNum) {
        _log.error("fileNum=%d but expected=%d", fileNum, publishFileNum);
        return 1;
    }

    if (op == "start") {
        // "op"="start" 
        //  "file" = file number
        // This is the ACK from the start command so we know we can proceed
        // with sending the chunks as fast as possible without ACK
        _log.info("start fileNum=%d", fileNum);
        publishStart = true;
    }
    else
    if (op == "resend") {
        // "op"="resend"
        //  "file" = file number to request chunk from
        //  "chunks" (array) = array of integer chunks to resend
        _log.info("resend fileNum=%d", fileNum);
        while(!chunks.empty()) {
            republishQueue.push_back(chunks.front());
            chunks.pop_front();
        }
    }
    else
    if (op == "done") {
        // "op"="done"
        //  "file" = file number 
        // File has been successfully saved, can delete locally
        _log.info("done fileNum=%d publishFileNum=%d", fileNum, publishFileNum);
        publishDone = true;
    }
    else
    if (op == "restart") {
        // "op"="restart" 
        // Start sending this file over again
        _log.info("restart");
        publishRestart = true;
    }
    

    return 0;
}


void CameraHelper::statePublishWaitForFile() {
    if (publishQueue.empty()) {
        // Nothing in the queue
        return;
    }
    if (!Particle.connected()) {
        // No cloud connection
        return;
    }

    // We have a file 
    publishFileNum = publishQueue.front();
    publishQueue.pop_front();

    char path[PATH_BUF_SIZE];

    snprintf(path, sizeof(path), "%s/%06d.sha1", CAMERA_DIR, publishFileNum);
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        _log.error("Failed to open %s errno=%d", path, errno);
        return;
    }
    char hash[41];
    int hashSize = read(fd, (char *)hash, sizeof(hash) - 1);
    close(fd);

    if (hashSize != (sizeof(hash) - 1)) {
        _log.error("invalid hash values %s", (char *)publishTempBuf);
        return;
    }
    hash[sizeof(hash) - 1] = 0;
    _log.trace("hash=%s", hash);

    snprintf(path, sizeof(path), "%s/%06d.jpg", CAMERA_DIR, publishFileNum);

    _log.info("publishing %s", path);

    publishFd = open(path, O_RDONLY);
    if (publishFd < 0) {
        _log.error("Failed to open %s errno=%d", path, errno);
        return;
    }

    // Note the size of the file
    struct stat sb;
    fstat(publishFd, &sb);
    publishFileSize = sb.st_size;
    _log.info("publishFileSize=%u", publishFileSize);

    lseek(publishFd, 0, SEEK_SET);

    publishRestart = false;
    publishStart = false;
    publishDone = false;
    publishChunk = 0;
    publishNumChunks = (publishFileSize + PUBLISH_CHUNK_SIZE - 1) / PUBLISH_CHUNK_SIZE;
    republishQueue.clear();

    // Calculate the hash for the file

    // Initial publish includes:
    // op: "start"
    // fileNum: publishFileNum
    // chunkSize: PUBLISH_CHUNK_SIZE
    // fileSize: file size in bytes
    // hash: SHA1 hash (20 hex bytes, 40 characters of hex digits)

    // Create the JSON object in publishJsonBuf 
    JSONBufferWriter writer(publishJsonBuf, sizeof(publishJsonBuf) - 1);

    writer.beginObject();
    writer.name("op").value("start");
    writer.name("fileNum").value(publishFileNum);
    writer.name("chunkSize").value(PUBLISH_CHUNK_SIZE);
    writer.name("fileSize").value(publishFileSize);
    writer.name("hash").value(hash);
 
    writer.endObject();

    writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;

    // Publish the start packet
    bool res = Particle.publish(CAMERA_EVENT_NAME, publishJsonBuf, PRIVATE | WITH_ACK);
    if (!res) {
        // Retry publish
        _log.error("start publish failed, will retry");
        publishTime = millis();
        publishStateHandler = &CameraHelper::statePublishRetryWait;
        return;
    }

    _log.trace("waiting for start confirmation");
    publishTime = millis();
    publishStateHandler = &CameraHelper::statePublishStartWait;
}

void CameraHelper::statePublishStartWait() {
    if (publishStart) {
        _log.trace("start received, send chunks");
        publishTime = millis();
        publishStateHandler = &CameraHelper::statePublishChunk;
        return;
    }
    if (millis() - publishTime > 20000) {
        // The start packet may have been lost. Retry sending start packet.
        _log.error("start publish didn't get confirmation, retry");
        publishTime = millis();
        publishStateHandler = &CameraHelper::statePublishRetryWait;
        return;
    }
}


void CameraHelper::statePublishChunk() {
    if (publishRestart) {
        // Server probably restarted and needs to start over from beginning
        publishQueue.push_front(publishFileNum);
        publishStateHandler = &CameraHelper::statePublishWaitForFile;
        return;
    }

    if (millis() - publishTime <= 1000) {
        return;
    }

    if (!Particle.connected()) {
        return;
    }

    if (publishDone) {
        _log.info("publish done!");
        close(publishFd);
        publishFd = -1;

        char path[PATH_BUF_SIZE];
        snprintf(path, sizeof(path), "%s/%06d.sha1", CAMERA_DIR, publishFileNum);
        unlink(path);

        snprintf(path, sizeof(path), "%s/%06d.jpg", CAMERA_DIR, publishFileNum);
        unlink(path);

        _log.info("removed %s", path);
        publishStateHandler = &CameraHelper::statePublishWaitForFile;
        return;
    }

    // Each chunk includes:
    // op: "chunk"
    // fileNum: publishFileNum
    // chunk: publishChunk (chunk index 0-based), base64 encoded
    
    int chunkToSend = publishChunk;

    // If there are chunks in the queue to resend, send those first
    if (!republishQueue.empty()) {
        chunkToSend = republishQueue.front();
        republishQueue.pop_front();

        _log.info("republish chunk=%d", chunkToSend);
    }

    // Read a chunk of data (PUBLISH_CHUNK_SIZE) into publishJsonBuf
    // (it's not JSON, but we need to save it into publishTempBuf
    // when we do Base64 encoding).

    size_t chunkOffset = chunkToSend * PUBLISH_CHUNK_SIZE;

    if (chunkOffset >= publishFileSize && republishQueue.empty()) {
        // Wait for more republish requests or the done flag (publishDone)
        return;
    }

    size_t count = publishFileSize - chunkOffset;
    if (count > PUBLISH_CHUNK_SIZE) {
        count = PUBLISH_CHUNK_SIZE;
    }

    lseek(publishFd, chunkOffset, SEEK_SET);

    int err = read(publishFd, publishJsonBuf, count);
    if (err < 0) {
        _log.error("Failed to read file");
        
        // TODO: Better error handling here
        publishStateHandler = &CameraHelper::statePublishWaitForFile;
        return;
    }

    // Base64 encode the chunk
    size_t dstLen = PUBLISH_TEMP_BUF_SIZE;
    Base64::encode((const uint8_t *)publishJsonBuf, count, (char *)publishTempBuf, dstLen, true /* nullTerminate */);

    // Create the JSON object in publishJsonBuf 
    JSONBufferWriter writer(publishJsonBuf, sizeof(publishJsonBuf) - 1);

    writer.beginObject();
    writer.name("op").value("chunk");
    writer.name("fileNum").value(publishFileNum);
    writer.name("chunk").value(chunkToSend);
    writer.name("data").value((const char *)publishTempBuf);
    writer.endObject();

    writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;

    if (chunkToSend == publishChunk) {
        publishChunk++;

        // These are here to test the missing chunks. Do not enable in real code!
        // if (publishChunk == 5) { return; }
        // if (publishChunk == 8) { return; }
    }
    
    // Publish the data
    // TODO: Probably use a future here instead
    _log.trace("publishing chunk=%d", chunkToSend);
    bool res = Particle.publish(CAMERA_EVENT_NAME, publishJsonBuf, PRIVATE | NO_ACK);
    if (!res) {
        // Retry publish
        _log.error("chunk publish failed, will retry");
        publishTime = millis();
        publishStateHandler = &CameraHelper::statePublishRetryWait;
        return;
    }

    _log.info("successfully published chunk %d", chunkToSend);

    publishTime = millis();
}


void CameraHelper::statePublishRetryWait() {
    if (millis() - publishTime <= 1000) {
        return;
    }
    if (!Particle.connected()) {
        return;
    }

    _log.trace("republishing");
    bool res = Particle.publish(CAMERA_EVENT_NAME, publishJsonBuf, PRIVATE | NO_ACK);
    if (res) {
        // Success
        if (publishStart) {
            // Send another chunk
            publishStateHandler = &CameraHelper::statePublishChunk;     
        }
        else {
            // We just retransmitted the start packet, not the chunk packet
            publishStateHandler = &CameraHelper::statePublishStartWait;     
        }
    }
    publishTime = millis();
}


// [static]
bool CameraHelper::createDirIfNecessary(const char *path) {
    struct stat statbuf;

    int result = stat(path, &statbuf);
    if (result == 0) {
        if ((statbuf.st_mode & S_IFDIR) != 0) {
            _log.info("%s exists and is a directory", path);
            return true;
        }

        _log.error("file in the way, deleting %s", path);
        unlink(path);
    }
    else {
        if (errno != ENOENT) {
            // Error other than file does not exist
            _log.error("stat filed errno=%d", errno);
            return false;
        }
    }

    // File does not exist (errno == 2)
    result = mkdir(path, 0777);
    if (result == 0) {
        _log.info("created dir %s", path);
        return true;
    }
    else {
        _log.error("mkdir failed errno=%d", errno);
        return false;
    }
}


CameraHelperTracker::CameraHelperTracker(USARTSerial &serial, int baud, int resetPin) : CameraHelper(serial, baud, resetPin) {

}

CameraHelperTracker::~CameraHelperTracker() {

}

void CameraHelperTracker::powerControl(bool on) {
    _log.info("camera power %s", (on ? "on" : "off"));

    pinMode(CAN_PWR, OUTPUT);
    digitalWrite(CAN_PWR, on ? HIGH : LOW);
}
