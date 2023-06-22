#ifndef CAMERA_H
#define CAMERA_H

#include <cstdio>
#include <stdexcept>
#include <unistd.h>
#include <cassert>
#include <vector>
#include <pthread.h>
#include <csignal>
#include <thread>

#include <sys/syscall.h>
#include <sys/types.h>
#include <ctime>
#include <sys/resource.h>
#include <opencv2/opencv.hpp>
#include <PixeLINKApi.h>
#include <PixeLINKCodes.h>

#define API_RANGE_ERROR(rc) (((PXL_RETURN_CODE)(rc) == ApiInvalidParameterError) || ((PXL_RETURN_CODE)(rc) == ApiOutOfRangeError))
#define UNUSED(x) (void)(x)
#define WAIT_SECONDS 20

enum OneTimeState {
    Inactive,
    InProgress,
    Stopping
};

enum ColorChannels {
    RedChannel,
    GreenChannel,
    BlueChannel,
    NumColorChannels = BlueChannel
};

class Camera {
  private:
    float pixelFormat{PIXEL_FORMAT_BAYER8};
    std::vector<ULONG> connectedCameraList;
    bool operationReturn{false};

    PXL_RETURN_CODE getValue(ULONG feature, float* value) const;
    void abortAutoWhiteBalance();

    OneTimeState nextImageState{Inactive};
    pthread_t nextImageThread{};

    OneTimeState autoExposeState{Inactive};
    pthread_t autoExposeThread{};

    OneTimeState autoBalanceState{Inactive};
    pthread_t autoBalanceThread{};

    void* GetNextFrame();
    static void* nextImageHelper(void* nothing) {
        return ((Camera*)nothing)->GetNextFrame();
    }

    void* autoExposure();
    static void* autoExposureHelper(void* nothing) {
        return ((Camera*)nothing)->autoExposure();
    }

    void* autoWhiteBalance();
    static void* autoWhiteBalanceHelper(void* nothing) {
        return ((Camera*)nothing)->autoWhiteBalance();
    }

    PXL_RETURN_CODE scanForCameras();
    ULONG imageSizeInBytes();
    ULONG imageSizeInPixels();
    static float pixelSize(ULONG pxlFmt);
  public:
    Camera() = default;
    virtual ~Camera();

    FRAME_DESC frameDesc{};
    HANDLE hCamera{};

    bool initialized{false};
    bool triggering{false};
    bool streaming{false};
    double exposure{0.018};
    float sensorTempLast{0.0f};
    float bodyTempLast{0.0f};

    time_t startTime{}, currentTime{};
    U32 frameCount{0}, previousFrame{0};
    float currentFps{0.0f};
    float firstFrameTime{0.0f};
    int trigMisses{0};

    U32 rawImageHeight{};
    U32 rawImageWidth{};
    U32 rawImageSize{};

    std::vector<U8> frameBuffer;
    cv::Mat frameMatrix;

    //CAMERA FUNCTIONS
    PXL_RETURN_CODE initialize();
    PXL_RETURN_CODE uninitialize(const std::string& calledFrom);
    PXL_RETURN_CODE setExposure(float newExp);
    PXL_RETURN_CODE bumpExposure(bool up);
    PXL_RETURN_CODE updateExposure(bool rewrite);
    PXL_RETURN_CODE disableTriggering() const;
    PXL_RETURN_CODE setFPS(bool autoOn, float fps) const;
    PXL_RETURN_CODE setDefaultColor();
    PXL_RETURN_CODE setFactoryColor();
    PXL_RETURN_CODE getCurrentTemperatures();
    PXL_RETURN_CODE setTriggering();
    PXL_RETURN_CODE stopTriggering();
    PXL_RETURN_CODE startStream();
    PXL_RETURN_CODE stopStream();
    static PXL_RETURN_CODE formatImage(const void* pFrame, const FRAME_DESC* pFrameDesc, ULONG format, ULONG* bufferSize, void* pImage) ;

    void initiateGetNextFrame();
    void initiateAutoExposure();
    void initiateAutoWhiteBalance();
    float getColor(const std::string& col);
    OneTimeState getAutoBalanceState() const;
    OneTimeState getAutoExposeState() const;
};

#endif // CAMERA_H
