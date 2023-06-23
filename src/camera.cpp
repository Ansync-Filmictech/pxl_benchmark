#include "camera.h"

//----------------------------------------------------------------
// Name:
//      CameraCapture destructor
//
// Description:
//      removes instance of CameraCapture as well as stops camera
//      stream.
//
// Notes:
//      - called during camera reset and app exit
//----------------------------------------------------------------
Camera::~Camera() {
    //PxLSetStreamState(hCamera, STOP_STREAM);
    stopStream();
	PxLUninitialize(hCamera);
}

//----------------------------------------------------------------
// Name:
//      initialize
//
// Description:
//      Initializes camera and starts stream with one call
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - fully resets camera to default for fresh use
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::initialize() {
    PXL_RETURN_CODE rc;
    rc = scanForCameras();

    if (!API_SUCCESS(rc) || connectedCameraList.empty()) {
        printf("Error 0x%X!: Unable to find connected cameras\n", rc);
        return rc;
    }

    rc = PxLInitializeEx(connectedCameraList[0], &hCamera, CAMERA_INITIALIZE_EX_FLAG_ISSUE_STREAM_STOP);

    if (API_SUCCESS(rc)) {
        rc = PxLLoadSettings(hCamera, FACTORY_DEFAULTS_MEMORY_CHANNEL);
        if (!API_SUCCESS(rc)) {
            printf("Could not load factory settings!\n");
        } else {
            printf("Factory default settings restored\n");
        }
    } else {
        printf("Error 0x%X!: Unable to initialize a camera\n", rc);
        return rc;
    }

    // Start with triggering disabled so we start with a clean slate
    rc = stopTriggering();
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to disable triggering\n", rc);
        return rc;
    }

    // Tell the API to use frame buffering while triggered
    U32  useBufferingCommand[] =
      { 49,    // 49 == command code for ALLOW_BUFFERED_FRAMES_WITH_TRIGGER
        1  };  // 1 == TRUE.  Set to 0 to restore the default behavior for PxLGetNextFrame

    rc = PxLPrivateCmd(hCamera, // Handle to the camera, returned via PxLInitializeEx
      sizeof(useBufferingCommand),
      useBufferingCommand);

    //Set to manual to prevent bad stuff on trigger switch

    CAMERA_INFO cameraInfo;
    rc = PxLGetCameraInfoEx(hCamera, &cameraInfo, sizeof(cameraInfo));

    if (API_SUCCESS(rc)) {
        printf("Camera Name : %s\n", cameraInfo.CameraName);
        printf("Model Name  : %s\n", cameraInfo.ModelName);
        printf("Description : %s\n", cameraInfo.Description);
        printf("Serial #    : %s\n", cameraInfo.SerialNumber);
        printf("Firmware    : %s\n", cameraInfo.FirmwareVersion);
        printf("FPGA        : %s\n", cameraInfo.FPGAVersion);
        printf("XML         : %s\n", cameraInfo.XMLVersion);
    }

    // Set Pixel Format of camera
    rc = PxLSetFeature(hCamera, FEATURE_PIXEL_FORMAT, FEATURE_FLAG_MANUAL, 1, &pixelFormat);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to set camera format\n", rc);
        return rc;
    }
    
    float ctValue;
    rc = PxLSetFeature(hCamera, FEATURE_COLOR_TEMP, FEATURE_FLAG_MANUAL | FEATURE_FLAG_OFF, 1, &ctValue);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to disable Color Temp\n", rc);
    }

    float gammaValue = 1.5f;
    rc = PxLSetFeature(hCamera, FEATURE_GAMMA, FEATURE_FLAG_MANUAL, 1, &gammaValue);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to set gamma\n", rc);
    }

    float saturationValue = 120.f;
    rc = PxLSetFeature(hCamera, FEATURE_SATURATION, FEATURE_FLAG_MANUAL, 1, &saturationValue);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to set gamma\n", rc);
    }

    
    rc = setDefaultColor();
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Unable to set color defaults\n", rc);
    }
  
    rawImageSize = imageSizeInBytes();
    frameBuffer.resize(rawImageSize);
    
    setFPS(false, 24.0);
    
    rc = startStream();

    printf("Success: Camera initialized and stream started\n");
    initialized = true;
   
    return rc;
}

PXL_RETURN_CODE Camera::scanForCameras() {
    PXL_RETURN_CODE rc = ApiSuccess;
    ULONG numCameras = 0;

    connectedCameraList.clear();

    rc = PxLGetNumberCameras(nullptr, &numCameras);
    if (API_SUCCESS(rc) && numCameras > 0) {
        connectedCameraList.resize(numCameras);
        rc = PxLGetNumberCameras(&connectedCameraList[0], &numCameras);
        if (!API_SUCCESS(rc)) {
            connectedCameraList.clear();
        }
    }

    return rc;
}

// returns the size of images from the camera (in bytes)
ULONG Camera::imageSizeInBytes() {
    PXL_RETURN_CODE rc = ApiSuccess;

    float params[4];
    U32 pxlFmt;
    auto numPixels = (float) imageSizeInPixels();
    U32 flags = FEATURE_FLAG_MANUAL;
    U32 numParams;

    assert(nullptr != hCamera);

    // Knowing pixel format means we can determine how many bytes per pixel.
    numParams = 1;
    rc = PxLGetFeature(hCamera, FEATURE_PIXEL_FORMAT, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) return 0;
    pxlFmt = (U32) params[0];

    return (U32) (numPixels * pixelSize(pxlFmt));
}

ULONG Camera::imageSizeInPixels() {
    PXL_RETURN_CODE rc = ApiSuccess;

    float params[4];     // reused for each feature query
    U32 roiWidth;
    U32 roiHeight;
    U32 pixelAddressingValue;       // integral factor by which the image is reduced
    U32 pxlFmt;
    float numPixels;
    U32 flags = FEATURE_FLAG_MANUAL;
    U32 numParams;

    assert(nullptr != hCamera);

    // Get region of interest (ROI)
    numParams = 4; // left, top, width, height
    rc = PxLGetFeature(hCamera, FEATURE_ROI, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) return 0;
    roiWidth = (U32) params[FEATURE_ROI_PARAM_WIDTH];
    roiHeight = (U32) params[FEATURE_ROI_PARAM_HEIGHT];
    rawImageHeight = roiHeight;
    rawImageWidth = roiWidth;

    // Query pixel addressing
    numParams = 2; // pixel addressing value, pixel addressing type (e.g. bin, average, ...)
    rc = PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) return 0;
    pixelAddressingValue = (U32) params[FEATURE_PIXEL_ADDRESSING_PARAM_VALUE];

    // We can calculate the number of pixels now.
    numPixels = (static_cast<float>(roiWidth) / static_cast<float>(pixelAddressingValue)) *
                (static_cast<float>(roiHeight) / static_cast<float>(pixelAddressingValue));

    // Knowing pixel format means we can determine how many bytes per pixel.
    numParams = 1;
    rc = PxLGetFeature(hCamera, FEATURE_PIXEL_FORMAT, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) return 0;
    pxlFmt = (U32) params[0];

    return (U32) (numPixels * pixelSize(pxlFmt));
}

inline float Camera::pixelSize(ULONG pxlFmt) {
    float retVal = 0.f;
    switch (pxlFmt) {

        case PIXEL_FORMAT_MONO8:
        case PIXEL_FORMAT_BAYER8_GRBG:
        case PIXEL_FORMAT_BAYER8_RGGB:
        case PIXEL_FORMAT_BAYER8_GBRG:
        case PIXEL_FORMAT_BAYER8_BGGR:
            retVal = 1.0f;
            break;
        case PIXEL_FORMAT_MONO12_PACKED:
        case PIXEL_FORMAT_BAYER12_GRBG_PACKED:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED:
        case PIXEL_FORMAT_MONO12_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_GRBG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED_MSFIRST:
            return 1.5f;
        case PIXEL_FORMAT_YUV422:
        case PIXEL_FORMAT_MONO16:
        case PIXEL_FORMAT_BAYER16_GRBG:
        case PIXEL_FORMAT_BAYER16_RGGB:
        case PIXEL_FORMAT_BAYER16_GBRG:
        case PIXEL_FORMAT_BAYER16_BGGR:
            retVal = 2.0f;
            break;
        case PIXEL_FORMAT_RGB24:
            retVal = 3.0f;
            break;
        case PIXEL_FORMAT_RGB48:
            retVal = 6.0f;
            break;
        default:
            assert(0);
            break;
    }
    return retVal;
}

//----------------------------------------------------------------
// Name:
//      setExposure
//
// Description:
//      Sets the exposure for the camera
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called during setup of camera
//
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::setExposure(float newExp) {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 1;
    float expParam;

    rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &expParam);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to get expParam failed\n", rc);
        return rc;
    }

    newExp = newExp / 1000.f;
    if (newExp < 0.000020) {
        newExp = 0.000020f;
    }

    rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, FEATURE_FLAG_MANUAL, numParams, &newExp);
    if (!API_SUCCESS(rc) && !API_RANGE_ERROR(rc)) {
        printf("Error 0x%X!: Attempt to set expParam failed\n", rc);
        return rc;
    }

    return updateExposure(true);
}

//----------------------------------------------------------------
// Name:
//      bumpExposure
//
// Description:
//      Manually adjust the exposure, increasing it by 10%
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called by +/- button on gui
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::bumpExposure(const bool up) {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 1;
    float expParam;

    rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &expParam);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to get expParam failed\n", rc);
        return rc;
    }

    if (up) expParam *= 1.1;
    else expParam /= 1.1;

    if (expParam < 0.000020) {
        expParam = 0.000020f;
    }

    rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, FEATURE_FLAG_MANUAL, numParams, &expParam);
    if (!API_SUCCESS(rc) && !API_RANGE_ERROR(rc)) {
        printf("Error 0x%X!: Attempt to set expParam failed\n", rc);
        return rc;
    }

    return updateExposure(true);
}

//----------------------------------------------------------------
// Name:
//      updateExposure
//
// Description:
//      updates exposure variable to human readable number
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - exposure pulled from camera is not the same that is
//        displayed in gui.  this is an abstracted number in order
//        to read clearer.
//      - called after new exposure is sent to camera
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::updateExposure(const bool rewrite) {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 1;
    float newExposure;

    rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &newExposure);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to get shutter speed failed\n", rc);
    }

    newExposure *= 1000;

    exposure = newExposure;
    return rc;
}

//----------------------------------------------------------------
// Name:
//      initiateOneTimeAutoExposure
//
// Description:
//      creates pthread for auto exposure feature of camera
//
// Notes:
//      - called by auto button on gui
//----------------------------------------------------------------
void Camera::initiateAutoExposure() {
    if (autoExposeState == Inactive) {
        pthread_attr_t attr;

        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        int s = pthread_create(&autoExposeThread, &attr, &Camera::autoExposureHelper, this);
        if (s != 0) {
            printf("Error %d: Could not create a one time auto exposure thread!\n", s);
        }

        pthread_attr_destroy(&attr);
    }
}

//----------------------------------------------------------------
// Name:
//      performOneTimeAutoExposure
//
// Description:
//      thread to perform a one-time auto expose operation, ending
//      when the operation is complete, or until told to abort
//
// Notes:
//      - called by initiateOneTimeAutoExposure
//----------------------------------------------------------------.
void *Camera::autoExposure() {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 1;
    float expParam = 0.0;  // Intialize to 0, but this value is ignored when initating auto adjustment.

    autoExposeState = InProgress;
    printf("Starting one time auto expParam adjustment.\n");

    rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, FEATURE_FLAG_ONEPUSH, numParams, &expParam);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to set auto expParam returned\n", rc);
        autoExposeState = Inactive;
        return nullptr;
    }

    // Now that we have initiated a one time operation, loop until it is done (or told to abort).
    while (autoExposeState == InProgress) {
        rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &expParam);
        if (API_SUCCESS(rc)) {
            if (!(flags & FEATURE_FLAG_ONEPUSH)) break;
        }
        usleep(250 * 1000);  // 250 ms - Give some time for the one time operation to complete
    }

    printf("Finished one time auto expParam adjustment. %s\n",
           autoExposeState == Stopping
           ? "Operation aborted.                "
           : "Operation completed successfully. ");

    autoExposeState = Inactive;

    updateExposure(true);
    return nullptr;
}

//----------------------------------------------------------------
// Name:
//      DisableTriggering
//
// Description:
//      Disables camera triggering feature
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called by stopTriggering
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::disableTriggering() const {
/*    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 5;
    float params[5];
	bool pause_stream = streaming;
    
    float fixedValue{0};
    rc = getValue(FEATURE_SPECIAL_CAMERA_MODE, &fixedValue);
    printf("SPECIAL_MODE: %f\n", fixedValue);
    if (API_SUCCESS(rc) && fixedValue == (float) FEATURE_SPECIAL_CAMERA_MODE_FIXED_FRAME_RATE) {
        rc = PxLSetFeature(hCamera,
                               FEATURE_SPECIAL_CAMERA_MODE,
                               FEATURE_FLAG_MANUAL,
                               1,
                               FEATURE_SPECIAL_CAMERA_MODE_NONE);

        if (!API_SUCCESS(rc)) {
            printf("Error 0x%X!: Attempt to set no special camera mode failed\n", rc);
            return rc;
        }
    }
    printf("SPECIAL_MODE: %f\n", fixedValue);


	if(pause_stream){
		rc = stopStream();
		if(!API_SUCCESS(rc))
			return rc;
	}

    // Read current settings
    rc = PxLGetFeature(hCamera, FEATURE_TRIGGER, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to get trigger feature returned\n", rc);
        return rc;
    }
    if (5 != numParams) {
        printf("Error: Number of parameters for trigger feature incorrect\n");
        return -1;
    }
    printf("MODE: %f\nTYPE: %f\nPOL: %f\nDELAY: %f\nPARAM: %f\nFLAGS: %d\n",
      params[FEATURE_TRIGGER_PARAM_MODE],
      params[FEATURE_TRIGGER_PARAM_TYPE],
      params[FEATURE_TRIGGER_PARAM_POLARITY],
      params[FEATURE_TRIGGER_PARAM_DELAY],
      params[FEATURE_TRIGGER_PARAM_PARAMETER],
      flags);

    // Disable triggering
    flags = FEATURE_FLAG_OFF | FEATURE_FLAG_PRESENCE;

    // Assign the default values...
    params[FEATURE_TRIGGER_PARAM_MODE] = TRIGGER_MODE_0;
    params[FEATURE_TRIGGER_PARAM_TYPE] =  TRIGGER_TYPE_FREE_RUNNING;
    params[FEATURE_TRIGGER_PARAM_POLARITY] = POLARITY_ACTIVE_LOW;
    params[FEATURE_TRIGGER_PARAM_DELAY] = 0;
    params[FEATURE_TRIGGER_PARAM_PARAMETER] = 0;

    rc = PxLSetFeature(hCamera, FEATURE_TRIGGER, flags, numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to set trigger feature failed\n", rc);
        return rc;
    }

    printf("MODE: %f\nTYPE: %f\nPOL: %f\nDELAY: %f\nPARAM: %f\nFLAGS: %d\n",
      params[FEATURE_TRIGGER_PARAM_MODE],
      params[FEATURE_TRIGGER_PARAM_TYPE],
      params[FEATURE_TRIGGER_PARAM_POLARITY],
      params[FEATURE_TRIGGER_PARAM_DELAY],
      params[FEATURE_TRIGGER_PARAM_PARAMETER],
      flags);

	if(pause_stream){
		rc = startStream();
		if(!API_SUCCESS(rc))
			return rc;
	}*/
    return 0;
}

//----------------------------------------------------------------
// Name:
//      setFPS
//
// Description:
//      sets the value for the frames per second of the camera
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called during setup of camera
//
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::setFPS(const bool autoOn, const float fps) const {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 1;
    float value = fps;

    flags = autoOn ? FEATURE_FLAG_OFF : FEATURE_FLAG_MANUAL;
    rc = PxLSetFeature(hCamera, FEATURE_FRAME_RATE, flags, numParams, &value);

    return rc;
}

//----------------------------------------------------------------
// Name:
//      setDefaultColor
//
//
// Description:
//      sets the default colors for the camera.
//
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called in setup for camera
//      - sets colors to default values during setup
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::setDefaultColor() {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 3;
    std::vector<float> cameraColors(numParams);

    // Get the current color settings
    rc = PxLGetFeature(hCamera, FEATURE_WHITE_SHADING, &flags, &numParams, &cameraColors[0]);
    if (!API_SUCCESS(rc)) {
        return rc;
    }


    // set the new color value, if it's in range
    rc = PxLSetFeature(hCamera, FEATURE_WHITE_SHADING, flags, numParams, &cameraColors[0]);
    if (!API_SUCCESS(rc)) {
        printf("Attempt to set White Balance to R:%f, G:%f, B:%f returned 0x%X!\n", cameraColors[0], cameraColors[1],
               cameraColors[2], rc);
    } else {
        printf("Final balance rc=0x%x --> R:%f, G:%f B:%f\n", rc, cameraColors[0], cameraColors[1], cameraColors[2]);
    }

    return rc;
}

//----------------------------------------------------------------
// Name:
//     setFactoryColor 
//
//
// Description:
//      sets the factory colors for the camera.
//
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - called in setup for camera
//      - sets colors to default values during setup
//----------------------------------------------------------------

PXL_RETURN_CODE Camera::setFactoryColor() {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = 3;
    std::vector<float> cameraColors(numParams);

    // Get the current color settings
    rc = PxLGetFeature(hCamera, FEATURE_WHITE_SHADING, &flags, &numParams, &cameraColors[0]);
    if (!API_SUCCESS(rc)) {
        return rc;
    }

    // adjust the specified color
    cameraColors[RedChannel] = 1.0;
    cameraColors[GreenChannel] = 1.0;
    cameraColors[BlueChannel] = 1.0;

    // set the new color value, if it's in range
    rc = PxLSetFeature(hCamera, FEATURE_WHITE_SHADING, flags, numParams, &cameraColors[0]);
    if (!API_SUCCESS(rc)) {
        printf("Attempt to set White Balance to R:%f, G:%f, B:%f returned 0x%X!\n", cameraColors[0], cameraColors[1],
               cameraColors[2], rc);
    } else {
        printf("Final balance rc=0x%x --> R:%f, G:%f B:%f\n", rc, cameraColors[0], cameraColors[1], cameraColors[2]);
    }

    return rc;
}
//----------------------------------------------------------------
// Name:
//      performOneTimeAutoWhiteBalance
//
// Description:
//      Performs one time white balance on camera
//
// Notes:
//
//----------------------------------------------------------------
void *Camera::autoWhiteBalance() {
    PXL_RETURN_CODE rc;
    float params[5];
    ULONG flags;
    ULONG numParams;
    int i;

    autoBalanceState = InProgress;

    // Step 2 - Get the current ROI
    numParams = 4;
    rc = PxLGetFeature(hCamera, FEATURE_ROI, &flags, &numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("  Error - PxLGetFeature(FEATURE_ROI) returned 0x%x\n", rc);
        PxLUninitialize(hCamera);
        return nullptr;
    }

    // Step 3 - Set the AUTO_ROI to a 256x256 window in the middle right of the roi
    params[0] = 200;
    params[1] = 1024 - 256.0;
    params[2] = params[3] = 256.0;
    rc = PxLSetFeature(hCamera, FEATURE_AUTO_ROI, FEATURE_FLAG_MANUAL, numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("  Error - PxLSetFeature(FEATURE_AUTO_ROI) returned 0x%x\n", rc);
        PxLUninitialize(hCamera);
        return nullptr;
    }

    // Step 4 - Perform a one-time, auto white balance
    params[0] = params[1] = params[2] = 1.0f;
    rc = PxLSetFeature(hCamera, FEATURE_WHITE_SHADING, FEATURE_FLAG_ONEPUSH, 3, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("Error - PxLSetFeature(FEATURE_WHITE_SHADING) returned 0x%x\n", rc);
        PxLUninitialize(hCamera);
        return nullptr;
    }

    // Step 5 - Wait for the white balance to complete
    printf("Waiting on White Balance to complete\n");
    for (i = 0; i < WAIT_SECONDS; i++) {
        rc = PxLGetFeature(hCamera, FEATURE_WHITE_SHADING, &flags, &numParams, &params[0]);
        if (!(flags & FEATURE_FLAG_ONEPUSH)) break;
        printf("  Interim balance --> R:%f, G:%f B:%f\n", params[0], params[1], params[2]);
        sleep(1);
    }

    if (!API_SUCCESS(rc)) {
        printf("Error - PxLGetFeature(FEATURE_WHITE_SHADING) returned 0x%x\n", rc);
        abortAutoWhiteBalance();
        PxLUninitialize(hCamera);
        return nullptr;
    }

    // The auto white balance completed successfully or with a warning -- or we
    // got tired of wating.
    if (i == WAIT_SECONDS) {
        printf("Tired of waiting on the white balance, aborting it\n");
        abortAutoWhiteBalance();
    } else {
        printf("Final balance rc=0x%x --> R:%f, G:%f B:%f\n", rc, params[0], params[1], params[2]);
    }

    autoBalanceState = Inactive;

    return nullptr;
}

//----------------------------------------------------------------
// Name:
//      abortAutoWhiteBalance
//
// Description:
//      cancels auto white balance
//
// Notes:
//      - called in performOneTimeAutoWhiteBalance
//      - prints error to console if unsuccessful
//----------------------------------------------------------------
void Camera::abortAutoWhiteBalance() {
    PXL_RETURN_CODE rc;
    float cameraColors[3];


    rc = PxLSetFeature(hCamera, FEATURE_WHITE_SHADING, FEATURE_FLAG_MANUAL, 3, cameraColors);
    if (!API_SUCCESS(rc)) {
        printf("  Error - PxLSetFeature to cancel AutoWB returned 0x%x\n", rc);
    }
}

//----------------------------------------------------------------
// Name:
//      initiateOneTimeWhiteBalance
//
// Description:
//      instantiates a thread for one time white balance
//
// Notes:
//
//----------------------------------------------------------------
void Camera::initiateAutoWhiteBalance() {
    if (autoBalanceState == Inactive) {
        pthread_attr_t attr;

        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        int s = pthread_create(&autoBalanceThread, &attr, &Camera::autoWhiteBalanceHelper, this);
        if (s != 0) {
            printf("Error %d: Could not create a one time auto white balance thread!\n", s);
        }

        pthread_attr_destroy(&attr);
    }
}

//----------------------------------------------------------------
// Name:
//      UnitializeCamera
//
// Description:
//      Uninitializes camera to prevent errors on next use
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - sets initialization state to false
//      - called by multiple functions and areas of app
//        that have access to CameraCapture class
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::uninitialize(const std::string &calledFrom) {
    PXL_RETURN_CODE rc;
    //PxLSetStreamState(hCamera, STOP_STREAM);
    stopStream();
	rc = PxLUninitialize(hCamera);
    if (!API_SUCCESS(rc)) {
        printf("ERROR 0x%X!: Unable to uninitialize camera\n", rc);
        return rc;
    }

    printf("Success: Camera uninitialized from %s\n", calledFrom.c_str());
    initialized = false;
    return rc;
}

//----------------------------------------------------------------
// Name:
//      GetCurrentTemperatures
//
//
// Description:
//      Gets the current temps on the camera
//
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::getCurrentTemperatures() {
    PXL_RETURN_CODE rc = ApiSuccess;
    float temp = 0.0f;

    rc = getValue(FEATURE_SENSOR_TEMPERATURE, &temp);
    if (API_SUCCESS(rc)) sensorTempLast = temp;

    rc = getValue(FEATURE_BODY_TEMPERATURE, &temp);
    if (API_SUCCESS(rc)) bodyTempLast = temp;

    return rc;
}

//----------------------------------------------------------------
// Name:
//      getValue
//
//
// Description:
//      gets the value of the feature that is passed
//      to the function
//
// Returns:
//      Pixelink return code for api success checks
//
// Notes:
//      - sets the value passed to the current feature value
//----------------------------------------------------------------
PXL_RETURN_CODE Camera::getValue(const ULONG feature, float *value) const {
    PXL_RETURN_CODE rc = ApiSuccess;
    float featureValue;
    ULONG flags;
    ULONG numParams = 1;

    rc = PxLGetFeature(hCamera, feature, &flags, &numParams, &featureValue);
    if (!API_SUCCESS(rc)) return rc;

    *value = featureValue;

    return ApiSuccess;
}


//TODO: Meerge setTrigger and stopTrigger to 1 function
PXL_RETURN_CODE Camera::setTriggering() {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = FEATURE_TRIGGER_NUM_PARAMS;
    float params[FEATURE_TRIGGER_NUM_PARAMS];
    bool pause_stream = streaming;
    triggering = true;

	if(pause_stream){
		rc = stopStream();
		if(!API_SUCCESS(rc))
			return rc;
	}

    setFPS(false, 40.0);
   
	//Assign values for hardware Triggering	
    flags = FEATURE_FLAG_MANUAL | FEATURE_FLAG_PRESENCE;
    params[FEATURE_TRIGGER_PARAM_MODE]      = TRIGGER_MODE_0;
    params[FEATURE_TRIGGER_PARAM_TYPE]      = TRIGGER_TYPE_HARDWARE;
    params[FEATURE_TRIGGER_PARAM_POLARITY]  = POLARITY_ACTIVE_LOW;
    params[FEATURE_TRIGGER_PARAM_DELAY]     = 0;
    params[FEATURE_TRIGGER_PARAM_PARAMETER] = 0;

    //Write them to the camera
    rc = PxLSetFeature(hCamera, FEATURE_TRIGGER, flags, numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to set trigger feature failed\n", rc);
        return rc;
    }
    printf("Success: Camera triggering set\n");

	if(pause_stream){
		rc = startStream();
		if(!API_SUCCESS(rc))
			return rc;
	}

    setExposure(.18);
    firstFrameTime = frameDesc.fFrameTime;
    return rc;
}

PXL_RETURN_CODE Camera::stopTriggering() {
    PXL_RETURN_CODE rc;
    U32 flags;
    U32 numParams = FEATURE_TRIGGER_NUM_PARAMS;
    float params[FEATURE_TRIGGER_NUM_PARAMS];
    bool pause_stream = streaming;
    triggering = false;

	if(pause_stream){
		rc = stopStream();
		if(!API_SUCCESS(rc))
			return rc;
	}
		
	setFPS(false, 24.0);

    //Assign values from power up
    flags = FEATURE_FLAG_OFF | FEATURE_FLAG_MANUAL | FEATURE_FLAG_PRESENCE;
    params[FEATURE_TRIGGER_PARAM_MODE]      = TRIGGER_MODE_0;
    params[FEATURE_TRIGGER_PARAM_TYPE]      = TRIGGER_TYPE_HARDWARE;
    params[FEATURE_TRIGGER_PARAM_POLARITY]  = POLARITY_ACTIVE_LOW;
    params[FEATURE_TRIGGER_PARAM_DELAY]     = 0;
    params[FEATURE_TRIGGER_PARAM_PARAMETER] = 0;

	//Write them to the camera
    rc = PxLSetFeature(hCamera, FEATURE_TRIGGER, flags, numParams, &params[0]);
    if (!API_SUCCESS(rc)) {
        printf("Error 0x%X!: Attempt to set trigger feature failed\n", rc);
        return rc;
    }

    if(pause_stream){
        rc = startStream();
        if(!API_SUCCESS(rc))
            return rc;
	}
    return rc;
}

//----------------------------------------------------------------
// Name: formatImage
//
//
// Description: formats an image using PxLFormatImage
//
//
// Returns: returns a PXL_RETURN_CODE to report if the
//          operation was successful.
//
// Notes: -called in formatFrame()
//
//----------------------------------------------------------------
PXL_RETURN_CODE
Camera::formatImage(const void *pFrame, const FRAME_DESC *pFrameDesc, const ULONG format, ULONG *bufferSize,
                    void *pImage) {
    return PxLFormatImage(pFrame, pFrameDesc, format, pImage, bufferSize);
}

OneTimeState Camera::getAutoBalanceState() const {
    return autoBalanceState;
}

OneTimeState Camera::getAutoExposeState() const {
    return autoExposeState;
}

PXL_RETURN_CODE Camera::startStream(){
  PXL_RETURN_CODE rc = 0;
  rc = PxLSetStreamState(hCamera, START_STREAM);
  if (!API_SUCCESS(rc)) 
    printf("Error 0x%X!: Attempt to get stop stream failed\n", rc);
  else
    streaming = true;
  return rc;
}

PXL_RETURN_CODE Camera::stopStream(){
  PXL_RETURN_CODE rc = 0;
  rc = PxLSetStreamState(hCamera, STOP_STREAM);
  if (!API_SUCCESS(rc)) 
    printf("Error 0x%X!: Attempt to get stop stream failed\n", rc);
  else
    streaming = false;
  return rc;
}
