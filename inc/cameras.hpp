#include "GxIAPI.h"
#include "DxImageProc.h"

#include <string>
#include <stdio.h>
#include <iostream>

class Stereocamera
{
public:
    GX_STATUS status = GX_STATUS_SUCCESS;

    int initCameras();

    int updateDeviceList();

    int openCamera(int camera_ID);

    int initCameraSettings(float gain, float exposure, float red_ratio, float green_ratio, float blue_ratio, float framerate, int64_t triggermode);

    int setCallback(GXCaptureCallBack callback);

    int startCamera();

    int setExposure(float exposure);

    int trigger();

    Stereocamera(int camera_ID, float gain, float exposure, float red_ratio, float green_ratio, float blue_ratio, float framerate, int64_t triggermode)
    {
        initCameras();
        updateDeviceList();
        openCamera(camera_ID);
        initCameraSettings(gain, exposure, red_ratio, green_ratio, blue_ratio, framerate, triggermode);  
    }

private:
    uint32_t nDeviceNum = 0;
    GX_DEV_HANDLE hDevice = NULL;
};
