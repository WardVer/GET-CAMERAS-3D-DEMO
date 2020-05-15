#include "cameras.hpp"

int Stereocamera::initCameras()
{
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        std::cout << "init library failed" << std::endl;
        exit;
    }
}

int Stereocamera::updateDeviceList()
{

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status != GX_STATUS_SUCCESS || (nDeviceNum <= 0))
    {
        std::cout << "updating device list failed, are the cameras plugged in correctly or is another program using the cameras?" << std::endl;
        exit;
    }
}

int Stereocamera::openCamera(int camera_ID)
{
    status = GXOpenDeviceByIndex(camera_ID, &hDevice);
    if (status != GX_STATUS_SUCCESS)
    {
        std::cout << "opening cameras failed, are the cameras plugged in correctly or is another program using the cameras?" << std::endl;
        exit;
    }
}

int Stereocamera::initCameraSettings(float gain, float exposure, float red_ratio, float green_ratio, float blue_ratio, float framerate, int64_t triggermode)
{
    GXSetFloat(hDevice, GX_FLOAT_GAIN, gain);
    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, red_ratio);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, green_ratio);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, blue_ratio);
    GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, framerate);
    GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, triggermode);
    GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
}

int Stereocamera::setCallback(GXCaptureCallBack callback)
{
    GXRegisterCaptureCallback(hDevice, NULL, callback);
}


int Stereocamera::startCamera()
{
    status = GXStreamOn(hDevice);
    if(status != GX_STATUS_SUCCESS)
    {
        std::cout << "starting cameras failed" << std::endl;
        exit;
    }
}

int Stereocamera::trigger()
{
    GXSendCommand(hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
}

int Stereocamera::setExposure(float exposure)
{
    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
}

