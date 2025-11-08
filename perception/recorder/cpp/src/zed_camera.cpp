#include "zed_camera.h"
#include <sl/Camera.hpp>
#include <iostream>
#include <thread>
#include <chrono>

ZEDCamera::ZEDCamera()
{
    init_parameters.camera_resolution = sl::RESOLUTION::AUTO;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.sdk_verbose = 1;
}

ZEDCamera::~ZEDCamera()
{
    // Try to safely close resources, but don't fail if SDK is already shutting down
    try
    {
        if (is_streaming)
            disableStreaming();
        if (is_open)
            close();
    }
    catch (...)
    {
        // Silently ignore any exceptions during destruction
        // This prevents crashes during Python shutdown
    }
}

bool ZEDCamera::open()
{
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        last_error = sl::toString(returned_state);
        return false;
    }
    is_open = true;
    return true;
}

bool ZEDCamera::enableStreaming(int port)
{
    stream_params.port = port;
    auto returned_state = zed.enableStreaming(stream_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        last_error = sl::toString(returned_state);
        return false;
    }
    is_streaming = true;
    return true;
}

bool ZEDCamera::grab()
{
    auto res = zed.grab();
    if (res <= sl::ERROR_CODE::SUCCESS)
    {
        fcount++;
        return true;
    }
    last_error = sl::toString(res);
    return false;
}

void ZEDCamera::disableStreaming()
{
    zed.disableStreaming();
    is_streaming = false;
}

void ZEDCamera::close()
{
    zed.close();
    is_open = false;
}

int ZEDCamera::getFrameCount() const
{
    return fcount;
}

std::string ZEDCamera::getLastError() const
{
    return last_error;
}
