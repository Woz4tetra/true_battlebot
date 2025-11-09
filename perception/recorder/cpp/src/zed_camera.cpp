#include "zed_camera.h"
#include <sl/Camera.hpp>
#include <iostream>
#include <thread>
#include <chrono>

ZEDCamera::ZEDCamera(sl::InitParameters params) : init_parameters_(params)
{
}

ZEDCamera::~ZEDCamera()
{
    // Try to safely close resources, but don't fail if SDK is already shutting down
    try
    {
        if (is_open_)
            close();
    }
    catch (const std::exception &e)
    {
        // Print details about the exception
        std::cerr << "Error occurred during ZEDCamera destruction: " << e.what() << std::endl;
        std::cerr << "Last error: " << getLastError() << std::endl;
    }
}

bool ZEDCamera::start()
{
    if (is_open_)
    {
        std::cerr << "Camera is already open" << std::endl;
        return false;
    }
    if (worker_future_.valid())
    {
        std::cerr << "Worker thread is already running" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(mtx_);

    stop_worker_.store(false, std::memory_order_release);
    worker_future_ = std::async(std::launch::async, &ZEDCamera::worker, this, std::move(&zed_));
    return true;
}

bool ZEDCamera::update(sl::Camera zed)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto res = zed->grab();

    if (res <= sl::ERROR_CODE::SUCCESS)
    {
        fcount_++;
        return true;
    }
    last_error_ = sl::toString(res);

    return false;
}

void ZEDCamera::worker()
{
    sl::Camera zed;
    open(zed);

    while (!stop_worker_.load(std::memory_order_acquire))
    {
        if (!update(zed))
        {
            std::cerr << "Error occurred during camera update: " << getLastError() << std::endl;
            close(zed);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // Attempt to reopen
            open(zed);
        }
    }
}

void ZEDCamera::open(sl::Camera zed)
{
    auto returned_state = zed.open(init_parameters_);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        last_error_ = sl::toString(returned_state);
        return false;
    }
    is_open_ = true;
}

void ZEDCamera::close(sl::Camera zed)
{
    zed.close();
}

bool ZEDCamera::startRecording(sl::Camera zed, const std::string &filename)
{
    sl::RecordingParameters record_params(filename.c_str());

    auto res = zed.enableRecording(record_params);

    if (res != sl::ERROR_CODE::SUCCESS)
    {
        last_error_ = sl::toString(res);
        return false;
    }

    return true;
}

void ZEDCamera::stopRecording(sl::Camera zed)
{
    zed.disableRecording();
}

void ZEDCamera::stop()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        is_open_ = false;
    }

    stop_worker_.store(true, std::memory_order_release);
    if (worker_future_.valid())
    {
        try
        {
            worker_future_.get();
        }
        catch (...)
        {
            // Don't throw exceptions from the destructor
        }
    }
}

int ZEDCamera::getFrameCount() const
{
    return fcount_;
}

std::string ZEDCamera::getLastError() const
{
    return last_error_;
}
