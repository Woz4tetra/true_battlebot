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

bool ZEDCamera::open()
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
    worker_future_ = std::async(std::launch::async, &ZEDCamera::worker, this);
    return true;
}

bool ZEDCamera::update(sl::Camera &zed)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto res = zed.grab();

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

    // Open camera in worker thread
    auto returned_state = zed.open(init_parameters_);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_error_ = sl::toString(returned_state);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        is_open_ = true;
    }

    while (!stop_worker_.load(std::memory_order_acquire))
    {
        // Process commands from queue
        std::unique_lock<std::mutex> queue_lock(queue_mtx_);
        queue_cv_.wait_for(queue_lock, std::chrono::milliseconds(10),
                           [this]
                           { return !command_queue_.empty() || stop_worker_.load(); });

        while (!command_queue_.empty())
        {
            Command cmd = command_queue_.front();
            command_queue_.pop();
            queue_lock.unlock();

            processCommand(cmd, zed);

            queue_lock.lock();
        }
        queue_lock.unlock();

        // Continue grabbing frames if camera is open
        bool camera_open;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            camera_open = is_open_;
        }

        if (camera_open)
        {
            if (!update(zed))
            {
                std::cerr << "Error occurred during camera update: " << getLastError() << std::endl;

                // Try to reopen camera
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    is_open_ = false;
                }

                zed.close();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // Attempt to reopen
                returned_state = zed.open(init_parameters_);
                if (returned_state == sl::ERROR_CODE::SUCCESS)
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    is_open_ = true;
                }
                else
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    last_error_ = sl::toString(returned_state);
                }
            }
        }
        else
        {
            // If camera is not open, wait a bit before checking again
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Clean shutdown
    zed.close();
    {
        std::lock_guard<std::mutex> lock(mtx_);
        is_open_ = false;
    }
}

void ZEDCamera::processCommand(const Command &cmd, sl::Camera &zed)
{
    switch (cmd.type)
    {
    case CommandType::START_RECORDING:
        startRecordingInternal(zed, cmd.filename);
        break;
    case CommandType::STOP_RECORDING:
        stopRecordingInternal(zed);
        break;
    }
}

bool ZEDCamera::startRecording(const std::string &filename)
{
    std::lock_guard<std::mutex> queue_lock(queue_mtx_);
    command_queue_.emplace(CommandType::START_RECORDING, filename);
    queue_cv_.notify_one();
    return true;
}

void ZEDCamera::stopRecording()
{
    std::lock_guard<std::mutex> queue_lock(queue_mtx_);
    command_queue_.emplace(CommandType::STOP_RECORDING);
    queue_cv_.notify_one();
}

bool ZEDCamera::startRecordingInternal(sl::Camera &zed, const std::string &filename)
{
    sl::RecordingParameters record_params(filename.c_str());

    auto res = zed.enableRecording(record_params);

    if (res != sl::ERROR_CODE::SUCCESS)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_error_ = sl::toString(res);
        return false;
    }

    return true;
}

void ZEDCamera::stopRecordingInternal(sl::Camera &zed)
{
    zed.disableRecording();
}

void ZEDCamera::close()
{
    // Signal worker to stop
    stop_worker_.store(true, std::memory_order_release);

    // Wake up the worker thread
    {
        std::lock_guard<std::mutex> queue_lock(queue_mtx_);
        queue_cv_.notify_all();
    }

    if (worker_future_.valid())
    {
        try
        {
            worker_future_.get();
        }
        catch (...)
        {
            // Don't throw exceptions during shutdown
        }
    }

    // Clear any remaining commands
    {
        std::lock_guard<std::mutex> queue_lock(queue_mtx_);
        while (!command_queue_.empty())
        {
            command_queue_.pop();
        }
    }
}

int ZEDCamera::getFrameCount() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return fcount_;
}

std::string ZEDCamera::getLastError() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_error_;
}
