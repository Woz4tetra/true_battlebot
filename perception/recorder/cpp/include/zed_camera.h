#pragma once
#include <sl/Camera.hpp>
#include <string>
#include <future>
#include <memory>
#include <memory_resource>

class ZEDCamera
{
public:
    ZEDCamera(sl::InitParameters params = sl::InitParameters());
    ~ZEDCamera();

    bool open();
    void close();
    int getFrameCount() const;
    std::string getLastError() const;
    bool startRecording(const std::string &filename);
    void stopRecording();

private:
    sl::Camera zed_;
    sl::InitParameters init_parameters_;
    int fcount_ = 0;
    std::string last_error_;
    bool is_open_ = false;

    std::atomic<bool> stop_worker_{false};
    std::future<void> worker_future_;
    std::mutex mtx_;

    bool update(sl::Camera *zed);
    void worker(sl::Camera *zed);
};
