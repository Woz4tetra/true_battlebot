#pragma once
#include <sl/Camera.hpp>
#include <string>

class ZEDCamera
{
public:
    ZEDCamera();
    ~ZEDCamera();

    bool open();
    bool enableStreaming(int port = 30000);
    bool grab();
    void disableStreaming();
    void close();
    int getFrameCount() const;
    std::string getLastError() const;
    sl::Mat retrieveImage();

private:
    sl::Camera zed;
    sl::InitParameters init_parameters;
    sl::StreamingParameters stream_params;
    int fcount = 0;
    std::string last_error;
    bool is_open = false;
    bool is_streaming = false;
};
