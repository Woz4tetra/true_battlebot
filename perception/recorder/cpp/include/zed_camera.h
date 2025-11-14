/**
 * Thread-safe ZED camera wrapper with command queue for recording control
 *
 * Usage example:
 *   ZEDCamera camera(init_params);
 *   camera.start();  // Starts worker thread
 *
 *   // Thread-safe commands - can be called from any thread
 *   camera.startRecording("output.svo");
 *   // ... do other work ...
 *   camera.stopRecording();
 *
 *   camera.stop();   // Stops worker thread and closes camera
 */
#pragma once
#include <sl/Camera.hpp>
#include <string>
#include <future>
#include <memory>
#include <memory_resource>
#include <queue>
#include <condition_variable>

enum class CommandType
{
    START_RECORDING,
    STOP_RECORDING
};

struct Command
{
    CommandType type;
    std::string filename; // Only used for START_RECORDING

    Command(CommandType t) : type(t) {}
    Command(CommandType t, const std::string &f) : type(t), filename(f) {}
};

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
    bool is_recording_ = false;

    std::atomic<bool> stop_worker_{false};
    std::future<void> worker_future_;
    mutable std::mutex mtx_;

    // Command queue for thread-safe communication
    std::queue<Command> command_queue_;
    std::mutex queue_mtx_;
    std::condition_variable queue_cv_;

    bool update(sl::Camera &zed);
    void worker();
    void processCommand(const Command &cmd, sl::Camera &zed);
    bool startRecordingInternal(sl::Camera &zed, const std::string &filename);
    void stopRecordingInternal(sl::Camera &zed);
};
