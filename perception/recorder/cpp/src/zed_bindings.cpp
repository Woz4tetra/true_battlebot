#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "zed_camera.h"

namespace py = pybind11;

PYBIND11_MODULE(recorder_cpp, m)
{
    m.doc() = "Python bindings for ZEDCamera using pybind11";

    py::class_<ZEDCamera>(m, "ZEDCamera")
        .def(py::init<>())
        .def(py::init<const sl::InitParameters &>())
        .def("open", &ZEDCamera::open)
        .def("start_recording", &ZEDCamera::startRecording)
        .def("stop_recording", &ZEDCamera::stopRecording)
        .def("close", &ZEDCamera::close)
        .def("get_frame_count", &ZEDCamera::getFrameCount)
        .def("get_last_error", &ZEDCamera::getLastError);

    py::class_<sl::InitParameters>(m, "InitParameters")
        .def(py::init<>())
        .def_readwrite("camera_resolution", &sl::InitParameters::camera_resolution)
        .def_readwrite("depth_mode", &sl::InitParameters::depth_mode)
        .def_readwrite("sdk_verbose", &sl::InitParameters::sdk_verbose)
        .def_readwrite("camera_fps", &sl::InitParameters::camera_fps);

    py::enum_<sl::RESOLUTION>(m, "RESOLUTION")
        .value("HD4K", sl::RESOLUTION::HD4K)
        .value("QHDPLUS", sl::RESOLUTION::QHDPLUS)
        .value("HD2K", sl::RESOLUTION::HD2K)
        .value("HD1536", sl::RESOLUTION::HD1536)
        .value("HD1080", sl::RESOLUTION::HD1080)
        .value("HD720", sl::RESOLUTION::HD720)
        .value("SVGA", sl::RESOLUTION::SVGA)
        .value("VGA", sl::RESOLUTION::VGA)
        .value("AUTO", sl::RESOLUTION::AUTO)
        .export_values();

    py::enum_<sl::DEPTH_MODE>(m, "DEPTH_MODE")
        .value("NONE", sl::DEPTH_MODE::NONE)
        .value("PERFORMANCE", sl::DEPTH_MODE::PERFORMANCE)
        .value("QUALITY", sl::DEPTH_MODE::QUALITY)
        .value("ULTRA", sl::DEPTH_MODE::ULTRA)
        .value("NEURAL_LIGHT", sl::DEPTH_MODE::NEURAL_LIGHT)
        .value("NEURAL", sl::DEPTH_MODE::NEURAL)
        .value("NEURAL_PLUS", sl::DEPTH_MODE::NEURAL_PLUS)
        .export_values();

    m.attr("__version__") = "0.0.0";
}
