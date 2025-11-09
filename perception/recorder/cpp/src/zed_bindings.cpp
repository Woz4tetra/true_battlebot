#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "zed_camera.h"

namespace py = pybind11;

PYBIND11_MODULE(recorder_cpp, m)
{
    m.doc() = "Python bindings for ZEDCamera using pybind11";

    py::class_<ZEDCamera>(m, "ZEDCamera")
        .def(py::init<>())
        .def("open", &ZEDCamera::open)
        .def("enable_streaming", &ZEDCamera::enableStreaming, py::arg("port") = 30000)
        .def("grab", &ZEDCamera::grab)
        .def("disable_streaming", &ZEDCamera::disableStreaming)
        .def("close", &ZEDCamera::close)
        .def("get_frame_count", &ZEDCamera::getFrameCount)
        .def("get_last_error", &ZEDCamera::getLastError)
        .def("retrieve_image", &ZEDCamera::retrieveImage);

    m.attr("__version__") = "0.0.0";
}
