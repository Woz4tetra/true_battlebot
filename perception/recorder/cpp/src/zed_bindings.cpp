#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "zed_camera.h"

namespace nb = nanobind;

NB_MODULE(recorder_cpp, m)
{
    m.doc() = "Python bindings for ZEDCamera using nanobind";

    nb::class_<ZEDCamera>(m, "ZEDCamera")
        .def(nb::init<>())
        .def("open", &ZEDCamera::open)
        .def("enable_streaming", &ZEDCamera::enableStreaming, nb::arg("port") = 30000)
        .def("grab", &ZEDCamera::grab)
        .def("disable_streaming", &ZEDCamera::disableStreaming)
        .def("close", &ZEDCamera::close)
        .def("get_frame_count", &ZEDCamera::getFrameCount)
        .def("get_last_error", &ZEDCamera::getLastError);

    m.attr("__version__") = "0.0.0";
}
