cmake_minimum_required(VERSION 3.18)
project(recorder_cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RelWithDebInfo build (required for ZED SDK compatibility)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Find Python using the method nanobind recommends
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

# Download and configure pybind11
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# Find ZED SDK
find_package(ZED 5 REQUIRED)
find_package(CUDA REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

# Link directories
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

# Add your source files here

# Add your source files here
set(SOURCES
    src/zed_camera.cpp
    src/zed_bindings.cpp
)

# Create pybind11 module

pybind11_add_module(recorder_cpp ${SOURCES})

# Link ZED libraries
target_link_libraries(recorder_cpp PRIVATE 
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
)

# Compiler-specific options (optional)
# target_compile_definitions(recorder_cpp PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# Installation - install to the build directory where Python can find it
# Use CMAKE_INSTALL_PREFIX which will be set by the build script
install(TARGETS recorder_cpp DESTINATION .)

# Install the Python stub file for type checking
install(FILES recorder_cpp.pyi DESTINATION .)