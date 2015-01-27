cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PXCGRABBER)
find_package(PCL 1.2 REQUIRED)

set(SK_INCLUDE_DIR "/opt/softkinetic/DepthSenseSDK/include" CACHE PATH "path of soft kinetic")

include_directories(${PCL_INCLUDE_DIRS} ${SK_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} /opt/softkinetic/DepthSenseSDK/lib)
add_definitions(${PCL_DEFINITIONS})
add_executable(pxc_test pxc_viewer.cpp softkinetic_grabber.cpp softkinetic_image.cpp softkinetic_device.cpp)
target_link_libraries(pxc_test  ${PCL_LIBRARIES} DepthSense)
