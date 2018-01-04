cmake_minimum_required (VERSION 2.8.13)
project (depth-eye)

#set(CMAKE_INCLUDE_CURRENT_DIR 1)
message("CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
message("ARCH = ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(CMAKE_CXX_FLAGS 
    "-msse2 -pthread -std=c++11 -fPIC -ffast-math"
  )
  set(COMMON_LIBS
    /usr/local/lib
    usb-1.0
    ./lib
    dl
  )
  set(COMMON_INCLUDE
    /usr/include/libusb-1.0
    ./include/
  )
  set(voxel 
    ./lib
  )

endif()

include_directories(${COMMON_INCLUDE})
link_directories("/usr/local/lib" ${COMMON_LIBS} ${voxel})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

#### Packaging related information -- common to both platforms
set(CPACK_PACKAGE_NAME "Depth Eye SDK")
set(CPACK_PACKAGE_VERSION "0.1")
set(CPACK_PACKAGE_CONTACT "huangjiandong@haloai.com")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

add_subdirectory(src/Sample1)