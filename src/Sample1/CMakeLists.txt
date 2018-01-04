
find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)

add_executable(DepthEyeH1Sample DepthEyeH1Sample.cpp)
target_link_libraries(DepthEyeH1Sample voxel )

install(TARGETS
  DepthEyeH1Sample
  RUNTIME  
  DESTINATION bin
  COMPONENT test
)
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL test)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library test programs.")
ENDIF()
