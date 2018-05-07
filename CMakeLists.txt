cmake_minimum_required(VERSION 2.8)

project( CarControl-0.3 )

find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE})
link_directories ( ${OPENNI2_REDIST})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
               STRINGS "Debug" "Release")
endif()
IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL
    ${CMAKE_CURRENT_SOURCE_DIR}/control/include
)

link_directories( ${LIB_FREENECT_INSTALL_DIR}/lib
    ${LIBRARY_OUTPUT_PATH}
)

find_package(OpenMP)
if(OPENMP_FOUND)
    message("Find OpenMP: ${OpenMP_C_FLAGS}, ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(peripheral_driver)
add_subdirectory(HAL)

set( vision_exe vision-test)
set( vision_src 
    main_pid/vision_test.cpp
    control/src/OpenNIHelper.cpp
)
add_executable( ${vision_exe} ${vision_src})
target_link_libraries( ${vision_exe} OpenNI2 i2c-pwm HAL ${OpenCV_LIBS})
