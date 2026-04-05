cmake_minimum_required(VERSION 3.4.1)

project(rknn_deploy_demo)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")


# GENERAL LIB
# rknn api
if(TARGET_SOC STREQUAL "rk356x")
  set(PLATFORM RK3566_3568)
  if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    set(LIB_ARCH arm64-v8a)
    set(RKNN_API_PATH ${RKNPU2}/runtime/RK356X/Android/librknn_api)
    set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
  else()
    set(LIB_ARCH aarch64)
    set(RKNN_API_PATH ${RKNPU2}/runtime/RK356X/Linux/librknn_api)
    set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
  endif()
elseif(TARGET_SOC STREQUAL "rk3588")
  set(PLATFORM RK3588)
  add_definitions(-DPLATFORM_RK3588)
  if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    set(LIB_ARCH arm64-v8a)
    set(RKNN_API_PATH ${RKNPU2}/runtime/RK3588/Android/librknn_api)
    set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
  else()
    set(LIB_ARCH aarch64)
    set(RKNN_API_PATH ${RKNPU2}/runtime/RK3588/Linux/librknn_api)
    set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
  endif()
elseif(TARGET_SOC STREQUAL "rv110x")
  set(PLATFORM RV1106_1103)
  add_definitions(-DPLATFORM_RV1106_1103)
  set(LIB_ARCH armhf)
  set(RKNN_API_PATH ${RKNPU2}/runtime/RV1106/Linux/librknn_api)
  set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnmrt.so)
else()
  message(FATAL_ERROR "TARGET_SOC is not set, ref value: rk356x or rk3588 ")
endif()
include_directories(${RKNN_API_PATH}/include)

# RKDEMO UTILS
set(RKDEMO_UTILS ${CMAKE_SOURCE_DIR}/libs/rkdemo_utils)
set(RKDEMO_UTILS_CC ${RKDEMO_UTILS}/rknn_demo_utils.cc)
include_directories( ${RKDEMO_UTILS})

# RESIZE FUNC
set(RESIZE_FUNC ${CMAKE_SOURCE_DIR}/libs/resize_func)
set(RESIZE_FUNC_CC ${RESIZE_FUNC}/resize_function.cc)
include_directories( ${RESIZE_FUNC})

# cnpy
set(CNPY ${CMAKE_SOURCE_DIR}/libs/cnpy)
set(CNPY_CPP ${CNPY}/cnpy.cpp)
include_directories( ${CNPY})

# third party - stb
set(RKNPU2_THIRD_PARTY ${RKNPU2}/examples/3rdparty)
set(STB ${RKNPU2_THIRD_PARTY}/stb)
include_directories( ${STB})

# others
include_directories( ${CMAKE_SOURCE_DIR}/libs)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # set libz
  set(LIBZ_LIB ${RKNPU2}/examples/3rdparty/opencv/opencv-linux-${LIB_ARCH}/share/OpenCV/3rdparty/lib/libzlib.a)
endif()


if (NOT (TARGET_SOC STREQUAL "rv110x"))
  add_executable(rknn_deploy_demo
            src/main.cc
            ${RKDEMO_UTILS_CC}
            ${RESIZE_FUNC_CC}
            ${CNPY_CPP}
          )
  
  target_link_libraries(rknn_deploy_demo
        ${RKNN_RT_LIB}
        ${LIBZ_LIB}
        dl
    )
endif()


# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_deploy_demo_${CMAKE_SYSTEM_NAME})
if (NOT (TARGET_SOC STREQUAL "rv110x"))
  install(TARGETS rknn_deploy_demo DESTINATION ./)
endif()

install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/run.sh DESTINATION ./)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data DESTINATION ./)
