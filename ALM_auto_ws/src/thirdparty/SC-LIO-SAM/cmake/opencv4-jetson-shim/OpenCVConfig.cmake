# ALM 미니 OpenCV 설정 (Jetson 전용 셤).
#
# 배경: JetPack 의 libopencv-dev 4.8 은 헤더/CMake 만 있고 라이브러리가 없어
# find_package(OpenCV) 가 실패한다. 실제 존재하는 라이브러리는 Ubuntu(jammy)의
# 런타임 패키지 libopencv-*4.5d (/usr/lib/aarch64-linux-gnu/*.so.4.5d) 뿐이다.
# 이 셤은 4.5d 라이브러리 + 그와 버전이 일치하는 4.5.4 헤더(apt-get download
# libopencv-core-dev 로 추출, ALM_auto_ws/thirdparty/opencv454/extracted)를 묶는다.
# 준비 절차는 docs/JETSON_SETUP.md 참고. lio_sam 은 libopencv_core 만 사용한다.

get_filename_component(_alm_ws "${CMAKE_CURRENT_LIST_DIR}/../../../../.." REALPATH)
set(_alm_cv_inc "${_alm_ws}/thirdparty/opencv454/extracted/usr/include/opencv4")
set(_alm_cv_libdir "/usr/lib/aarch64-linux-gnu")

if(NOT EXISTS "${_alm_cv_inc}/opencv2/core.hpp")
  message(FATAL_ERROR
    "opencv4-jetson-shim: 4.5.4 헤더 없음 (${_alm_cv_inc}). "
    "docs/JETSON_SETUP.md 의 'apt-get download libopencv-core-dev' 절차를 먼저 수행할 것")
endif()

set(OpenCV_VERSION 4.5.4)
set(OpenCV_VERSION_MAJOR 4)
set(OpenCV_VERSION_MINOR 5)
set(OpenCV_VERSION_PATCH 4)
set(OpenCV_INCLUDE_DIRS "${_alm_cv_inc}")

foreach(_m core imgproc imgcodecs)
  if(NOT TARGET opencv_${_m})
    add_library(opencv_${_m} SHARED IMPORTED)
    set_target_properties(opencv_${_m} PROPERTIES
      IMPORTED_LOCATION "${_alm_cv_libdir}/libopencv_${_m}.so.4.5d"
      INTERFACE_INCLUDE_DIRECTORIES "${_alm_cv_inc}")
  endif()
endforeach()

set(OpenCV_LIBS opencv_core)
set(OpenCV_LIBRARIES opencv_core)
set(OpenCV_FOUND TRUE)
set(OPENCV_FOUND TRUE)
