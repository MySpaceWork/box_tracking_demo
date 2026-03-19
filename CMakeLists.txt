cmake_minimum_required(VERSION 3.10)
project(box_tracking_demo CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# Find OpenCV -- the primary dependency.
# ===========================================================================
find_package(OpenCV REQUIRED COMPONENTS core imgproc video highgui videoio calib3d)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs:    ${OpenCV_LIBS}")

# ===========================================================================
# Stage 3: Optional Eigen3 support for similarity transform.
# ===========================================================================
option(USE_EIGEN3 "Use Eigen3 for efficient similarity transform" ON)

if(USE_EIGEN3)
  find_package(Eigen3 QUIET)
  if(EIGEN3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
    message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
    add_definitions(-DUSE_EIGEN3)
    set(EIGEN3_ENABLED TRUE)
  else()
    message(STATUS "Eigen3 not found, using OpenCV for matrix operations")
    set(EIGEN3_ENABLED FALSE)
  endif()
else()
  message(STATUS "Eigen3 support disabled (USE_EIGEN3=OFF)")
  set(EIGEN3_ENABLED FALSE)
endif()

# ===========================================================================
# Tracking library (standalone, no mediapipe dependency)
# ===========================================================================
add_library(box_tracker_lib STATIC
  src/box_tracker.cc
)

target_include_directories(box_tracker_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

if(EIGEN3_ENABLED)
  target_include_directories(box_tracker_lib PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(box_tracker_lib PUBLIC
  ${OpenCV_LIBS}
)

# ===========================================================================
# Demo executable
# ===========================================================================
add_executable(box_tracking_demo
  src/main.cc
)

target_include_directories(box_tracking_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(box_tracking_demo PRIVATE
  box_tracker_lib
  ${OpenCV_LIBS}
)
