cmake_minimum_required(VERSION 3.3)
project(direct_odometry)

set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -pthread")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# optional libraries

list(APPEND CMAKE_PREFIX_PATH "/volume/software/common/cmake/$ENV{ARCH}")

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Eigen REQUIRED)
message(WHAT " "  ${Eigen_INCLUDE_DIRS})
include_directories( ${Eigen_INCLUDE_DIRS} )

#find_package(glog REQUIRED)
#set(glog_ROOT /volume/software/common/foreign_packages/glog/latest/osl42-x86_64)
#set(glog_INCLUDE_DIRS ${glog_ROOT}/include)
#include_directories(${glog_INCLUDE_DIRS})
#set(glog_LIBRARY_DIR ${glog_ROOT}/lib64)
#LINK_DIRECTORIES(${glog_LIBRARY_DIR})

#find_package(ceres REQUIRED)
#set(ceres_ROOT /volume/software/common/foreign_packages/ceres/latest)
#set(ceres_INCLUDE_DIRS ${ceres_ROOT}/include;${ceres_ROOT}/config)
#include_directories(${ceres_INCLUDE_DIRS})
#set(ceres_LIBRARY_DIR ${ceres_ROOT}/lib/osl42-x86_64)
#LINK_DIRECTORIES(${ceres_LIBRARY_DIR})

add_executable(direct_odometry
               main.cpp

        utils/definesAndIncludes.h
        utils/SystemSettings.h
        utils/SystemSettings.cpp
        utils/data_structures.h
        utils/common_functions.h
        utils/common_functions.cpp

        classes/camera.h
        classes/camera.cpp
        classes/camPinhole.cpp
        classes/camPinholeDist.cpp

        classes/frame.h
        classes/frame.cpp
        classes/extractHGP.cpp

        classes/trackingFrontEnd.h
        classes/trackingFrontEnd.cpp
        classes/tracking_functions.cpp

        classes/backEnd.cpp
        classes/backEnd.h
        classes/dvo.h
        classes/dvo.cpp

        test_functions.h
        test_functions.cpp

        utils/SequenceSettings.cpp
        utils/SequenceSettings.h

        utils/optimizer.cpp
        utils/optimizer.h)

#set(glog_LIBS glog)
#set(ceres_LIBS ceres;gomp;lapack;blas)

target_link_libraries(direct_odometry ${OpenCV_LIBS})
target_link_libraries(direct_odometry ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/libg2o.so)
#target_link_libraries(direct_odometry ${Eigen_LIBS})

#target_link_libraries(direct_odometry ${ceres_LIBS})
#target_link_libraries(direct_odometry ${glog_LIBS})
