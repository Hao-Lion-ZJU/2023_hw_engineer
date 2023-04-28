cmake_minimum_required(VERSION 3.0.0)
project(task_1 VERSION 0.1.0)

include(CTest)
enable_testing()

#  C++14
set(CMAKE_CXX_STANDARD 14)

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

#子目录
set ( DIR_INCS
	  ./alogrithm
      ./Match
      ./device
      ./pose
      ./device
      )

include_directories ($(DIR_INCS))


aux_source_directory (./ main_src)
aux_source_directory (./alogrithm alogrithm_src)
aux_source_directory (./Match match_src)
aux_source_directory (./pose pose_src)

# file(GLOB device_src device/*.cpp)
# include_directories(device device/MVCamera/include device/serial/include)

file(GLOB utils_src utils/*.cpp)
include_directories(utils)

file(GLOB device_src device/*.cpp)
include_directories(device device/MVCamera/include device/serial/include)

add_executable(task_1  ${main_src}  ${match_src} ${utils_src} ${pose_src} ${device_src} ${alogrithm_src})#

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    #显示OpenCV_INCLUDE_DIRS的值
    message(${OpenCV_INCLUDE_DIRS})
    #添加OpenCV头文件
    target_include_directories(task_1 PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(task_1 ${OpenCV_LIBS}) 
endif()

# GLOG
find_package (glog REQUIRED)
target_link_libraries (task_1 glog::glog)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


# static local library
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    target_link_libraries(task_1 
        ${PROJECT_SOURCE_DIR}/device/MVCamera/lib/arm64/libMVSDK.so 
        ${PROJECT_SOURCE_DIR}/device/serial/lib/arm64/libserial.a)
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    target_link_libraries(task_1 
        ${PROJECT_SOURCE_DIR}/device/MVCamera/lib/x64/libMVSDK.so 
        ${PROJECT_SOURCE_DIR}/device/serial/lib/x64/libserial.a)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


