cmake_minimum_required(VERSION 3.13)

#set(CMAKE_C_COMPILER   /home/ubuntu/Downloads/toybrick-toolchains/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
#set(CMAKE_C++_COMPILER /home/ubuntu/Downloads/toybrick-toolchains/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)

#set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc-8)
#set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-8)

#set(CMAKE_C_COMPILER /opt/x5/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
#set(CMAKE_XXX_COMPILER /opt/x5/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

project(T-robot)

set(CMAKE_BUILD_TYPE Release)


set(BUDILD_FOR_HORIZON_X5 on)
set(BUILD_TARGET BUDILD_x5)
add_definitions( -DBUILD_PLATFORM=${BUILD_TARGET})
message(STATUS "BUILD_TARGET -> " ${BUILD_TARGET})

set(ROOT_DIR                ${CMAKE_CURRENT_SOURCE_DIR})
set(MAIN_PROJECT_TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET})
set(MAIN_PROJECT_SRC_DIR    ${CMAKE_CURRENT_SOURCE_DIR})
set(MAIN_PROJECT_3TH_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/thridPart)


set(EXE_OUT_DIR     ${ROOT_DIR}/output/${BUILD_TARGET}/bin)
set(PACKET_OUT_DIR  ${ROOT_DIR}/output/${BUILD_TARGET}/out/)

set(EXECUTABLE_OUTPUT_DIR ${EXE_OUT_DIR}/)

#message(STATUS "toolchain: " ${ROOT_DIR}/${MAIN_PROJECT_3TH_DIR}/compiler/${BUILD_TARGET}/toolchain.cmake)
message(STATUS "build_for_target: " ${BUILD_TARGET})
message(STATUS "root and out dir: " ${ROOT_DIR} ${EXE_OUT_DIR})

#link_directories(${MAIN_PROJECT_3TH_DIR}/boost/${BUILD_TARGET}/lib/libboost_json.so.1.79.1.0
#                 ${MAIN_PROJECT_3TH_DIR}/boost/${BUILD_TARGET}/lib/libboost_serialization.so.1.79.1.0
#                 ${MAIN_PROJECT_3TH_DIR}/boost/${BUILD_TARGET}/lib/libboost_container.so.1.79.1.0
#        )

#include_directories(${ROOT_DIR}/include/)

# 设置并查找线程库，只需要在顶层的 CMakeLists.txt 中添加
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include(${ROOT_DIR}/stereo_depth/stereo_depth.cmake)
include(${ROOT_DIR}/keypoints_match/keypoints_match.cmake)
include(${ROOT_DIR}/horizon_x5/model.cmake)
#include(${ROOT_DIR}/object_detect/src/object_detect.cmake)
#include(${ROOT_DIR}/deepLearning/src/deepLearning.cmake)
#include(${ROOT_DIR}/StatesManager/src/states_manager.cmake)
