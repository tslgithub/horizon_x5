project(keypoints_match)

add_definitions(-w)

message(STATUS "keypoints_match -> " ${ROOT_DIR}/keypoints_match)

find_package(OpenCV REQUIRED)

set(SRC ${ROOT_DIR}/keypoints_match/src)
set(UTILS_SRC ${ROOT_DIR}/utils_common/)

include_directories(
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include
        ${OpenCV_INCLUDE_DIRS}
        ${MAIN_PROJECT_3TH_DIR}
        ${SRC}
        ${UTILS_SRC}
)

message(STATUS "hb_dnn -> " ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include)

add_executable(keypoints_match
        ${SRC}/main.cpp
        ${SRC}/keypoints_match.cpp
        ${UTILS_SRC}/utils_rgbd.cpp

)

target_link_libraries(keypoints_match
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/lib/libdnn.so
        ${OpenCV_LIBS}
        ${MAIN_PROJECT_3TH_DIR}/lib/libalog.so.1.0.1
        ${MAIN_PROJECT_3TH_DIR}/lib/libcnn_intf.so.1.3.6
        ${MAIN_PROJECT_3TH_DIR}/lib/libhbmem.so.1.0.0
        ${MAIN_PROJECT_3TH_DIR}/lib/libhbrt_bayes_aarch64.so
#        ${MAIN_PROJECT_3TH_DIR}/lib/libopencv_world.so
)