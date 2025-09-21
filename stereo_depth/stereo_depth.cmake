project(stereo_depth)
add_definitions(-w)

message(STATUS "stereo_depth: " ${ROOT_DIR}/stereo_depth)
find_package(OpenCV REQUIRED)

set(SRC ${ROOT_DIR}/stereo_depth/src)
set(UTILS_SRC ${ROOT_DIR}/utils_common/)
set(MODEL_SRC ${ROOT_DIR}/horizon_x5/src)

include_directories(${SRC}
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include
        ${OpenCV_INCLUDE_DIRS}
        ${MODEL_SRC}
        ${SRC}
        ${UTILS_SRC}
)


message(STATUS "hb_dnn -> "  ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include)

add_executable(stereo_depth
        ${SRC}/main.cpp
        ${SRC}/stereo_depth.cpp
        ${UTILS_SRC}/utils_rgbd.cpp
        ${MODEL_SRC}/model.cpp
)

target_link_libraries(stereo_depth
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/lib/libdnn.so
        ${OpenCV_LIBS}
)