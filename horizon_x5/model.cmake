project(model)

add_definitions(-w)

set(SRC ${ROOT_DIR}/horizon_x5/src)

include_directories(
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include
        ${MAIN_PROJECT_3TH_DIR}
        ${SRC}
)

message(STATUS "hb_dnn -> " ${MAIN_PROJECT_3TH_DIR}/hb_dnn/include)

add_executable(model
    ${SRC}/main.cpp
    ${SRC}/model.cpp
)

target_link_libraries(
        model
        ${MAIN_PROJECT_3TH_DIR}/lib/libalog.so.1.0.1
        ${MAIN_PROJECT_3TH_DIR}/lib/libcnn_intf.so.1.3.6
        ${MAIN_PROJECT_3TH_DIR}/lib/libhbmem.so.1.0.0
#        ${MAIN_PROJECT_3TH_DIR}/lib/libhbrt_bayes_aarch64.so
        ${MAIN_PROJECT_3TH_DIR}/hb_dnn/lib/libdnn.so
)