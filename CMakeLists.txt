cmake_minimum_required(VERSION 3.0)

project(hybrid_a_star)

set(CMAKE_CXX_COMPILER "g++")

set (CMKAE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-g")


find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(osqp REQUIRED)


include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${OSQP_INCLUDE_DIRS}
        src/
        lib/include/
)

set(PROTO_META_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/proto)

file(GLOB proto_files ${CMAKE_CURRENT_SOURCE_DIR}/src/proto/*.proto)

foreach(FIL ${proto_files})
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
    message("file_name: " ${FILE_NAME})
    set(PROTO_FILE ${FILE_NAME}.proto)
    execute_process(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I=${PROTO_META_BASE_DIR} --cpp_out=./ ${PROTO_FILE}
    )
    message("Copying " ${FILE_NAME})
    file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${FILE_NAME}.pb.h DESTINATION ${PROTO_META_BASE_DIR})
    file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/${FILE_NAME}.pb.h)
    file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${FILE_NAME}.pb.cc DESTINATION ${PROTO_META_BASE_DIR})
    file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/${FILE_NAME}.pb.cc)
endforeach()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

file(GLOB_RECURSE cc_files src/*.cc)

add_executable(${PROJECT_NAME}  
demo.cc
${cc_files}
)

target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES} ${PROTOBUF_LIBRARIES} glog::glog osqp::osqp 
${CMAKE_CURRENT_SOURCE_DIR}/lib/librs_static.a)

