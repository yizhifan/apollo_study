cmake_minimum_required(VERSION 3.19)
project(apollo_study)

set(CMAKE_CXX_STANDARD 14)

add_executable(apollo_study
        main.cpp)

target_link_libraries(main ${PROTOBUF_LIBRARIES} -lgtest -lpthread -lglog gflags -labsl -lboost_system osqp::osqp)