cmake_minimum_required(VERSION 3.15)
project(DPApp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일러 옵션
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 인클루드 디렉토리
include_directories(include)
include_directories(src)

# 소스 파일들
file(GLOB_RECURSE COMMON_SOURCES "src/common/*.cpp" "src/network/*.cpp" "src/pointcloud/*.cpp")
file(GLOB_RECURSE MASTER_SOURCES "src/master/*.cpp")
file(GLOB_RECURSE SLAVE_SOURCES "src/slave/*.cpp")

# 공통 라이브러리
add_library(dpapp_common STATIC ${COMMON_SOURCES})

# Master 실행 파일
add_executable(master ${MASTER_SOURCES})
target_link_libraries(master dpapp_common)

# Slave 실행 파일
add_executable(slave ${SLAVE_SOURCES})
target_link_libraries(slave dpapp_common)

# 플랫폼별 네트워크 라이브러리
if(WIN32)
    target_link_libraries(master ws2_32 wsock32)
    target_link_libraries(slave ws2_32 wsock32)
endif()

if(UNIX)
    target_link_libraries(master pthread)
    target_link_libraries(slave pthread)
endif()