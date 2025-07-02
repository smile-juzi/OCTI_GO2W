cmake_minimum_required(VERSION 3.10)
project(httpserver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found! Install with: sudo apt install libopencv-dev")
endif()

# 添加可执行文件
add_executable(httpserver http_server.cpp)

# 添加包含目录（包含 OpenCV 和自定义头文件）
target_include_directories(httpserver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # 针对 Ubuntu 系统的 OpenCV 4 头文件路径
)

# 链接所有依赖库
target_link_libraries(httpserver PRIVATE
    ${OpenCV_LIBS}
    #pthread
    # 如果需要使用 cpp-httplib 的 CPPHTTPLIB_OPENSSL_SUPPORT 特性
    #ssl crypto
)

# 针对单文件头库
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h")
    target_include_directories(httpserver PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
else()
    message(WARNING "httplib.h not found in include directory")
endif()
