project(octirobot)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)


#寻找OpenCV库
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(apriltag REQUIRED)
include_directories(${apriltag_INCLUDE_DIRS})

find_package(TGUI REQUIRED)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2 /home/elf/Desktop/work_station/OctiRobotVersion2/ /usr/local/include/apriltag /usr/local/lib)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread sfml-graphics sfml-window sfml-system yaml-cpp apriltag TGUI::TGUI ${OpenCV_LIBS})

# 将当前目录下的所有源文件添加到变量 SRC_LIST 中
file(GLOB SRC_LIST "*.cpp")


set(SRC_PATH path_control/src)
set(LIB_SRC ${SRC_PATH}/localPlanner/local_planner.cpp ${SRC_PATH}/dwa/dwa.cpp ${SRC_PATH}/vision/vision_device.cpp ${SRC_PATH}/lad/laddar_device.cpp ${SRC_PATH}/charge/octi_charge.cpp ${SRC_PATH}/video/video_device.cpp ${SRC_PATH}/octirobot/octi_robot.cpp ${SRC_PATH}/tinyxml/tinyxml2.cpp  ${SRC_PATH}/robotShowUi/ui.cpp ${SRC_PATH}/yaml/octi_yaml.cpp ${SRC_PATH}/xml/octi_xml.cpp) # ${SRC_PATH}/xml/xml_rw.cpp
add_library(mySharedLib SHARED ${LIB_SRC})


# 
add_executable(main path_control/target/main.cpp)
# add_executable(yamltest path_control/target/yaml_test.cpp)

add_executable(deviceTest path_control/target/device_test.cpp)

add_executable(pathRecord path_control/target/path_record.cpp)

add_executable(robotTest path_control/target/robot_test.cpp)

add_executable(videoTest path_control/target/video_test.cpp)

add_executable(chargeTest path_control/target/charge_test.cpp)

add_executable(visionTest path_control/target/vision_test.cpp)

add_executable(myUiTest path_control/target/test.cpp)

add_executable(gotest path_control/target/gotest.cpp)

# TARGET_LINK_LIBRARIES(charge_test ${OpenCV_LIBS} ${apriltag_LIBS})

TARGET_LINK_LIBRARIES(main mySharedLib)

# TARGET_LINK_LIBRARIES(yamltest mySharedLib)

# TARGET_LINK_LIBRARIES(pathRecord mySharedLib)
TARGET_LINK_LIBRARIES(deviceTest mySharedLib)
TARGET_LINK_LIBRARIES(pathRecord mySharedLib)
TARGET_LINK_LIBRARIES(robotTest mySharedLib)
TARGET_LINK_LIBRARIES(videoTest mySharedLib)
TARGET_LINK_LIBRARIES(chargeTest mySharedLib)
TARGET_LINK_LIBRARIES(visionTest mySharedLib)
TARGET_LINK_LIBRARIES(myUiTest mySharedLib)
TARGET_LINK_LIBRARIES(gotest mySharedLib)