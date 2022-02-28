cmake_minimum_required(VERSION 3.0.2)
project(extract_RT)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs roscpp)

find_package(OpenCV 3.4.17 REQUIRED)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
