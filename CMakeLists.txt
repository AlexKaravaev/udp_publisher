cmake_minimum_required(VERSION 2.8.3)
project(raw_udp_receiver)
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    std_msgs)


find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
               INCLUDE_DIRS include
               LIBRARIES input)

add_subdirectory(src/lib)

add_executable(udp_receiver_node src/udp_receiver_node.cc)
target_link_libraries(udp_receiver_node ${catkin_LIBRARIES})

add_executable(udp_topic_node src/udp_topic_node.cc)
target_link_libraries(udp_topic_node ${catkin_LIBRARIES})

# Compile all
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(PROGRAMS src/vdump
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
