cmake_minimum_required(VERSION 2.8.3)
project(udp_publisher)
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    std_msgs)
    
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

add_executable(udp_publisher_node src/udp_publisher_node.cc src/udp_publisher.cc)
target_link_libraries(udp_publisher_node ${catkin_LIBRARIES})

# Compile all
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS udp_publisher_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      )
