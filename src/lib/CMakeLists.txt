add_library(input udp_client.cc)
target_link_libraries(input
    ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(input ${catkin_EXPORTED_TARGETS})
endif()

install (TARGETS input
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

    )
