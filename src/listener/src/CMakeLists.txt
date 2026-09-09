add_subdirectory(rp_stuff)

add_executable(listener_node listener_node.cpp)
target_link_libraries(listener_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rp_library
)