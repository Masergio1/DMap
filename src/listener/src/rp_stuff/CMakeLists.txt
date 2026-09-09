
add_library(rp_library SHARED
  grid_map.cpp
  draw_helpers.cpp
  laser_scan.cpp
  world_item.cpp
  laser_scanner.cpp
  dmap.cpp
  dmap_localizer.cpp)

target_link_libraries(rp_library
  ${OpenCV_LIBS}
  )

add_executable(grid_main
  grid_main.cpp)

target_link_libraries(grid_main
  ${OpenCV_LIBS}
  rp_library
  )

add_executable(test_grid_map
  test_grid_map.cpp)

target_link_libraries(test_grid_map
  ${OpenCV_LIBS}
  rp_library
  )

add_executable(dmap_test_in_class
  dmap_test.cpp
)

target_link_libraries(dmap_test_in_class
  ${OpenCV_LIBS}
  rp_library
  )

add_executable(dmap_localizer_test
  dmap_localizer_test.cpp)

target_link_libraries(dmap_localizer_test
  ${OpenCV_LIBS}
  rp_library
  )

add_executable(dmap_grid_localizer_test
  dmap_grid_localizer_test.cpp
)

target_link_libraries(dmap_grid_localizer_test
  ${OpenCV_LIBS}
  rp_library
  )

add_executable(rp_simulator
  simulator.cpp)

target_link_libraries(rp_simulator
  ${OpenCV_LIBS}
  rp_library
  )

