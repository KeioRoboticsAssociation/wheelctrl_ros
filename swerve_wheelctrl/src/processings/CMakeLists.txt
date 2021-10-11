set(LIBRARY_NAME processings_4ws)

add_library(${LIBRARY_NAME}
  SHARED
  processings.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/processings
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})
