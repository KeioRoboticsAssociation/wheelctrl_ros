set(LIBRARY_NAME converter_4ws)

add_library(${LIBRARY_NAME}
  SHARED
  cmdvel24ws.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/converter
  PUBLIC ${PROJECT_SOURCE_DIR}/src/processings
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} processings_4ws)
