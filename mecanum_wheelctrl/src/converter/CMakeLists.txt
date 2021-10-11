set(LIBRARY_NAME converter_mecanum)

add_library(${LIBRARY_NAME}
  SHARED
  cmdvel2mecanum.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/converter
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})
