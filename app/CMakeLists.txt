add_executable(trio-play main.cpp)
target_link_libraries(trio-play
  PRIVATE
  trio-lib
  ${OpenCV_LIBS}
)
