# 현재 디렉토리에 있는 모든 파일을 SRC_FILES 변수에 추가한다.
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])
find_package(OpenCV REQUIRED)

add_executable(program ${SRC_FILES})
target_link_libraries(program PUBLIC total_ot ${OpenCV_LIBRARIES})
