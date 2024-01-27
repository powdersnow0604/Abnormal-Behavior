# 현재 디렉토리에 있는 모든 파일을 SRC_FILES 변수에 추가한다.
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_library(total_ot STATIC ${SRC_FILES})

option(DEBUG "using -g flag" OFF)

# total_ot 의 include 경로 지정
target_include_directories(total_ot PUBLIC ${CMAKE_SOURCE_DIR}/include /usr/local/include/opencv4 
${CMAKE_SOURCE_DIR}/../library/include)

# total_ot 의 컴파일 옵션 지정
if(DEBUG)
  target_compile_options(total_ot PUBLIC -Wall -Werror -g)
else()
  target_compile_options(total_ot PUBLIC -Wall -Werror)
endif()

# total_ot 를 C++ 11 로 컴파일
target_compile_features(total_ot PRIVATE cxx_std_11)

#OpenBLAS
# total_ot 의 include 경로 지정
#target_include_directories(total_ot PUBLIC /home/powdersnow/workspace/C_derived/library/OpenBLAS/include)
# total_ot 에 링크
#target_link_libraries(total_ot PRIVATE /home/powdersnow/workspace/C_derived/library/OpenBLAS/lib/libopenblas.so)

#opencv
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])
find_package(OpenCV REQUIRED)
target_link_libraries(total_ot ${OpenCV_LIBRARIES})

#errctl
find_library(ERRCTL NAMES liberrctl.so PATHS /home/powdersnow/workspace/C_derived/library/lib REQUIRED)
target_link_libraries(total_ot ${ERRCTL})