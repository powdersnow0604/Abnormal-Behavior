# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  powdersnow
  VERSION 0.1
  DESCRIPTION "test project"
  LANGUAGES CXX)

# 라이브러리 파일은 빌드 디렉토리 안에 lib 폴더에 출력.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 실행 파일은 빌드 디렉토리 안에 bin 폴더에 출력.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(app)
add_subdirectory(lib)