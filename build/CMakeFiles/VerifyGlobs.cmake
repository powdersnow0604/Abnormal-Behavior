# CMAKE generated file: DO NOT EDIT!
# Generated by CMake Version 3.16

# SRC_FILES at app/CMakeLists.txt:2 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/home/powdersnow/workspace/C_derived/object_tracking/app/*.cpp")
set(OLD_GLOB
  "/home/powdersnow/workspace/C_derived/object_tracking/app/main.cpp"
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles/cmake.verify_globs")
endif()

# SRC_FILES at lib/CMakeLists.txt:2 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/home/powdersnow/workspace/C_derived/object_tracking/lib/*.cpp")
set(OLD_GLOB
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/bool_vector.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/customer.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/detect.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/hungarian.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/iou.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/kalman_filter.cpp"
  "/home/powdersnow/workspace/C_derived/object_tracking/lib/tracker.cpp"
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles/cmake.verify_globs")
endif()
