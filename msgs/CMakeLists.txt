if(${Python_Interpreter_FOUND})
  set(python_args PYTHON_SOURCES python_sources)
endif()
if(JAVA_FOUND)
  set(java_args JAVA_SOURCES java_sources)
endif()

lcm_wrap_types(
  CPP_HEADERS cpp_headers
  ${python_args}
  ${java_args}
  imu.lcm
)

lcm_add_library(my_lcmtypes-cpp CPP ${cpp_headers})
target_include_directories(my_lcmtypes-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
