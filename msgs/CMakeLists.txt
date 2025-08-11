include(${LCM_USE_FILE})

find_package(PythonInterp)
find_package(Java)
# If Java is found, include tools for Java (like building a jar)
if(JAVA_FOUND)
  include(UseJava)
endif()

# Gather all of the LCM message definitions into the `lcm_msgs` variable
file(GLOB lcm_msgs CONFIGURE_DEPENDS "*.lcm")

if(PYTHONINTERP_FOUND)
  # If Python was found, then we set this variable that tells the 
  # `lcm_wrap_types` call to build the Python bindings.
  set(python_lcm_gen PYTHON_SOURCES py_msg_sources)
endif()
if(JAVA_FOUND)
  # If Java was found, then we set this variable that tells the `lcm_wrap_types`
  # call to build the Java bindings.
  set(java_lcm_gen JAVA_SOURCES java_msg_sources)
endif()

lcm_wrap_types(
  CPP_HEADERS cpp_headers
  ${python_lcm_gen}
  ${java_lcm_gen}
  ${lcm_msgs}
)

lcm_add_library(my_lcmtypes-cpp CPP ${cpp_headers})
target_include_directories(my_lcmtypes-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

if(JAVA_FOUND)
  # If Java is found, build a jar file
  add_jar(lcm_msgs_jar
    OUTPUT_NAME simple_ekf_msgs
    INCLUDE_JARS lcm-java
    SOURCES "${java_msg_sources}"
  )
  install_jar(lcm_msgs_jar share/java)

endif()