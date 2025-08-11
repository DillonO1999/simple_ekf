add_library(helper_lib STATIC
    helper_functions.cpp
    propagate.cpp
    earth_functions.cpp
    dynamics_functions.cpp
)

add_executable(simple_ekf
    simple_ekf.cpp)
target_link_libraries(simple_ekf PRIVATE lcm my_lcmtypes-cpp yaml-cpp helper_lib)