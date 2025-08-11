add_library(helper_lib STATIC
    helper_functions.cpp
)

add_executable(myApp
    myProject.cpp)
target_link_libraries(myApp PRIVATE lcm my_lcmtypes-cpp helper_lib)