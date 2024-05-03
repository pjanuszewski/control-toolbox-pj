cmake_minimum_required(VERSION 3.5)
project(ct VERSION 3.0.2 LANGUAGES CXX)

#set(USE_PRESPEC TRUE CACHE BOOL "Enable pre-specified template instantiations for Control Toolbox" FORCE)
#set(CMAKE_BUILD_TYPE Release)
#Make sure metapackage does not fail when building documentation
add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building CT Documentation..."
        VERBATIM)

add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append "Running clang-format on CT..."
        VERBATIM)

        find_package(kindr)
include_directories(${kindr_INCLUDE_DIRS})