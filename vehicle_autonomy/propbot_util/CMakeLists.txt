cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build Type")
endif()

project(propbot_util
    VERSION 1.0.0
    LANGUAGES CXX
)
find_package(catkin REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(propbot_util
    src/exception.cc
)

# Generate the version header
configure_file(
    include/propbot_util.hh.in 
    ${PROJECT_BINARY_DIR}/include/propbot_util.hh
)

target_include_directories(propbot_util
    PRIVATE
    include
    ${PROJECT_BINARY_DIR}/include
    
    PUBLIC
    $<INSTALL_INTERFACE:include>
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES propbot_util
)

install(DIRECTORY include/propbot_util/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*.in" EXCLUDE
)

install(TARGETS propbot_util
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
