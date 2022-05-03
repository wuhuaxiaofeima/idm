cmake_minimum_required (VERSION 3.0)
project(rdb_client_project)

# compiler options
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# find packages
#find_package(GeographicLib)

# include headers search paths
include_directories ("${CMAKE_SOURCE_DIR}/3rd/RDBHandler")
include_directories ("${CMAKE_SOURCE_DIR}/src/PIDController")

# add sub-target directories
add_subdirectory ("${CMAKE_SOURCE_DIR}/3rd/RDBHandler")
add_subdirectory ("${CMAKE_SOURCE_DIR}/src/PIDController")

# complie executable
add_executable (ExampleConsoleDriverCtrlTorqueIDM "${CMAKE_SOURCE_DIR}/src/ExampleConsoleDriverCtrlTorqueIDM")
# add_executable (auto-test "${CMAKE_SOURCE_DIR}/src/auto-test.cpp")
#TARGET_LINK_LIBRARIES （设置要链接的库文件的名称）
target_link_libraries (ExampleConsoleDriverCtrlTorqueIDM pid_controller rdb_handler)
# target_link_libraries (auto-test)



# cmake_minimum_required(VERSION 3.0)
# project(rdb_client_project)
# add_definitions(-w)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


# include_directories ("${PROJECT_SOURCE_DIR}/VtdFramework")
# include_directories ("${PROJECT_SOURCE_DIR}/RDBClientSample/PIDController")

# add_subdirectory ("${PROJECT_SOURCE_DIR}/VtdFramework")
# add_subdirectory ("${PROJECT_SOURCE_DIR}/RDBClientSample/PIDController")


# add_executable(${PROJECT_NAME} ExampleConsoleDriverCtrlTorque4.cpp )

# target_link_libraries(${PROJECT_NAME}VtdFramework)
# target_link_libraries(${PROJECT_NAME}RDBClientSample/PIDController)







# 报错boost版本太低？？
# cmake_minimum_required(VERSION 3.0)
# project(rdb_client_project)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


# add_subdirectory ("${CMAKE_SOURCE_DIR}/VtdFramework")
# add_subdirectory ("${CMAKE_SOURCE_DIR}/RDBClientSample/PIDController")

# include_directories ("${CMAKE_SOURCE_DIR}/VtdFramework")
# include_directories ("${CMAKE_SOURCE_DIR}/RDBClientSample/PIDController")

# add_executable(${PROJECT_NAME} ExampleConsoleDriverCtrlTorque4.cpp )

# target_link_libraries(${PROJECT_NAME}VtdFramework)
# target_link_libraries(${PROJECT_NAME}RDBClientSample/PIDController)


# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CMAKE_SOURCE_DIR})
