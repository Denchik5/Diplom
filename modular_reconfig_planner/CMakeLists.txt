cmake_minimum_required(VERSION 3.20)
project(ModularReconfigurationPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_OFFLINE_RUNNER "Build offline planner executable" ON)
option(BUILD_COPPELIA_RUNNER "Build CoppeliaSim planner executable" ON)
option(USE_COPPELIASIM_ZMQ "Use the real CoppeliaSim ZeroMQ Remote API client" OFF)

add_library(project_options INTERFACE)
target_include_directories(project_options INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/minijson
)

file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    src/core/*.cpp
    src/planning/*.cpp
    src/logging/*.cpp
)

add_library(planner_core ${CORE_SOURCES})
target_link_libraries(planner_core PUBLIC project_options)
target_compile_features(planner_core PUBLIC cxx_std_17)

add_custom_target(copy_runtime_data ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/scenarios ${CMAKE_CURRENT_BINARY_DIR}/scenarios
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/configurations ${CMAKE_CURRENT_BINARY_DIR}/configurations
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/weights ${CMAKE_CURRENT_BINARY_DIR}/weights
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/data ${CMAKE_CURRENT_BINARY_DIR}/data
    COMMENT "Copying runtime scenarios, configurations, weights and motion settings"
)

if(BUILD_OFFLINE_RUNNER)
    add_executable(planner_offline
        src/main_offline.cpp
        src/sim/MockCoppeliaSimInterface.cpp
    )
    target_link_libraries(planner_offline PRIVATE planner_core project_options)
    add_dependencies(planner_offline copy_runtime_data)
endif()

if(BUILD_COPPELIA_RUNNER)
    if(USE_COPPELIASIM_ZMQ)
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/zmqRemoteApi/clients/cpp/CMakeLists.txt")
            message(FATAL_ERROR "ZeroMQ Remote API client not found. Copy CoppeliaSim/programming/zmqRemoteApi to external/zmqRemoteApi, then configure with -DUSE_COPPELIASIM_ZMQ=ON")
        endif()

        add_subdirectory(external/zmqRemoteApi/clients/cpp external_zmq_build EXCLUDE_FROM_ALL)
        add_executable(planner_coppelia
            src/main_coppelia.cpp
            src/sim/CoppeliaSimZmqInterface.cpp
        )
        target_compile_definitions(planner_coppelia PRIVATE MRR_WITH_COPPELIASIM_ZMQ=1)
        target_link_libraries(planner_coppelia PRIVATE planner_core project_options RemoteAPIClient)
        target_include_directories(planner_coppelia PRIVATE external/zmqRemoteApi/clients/cpp)
    else()
        add_executable(planner_coppelia
            src/main_coppelia.cpp
            src/sim/CoppeliaSimZmqInterfaceStub.cpp
        )
        target_compile_definitions(planner_coppelia PRIVATE MRR_WITH_COPPELIASIM_ZMQ=0)
        target_link_libraries(planner_coppelia PRIVATE planner_core project_options)
    endif()
    add_dependencies(planner_coppelia copy_runtime_data)
endif()
