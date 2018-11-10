cmake_minimum_required(VERSION 2.4.6)

project (LpmsSimpleExample)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

SET(CMAKE_BUILD_TYPE "Release")

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if (BUILD_ARCHITECTURE STREQUAL "32-bit") 
        include_directories("C:/OpenMAT/OpenMAT-1.3.5/include")
        link_directories("C:/OpenMAT/OpenMAT-1.3.5/lib/x86")
    endif()
    if (BUILD_ARCHITECTURE STREQUAL "64-bit") 
        include_directories("C:/OpenMAT/OpenMAT-1.3.5-x64/include")
        link_directories("C:/OpenMAT/OpenMAT-1.3.5-x64/lib/x64")
    endif()
endif()

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(sources
    main.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11") 
endif()

add_executable(${CMAKE_PROJECT_NAME} main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_libraries(${CMAKE_PROJECT_NAME} LpSensorD.lib)
    endif()
    if (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_libraries(${CMAKE_PROJECT_NAME} LpSensor.lib)
    endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(${CMAKE_PROJECT_NAME} LpSensor.so)
    target_link_libraries(${CMAKE_PROJECT_NAME} pthread)
    target_link_libraries(${CMAKE_PROJECT_NAME} dl)
    target_link_libraries(${CMAKE_PROJECT_NAME} rt)
endif()
