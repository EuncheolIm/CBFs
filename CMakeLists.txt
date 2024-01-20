cmake_minimum_required(VERSION 3.15)
project(sm)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")
#fing eigen library
find_package (Eigen3 3.3.7 REQUIRED)
message(STATUS "Eigen3 : " ${EIGEN3_INCLUDE_DIR}/Eigen)
message(STATUS "current : " ${CMAKE_CURRENT_SOURCE_DIR})

option(QPOASES_BUILD_EXAMPLES "Build examples." OFF)

add_subdirectory("${PROJECT_SOURCE_DIR}/qpOASES") # CMake Path append
add_definitions(-DMJSIMULATE_STATIC) # use static =,solve the "error LNK2001/LNK2019" in windows
add_subdirectory(libs/yaml-cpp) # This is one way of referencing another CMake project

# set MUJOCO root
set(MUJOCO_HOME ${CMAKE_SOURCE_DIR}/mujoco-2.3.2)
set(MUJOCO_SIMULATE ${MUJOCO_HOME}/simulate )

message(STATUS "MUJOCO_HOME: " ${MUJOCO_HOME})
message(STATUS "MUJOCO_SIMULATE: " ${MUJOCO_SIMULATE})

link_directories(${MUJOCO_HOME}/lib/)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MUJOCO_HOME}/include
  ${MUJOCO_HOME}/simulate
  /usr/local/include/rbdl
  ${MUJOCO_HOME}/lodepng-master
  ${YAML_CPP_INCLUDE_DIR}
  DEPENDS system_lib
)

message(STATUS "Include dir: " ${INCLUDE_DIRECTORIES})

set(COM_MUJOCO
        ${MUJOCO_SIMULATE}/glfw_adapter.cc
        ${MUJOCO_SIMULATE}/glfw_dispatch.cc
        ${MUJOCO_SIMULATE}/simulate.cc
        ${MUJOCO_SIMULATE}/platform_ui_adapter.cc
        ${MUJOCO_HOME}/lodepng-master/lodepng.cpp
        )

set(COM_LIB
        libmujoco.so.2.3.1
        glfw3
        -lrbdl
        -lrbdl_urdfreader
        -lpthread
        # -lqpOASES
        qpOASES
        yaml-cpp
        )
set(COM_SRC

        src/controller.cpp

        src/main.cc
        src/trajectory.cpp
        src/robotmodel.cpp
        src/quadraticprogram.cc
        )

add_executable(main ${COM_MUJOCO} ${COM_SRC})
target_link_libraries(main  ${COM_LIB} ${CMAKE_DL_LIBS})
