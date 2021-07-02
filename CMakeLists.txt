cmake_minimum_required(VERSION 3.19)
project(ign_learn_system)

set(CMAKE_CXX_STANDARD 17)
set(IGN_PLUGIN_VER 1)
set(IGN_COMMON_VER 4)
set(IGN_GAZEBO_VER 6)

find_package(ignition-plugin${IGN_PLUGIN_VER} REQUIRED COMPONENTS register)
# find_package(ignition-common${IGN_COMMON_VER})
find_package(ignition-gazebo${IGN_GAZEBO_VER})

add_library(DiffSystem SHARED diff_system.cpp)

target_link_libraries(DiffSystem
        ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
        ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
        )


add_executable(DiffTest main.cpp)
target_link_libraries(DiffTest
        ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
        DiffSystem)