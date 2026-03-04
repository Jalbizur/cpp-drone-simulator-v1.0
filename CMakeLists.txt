cmake_minimum_required(VERSION 3.21)

project(drone-sim VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL2 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_executable(drone-sim
    src/main.cpp
    src/Drone.cpp
    src/Physics.cpp
    src/PID.cpp
    src/Telemetry.cpp
)

target_link_libraries(drone-sim
    PRIVATE 
        SDL2::SDL2
        SDL2::SDL2main
        glm::glm
)

target_include_directories(drone-sim PRIVATE src)