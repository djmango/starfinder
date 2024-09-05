cmake_minimum_required(VERSION 3.16.3)
project(starfinder)


add_compile_options(-std=c++17)


find_package(Boost REQUIRED COMPONENTS
    program_options
)

find_package(OpenCV REQUIRED)


include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_render
    src/render.cpp
)
target_link_libraries(${PROJECT_NAME}_render
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_render
    PROPERTIES
        OUTPUT_NAME render
)
