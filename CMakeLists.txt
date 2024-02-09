cmake_minimum_required (VERSION 3.2)
project(c_physics)
set(CMAKE_C_STANDARD 11)
set(EXTERNAL_LIBRARIES_DIRECTORY_PATH ${CMAKE_SOURCE_DIR}/src/ext)
include(${CMAKE_SOURCE_DIR}/src/ext/external_Setup.cmake )

file(GLOB_RECURSE src ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.h NO_CACHE )

add_executable(${PROJECT_NAME} ${src})
target_link_libraries(${PROJECT_NAME} SDL2::SDL2 SDL2::SDL2main)
#target_link_libraries(${PROJECT_NAME} SDL2::SDL2 SDL2::SDL2main -mwindows)
target_link_libraries(${PROJECT_NAME} upng)
target_link_libraries(${PROJECT_NAME} glad)
target_link_libraries(${PROJECT_NAME} cglm)
target_include_directories(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY_PATH}/upng/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY_PATH}/glad/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY_PATH}/cglm/include)


file(GLOB assets ${CMAKE_SOURCE_DIR}/assets/* NO_CACHE )
file(COPY ${assets} DESTINATION "${CMAKE_BINARY_DIR}/assets")