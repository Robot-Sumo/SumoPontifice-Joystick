cmake_minimum_required (VERSION 2.8) 

add_compile_options(-std=c++0x -Wall)
project (joystickPS3)
#set(raspicam_DIR /usr/local/lib/cmake/)
#find_package(raspicam REQUIRED)
find_library(wiringPi_LIBS wiringPi wiringSerial)
#find_package(Threads REQUIRED)


set(joystickPS3_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/joystick.cpp
    ${PROJECT_SOURCE_DIR}/src/driver.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)


add_library(joystickPS3_libs ${joystickPS3_SOURCE_FILES})
add_executable (main  ${PROJECT_SOURCE_DIR}/src/main.cpp)  
target_link_libraries (main joystickPS3_libs ${wiringPi_LIBS})




#target_link_libraries (serialTest ${raspicam_LIBS} ${CMAKE_THREAD_LIBS_INIT} )

# add_executable (wiring wiring.cpp) 
# target_link_libraries(wiring ${wiringPi_LIB} ${raspicam_LIBS})





