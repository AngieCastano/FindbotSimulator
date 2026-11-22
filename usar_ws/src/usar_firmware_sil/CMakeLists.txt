cmake_minimum_required(VERSION 3.8)
project(usar_firmware_sil)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Firmware del ESP32 (zona de la sesión de hardware): se compila tal cual, sin copiarlo ni
# modificarlo. robot_core no depende de Arduino; config_gen.h lo genera tools/gen_config.py
# desde hardware/config/robot_hw.yaml.
set(FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../hardware/firmware" CACHE PATH
    "Carpeta del firmware del ESP32")
file(GLOB ROBOT_CORE_SOURCES "${FIRMWARE_DIR}/lib/robot_core/src/*.cpp")
if(NOT ROBOT_CORE_SOURCES)
  message(FATAL_ERROR "No se encontró el firmware en ${FIRMWARE_DIR}")
endif()
add_library(robot_core STATIC ${ROBOT_CORE_SOURCES})
target_include_directories(robot_core PUBLIC
  "${FIRMWARE_DIR}/lib/robot_core/src" "${FIRMWARE_DIR}/include")
set_target_properties(robot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(firmware_sil src/firmware_sil.cpp)
target_include_directories(firmware_sil PRIVATE include)
target_link_libraries(firmware_sil robot_core)
ament_target_dependencies(firmware_sil rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

install(TARGETS firmware_sil DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_firmware_sil test/test_firmware_sil.cpp)
  target_include_directories(test_firmware_sil PRIVATE include)
  target_link_libraries(test_firmware_sil robot_core)
endif()

ament_package()
