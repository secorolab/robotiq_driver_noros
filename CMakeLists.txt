cmake_minimum_required(VERSION 3.8)
project(robotiq_driver_noros LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(serial REQUIRED)

# Robotiq driver library.

add_library(
  robotiq_driver_noros
  SHARED
  # include/robotiq_driver/fake/fake_driver.hpp
  # include/robotiq_driver/crc_utils.hpp
  include/robotiq_driver_noros/data_utils.hpp
  include/robotiq_driver_noros/default_driver.hpp
  # include/robotiq_driver/default_driver_factory.hpp
  include/robotiq_driver_noros/default_serial.hpp
  # include/robotiq_driver/default_serial_factory.hpp
  # include/robotiq_driver/driver.hpp
  # include/robotiq_driver/driver_exception.hpp
  # include/robotiq_driver/driver_factory.hpp
  # include/robotiq_driver/hardware_interface.hpp
  # include/robotiq_driver/serial.hpp
  # include/robotiq_driver/serial_factory.hpp
  # include/robotiq_driver/visibility_control.hpp
  src/crc_utils.cpp
  src/data_utils.cpp
  # src/hardware_interface.cpp
  src/default_driver.cpp
  # src/default_driver_factory.cpp
  # src/fake/fake_driver.cpp
  src/default_serial.cpp
  # src/default_serial_factory.cpp
)
target_link_libraries(robotiq_driver_noros atomic)
target_include_directories(
  robotiq_driver_noros
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  robotiq_driver_noros
  serial::serial
)

## Test executable
add_executable(gripper_interface_test
  src/gripper_interface_test.cpp
  src/command_line_utility.cpp
)
target_include_directories(
  gripper_interface_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gripper_interface_test
  robotiq_driver_noros
)

###############################################################################
# INSTALL

# Install all files of the include folder into the give destination.
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

# Install our library.
install(
  TARGETS robotiq_driver_noros
  EXPORT robotiq_driver_noros_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)
