cmake_minimum_required(VERSION 3.20)
project(robotiq_driver_noros VERSION 0.0.1 LANGUAGES CXX)

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
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install library.
install(
  TARGETS robotiq_driver_noros gripper_interface_test
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

# Install export targets
install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE robotiq::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Generate config files
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
