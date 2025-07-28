# The following line of boilerplate has to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# --- STEP 1: Set the SDKCONFIG variable FIRST ---
# This MUST be done before any ESP-IDF logic reads the configuration.
if(EXISTS ${CMAKE_SOURCE_DIR}/sdkconfig.local)
    set(SDKCONFIG ${CMAKE_SOURCE_DIR}/sdkconfig.local)
    message(STATUS "Using local configuration from sdkconfig.local")
endif()

# --- STEP 2: Include the ESP-IDF project script ---
# This script defines the idf_* functions and prepares the build system.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# --- STEP 3: Call project() ---
# This command officially initializes the project. It will read the SDKCONFIG
# variable you set in Step 1 and use it to configure the build.
project(ESPWaypointDevice)

# --- STEP 4: Set project-specific properties ---
# Now that the project is initialized and functions are defined, you can call them.
idf_build_set_property(MINIMAL_BUILD ON)
