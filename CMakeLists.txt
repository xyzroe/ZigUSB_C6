# The following lines of boilerplate must be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ZigUSB)

# Add a custom command that will be executed before building the main target
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Executing update version script..."
    COMMAND bash ${CMAKE_SOURCE_DIR}/tools/update_version.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Update version script finish!"
)

# Define a custom target for the OTA script
add_custom_target(make_ota ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Executing make OTA script..."
    COMMAND bash ${CMAKE_SOURCE_DIR}/tools/make_ota.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Make OTA script finish!"
)

# Add a dependency of this target on the main project build target
add_dependencies(make_ota ${CMAKE_PROJECT_NAME}.elf)