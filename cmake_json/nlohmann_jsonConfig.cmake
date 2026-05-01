# nlohmann_jsonConfig.cmake - header-only, compatible with FindPackageByModuleName
set(nlohmann_json_FOUND TRUE)
set(nlohmann_json_VERSION "3.12.0")

include(CMakeFindDependencyMacro)

# Create the target if not exists
if(NOT TARGET nlohmann_json::nlohmann_json)
    add_library(nlohmann_json INTERFACE)
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
    target_include_directories(nlohmann_json INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../nlohmann_json-src/single_include")
endif()
