cmake_minimum_required(VERSION 3.20)
project(noxim_dramsys LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# SystemC
# ============================================================================
set(SYSTEMC "/data/zhuo.wang/systemc302_v2_clean")
set(SYSTEMC_INCLUDE "${SYSTEMC}/include")
set(SYSTEMC_LIB "${SYSTEMC}/lib-linux64")

find_library(SYSTEMC_SHARED NAMES systemc-3.0.2 PATHS "${SYSTEMC}/lib-linux64" NO_DEFAULT_PATH)
if(NOT SYSTEMC_SHARED)
    message(FATAL_ERROR "libsystemc-3.0.2.so not found at ${SYSTEMC}/lib-linux64")
endif()

include_directories(SYSTEM ${SYSTEMC_INCLUDE})
link_directories(${SYSTEMC_LIB})

# ============================================================================
# Threads
# ============================================================================
find_package(Threads REQUIRED)

# ============================================================================
# yaml-cpp
# ============================================================================
set(YAML_CPP_ROOT "/data/zhuo.wang/noxim/libs/yaml-cpp")
set(YAML_CPP_INCLUDE "${YAML_CPP_ROOT}/include")
set(YAML_CPP_LIB "${YAML_CPP_ROOT}/build/libyaml-cpp.a")

# ============================================================================
# Noxim (from forked repo, built as static lib)
# ============================================================================
set(NOXIM_ROOT "/data/zhuo.wang/noxim")
set(NOXIM_SRC "${NOXIM_ROOT}/src")
set(NOXIM_BUILD "${NOXIM_ROOT}/bin/build")

set(NOXIM_SRCS
    "${NOXIM_SRC}/Stats.cpp"
    "${NOXIM_SRC}/Router.cpp"
    "${NOXIM_SRC}/GlobalTrafficHardcoding.cpp"
    "${NOXIM_SRC}/LocalRoutingTable.cpp"
    "${NOXIM_SRC}/ProcessingElement.cpp"
    "${NOXIM_SRC}/Initiator.cpp"
    "${NOXIM_SRC}/GlobalStats.cpp"
    "${NOXIM_SRC}/Power.cpp"
    "${NOXIM_SRC}/TokenRing.cpp"
    "${NOXIM_SRC}/ConfigurationManager.cpp"
    "${NOXIM_SRC}/Channel.cpp"
    "${NOXIM_SRC}/ReservationTable.cpp"
    "${NOXIM_SRC}/MM.cpp"
    "${NOXIM_SRC}/GlobalRoutingTable.cpp"
    "${NOXIM_SRC}/NoC.cpp"
    "${NOXIM_SRC}/Logger.cpp"
    "${NOXIM_SRC}/Target.cpp"
    "${NOXIM_SRC}/GlobalParams.cpp"
    "${NOXIM_SRC}/Buffer.cpp"
    "${NOXIM_SRC}/GlobalTrafficTable.cpp"
    "${NOXIM_SRC}/Hub.cpp"
    "${NOXIM_SRC}/selectionStrategies/Selection_RANDOM.cpp"
    "${NOXIM_SRC}/selectionStrategies/Selection_BUFFER_LEVEL.cpp"
    "${NOXIM_SRC}/selectionStrategies/Selection_NOP.cpp"
    "${NOXIM_SRC}/selectionStrategies/SelectionStrategies.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_ODD_EVEN.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_NEGATIVE_FIRST.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_XY.cpp"
    "${NOXIM_SRC}/routingAlgorithms/RoutingAlgorithms.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_WEST_FIRST.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_DELTA.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_TABLE_BASED.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_DYAD.cpp"
    "${NOXIM_SRC}/routingAlgorithms/Routing_NORTH_LAST.cpp"
)

add_library(noxim STATIC EXCLUDE_FROM_ALL ${NOXIM_SRCS})
target_include_directories(noxim PUBLIC ${NOXIM_SRC} SYSTEM ${SYSTEMC_INCLUDE} ${YAML_CPP_INCLUDE})
target_compile_options(noxim PRIVATE -O3 -Wall)
target_link_libraries(noxim PUBLIC ${YAML_CPP_LIB} -L${SYSTEMC}/lib-linux64 -lsystemc-3.0.2)

# ============================================================================
# DRAMSys (static lib from pre-built)
# ============================================================================
set(DRAMSYS_ROOT "/data/zhuo.wang/DRAMSys")
set(DRAMSYS_BUILD "${DRAMSYS_ROOT}/build")

# Hardcode paths directly (DRAMSys header lives at DRAMSys/DRAMSys.h)
set(DRAMSYS_INCLUDE "${DRAMSYS_ROOT}/src")

add_library(dramsys STATIC IMPORTED)
set_target_properties(dramsys PROPERTIES IMPORTED_LOCATION "${DRAMSYS_BUILD}/lib/libdramsys.a")

add_library(drampwr STATIC IMPORTED)
set_target_properties(drampwr PROPERTIES IMPORTED_LOCATION "${DRAMSYS_BUILD}/lib/libDRAMPower.a")

add_library(sqlite3 STATIC IMPORTED)
set_target_properties(sqlite3 PROPERTIES IMPORTED_LOCATION "${DRAMSYS_BUILD}/lib/libsqlite3.a")

set(DRAMSYS_EXTRA_INC
    "${DRAMSYS_ROOT}/build/_deps/nlohmann_json-src/single_include"
    "${DRAMSYS_ROOT}/build/_deps/dramutils-src/include"
)

# DRAMSys resource directory (contains config JSON files).
set(DRAMSYS_RESOURCE_DIR "${DRAMSYS_ROOT}/configs")

# DRAMSys header-only library (JSON parsing utilities).
set(DRAMSYS_UTILS_INC "${DRAMSYS_ROOT}/build/_deps/dramutils-src/include")

# ============================================================================
# Application (DramInterface + NoCInterface + sc_main)
# ============================================================================
file(GLOB NOXIM_DRAMSYS_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NoximGlobals.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DramInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NoCInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NoC2DramBridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NoximPE.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sc_main.cpp"
)

add_executable(noxim_dramsys ${NOXIM_DRAMSYS_SRCS})

target_include_directories(noxim_dramsys PRIVATE
    ${NOXIM_SRC}
    ${NOXIM_ROOT}
    ${DRAMSYS_INCLUDE}
    ${DRAMSYS_EXTRA_INC}
    ${DRAMSYS_UTILS_INC}
    SYSTEM ${SYSTEMC_INCLUDE}
)

target_compile_definitions(noxim_dramsys PRIVATE
    DRAMSYS_RESOURCE_DIR="${DRAMSYS_RESOURCE_DIR}"
)

target_link_options(noxim_dramsys PRIVATE
    "-Wl,--whole-archive,$<TARGET_FILE:noxim>,--no-whole-archive"
)

target_link_libraries(noxim_dramsys
    PRIVATE
    noxim
    dramsys
    drampwr
    sqlite3
    systemc
    ${YAML_CPP_LIB}
    Threads::Threads
)

# Copy configs directory to build
file(COPY configs DESTINATION ${CMAKE_BINARY_DIR})