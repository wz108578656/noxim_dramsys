cmake_minimum_required(VERSION 3.25)
project(noxim_dramsys LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies via FetchContent
# ============================================================================
include(FetchContent)

# ---------------------------------------------------------------------------
# yaml-cpp (NoC config parsing, no SystemC dependency)
# ---------------------------------------------------------------------------
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.7.0
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(yaml-cpp)

# ---------------------------------------------------------------------------
# DRAMSys (DRAM simulator — handles SystemC 3.0.2 + dependencies internally)
# Must come before noxim so SystemC targets are available for noxim to use.
# ---------------------------------------------------------------------------
set(DRAMSYS_USE_FETCH_CONTENT    ON  CACHE BOOL "" FORCE)
set(DRAMSYS_BUILD_CLI            OFF CACHE BOOL "" FORCE)
set(DRAMSYS_BUILD_TESTS          OFF CACHE BOOL "" FORCE)
set(DRAMSYS_BUILD_BENCHMARKS     OFF CACHE BOOL "" FORCE)
set(DRAMSYS_BUILD_TOOLS          OFF CACHE BOOL "" FORCE)
set(DRAMSYS_BUILD_TRACE_ANALYZER OFF CACHE BOOL "" FORCE)
set(DRAMSYS_USE_EXTENSIONS       OFF CACHE BOOL "" FORCE)

FetchContent_Declare(dramsys
    GIT_REPOSITORY https://github.com/wz108578656/DRAMSys.git
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(dramsys)

# ---------------------------------------------------------------------------
# Noxim fork (NoC simulator used as source library)
# Has no CMakeLists.txt — we download and list sources manually.
# Uses SystemC (fetched by DRAMSys above) via the SystemC::systemc target.
# ---------------------------------------------------------------------------
FetchContent_Declare(noxim
    GIT_REPOSITORY https://github.com/wz108578656/noxim.git
    GIT_SHALLOW TRUE)
FetchContent_GetProperties(noxim)
if(NOT noxim_POPULATED)
    FetchContent_Populate(noxim)
endif()

set(NOXIM_SRC "${noxim_SOURCE_DIR}/src")

list(APPEND NOXIM_SRCS
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
target_include_directories(noxim PUBLIC ${NOXIM_SRC})
target_compile_options(noxim PRIVATE -O3 -Wall)

# Link against SystemC from DRAMSys FetchContent (ensures correct include paths and ABI)
target_link_libraries(noxim PUBLIC yaml-cpp::yaml-cpp SystemC::systemc)

# ============================================================================
# Application (NoC Crossbar + DRAMSys co-simulation)
# ============================================================================
file(GLOB NOXIM_DRAMSYS_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DramInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/traffic_pe.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dram_pe.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/channel_scheduler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/xbar_4x8.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sc_main.cpp"
)

add_executable(noxim_dramsys ${NOXIM_DRAMSYS_SRCS})

target_include_directories(noxim_dramsys PRIVATE
    ${NOXIM_SRC}
    SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/SYSTEM
)

target_compile_definitions(noxim_dramsys PRIVATE
    DRAMSYS_RESOURCE_DIR="${dramsys_SOURCE_DIR}/configs"
)

find_package(Threads REQUIRED)

target_link_libraries(noxim_dramsys PRIVATE
    noxim
    DRAMSys::DRAMSys
    Threads::Threads
)

# Copy configs directory to build (for DRAMSys path resolution)
file(COPY configs DESTINATION ${CMAKE_BINARY_DIR})
