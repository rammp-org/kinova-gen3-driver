# Kinova Gen3 low-level C++ driver.
#
# Targets Linux/aarch64 (NVIDIA Jetson). CANNOT build on macOS: the KORTEX SDK
# and the RT APIs used by later tasks are Linux-only. Build on the Jetson host
# (abra) via the dev loop:
#     ./scripts/build_on_abra.sh        # sync + cmake + build + ctest
# or manually:
#     ./scripts/sync_to_abra.sh
#     ssh abra 'cd ~/kinova-gen3-driver/build && cmake .. \
#       -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
#       && cmake --build . -j && ctest --output-on-failure'
#
# VERIFIED WORKING CMAKE_PREFIX_PATH (for find_package(pinocchio)):
#     /usr/local/lib/python3.10/dist-packages/cmeel.prefix
# (pinocchioConfig.cmake lives at <prefix>/lib/cmake/pinocchio/.)

cmake_minimum_required(VERSION 3.16)
project(kinova_gen3_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O2 -Wall -Wextra -Wno-deprecated-declarations)

# --- real-robot (KORTEX) build switch -----------------------------------------
# OFF (default): SIM-ONLY build. KortexTransport is NOT compiled or linked, no
# KORTEX headers are required, and the app gets -DKINOVA_NO_KORTEX so the real
# --ip path is #ifdef'd out. This is the only configuration that links on the
# Jetson today, because the vendored libKortexApiCpp.a is x86-64 ONLY (there is
# no aarch64 KORTEX C++ lib on abra yet).
#
# ON: compile + link the real KortexTransport path. Requires an aarch64 build of
# the KORTEX C++ SDK with KORTEX_HW_DIR pointing at it. Enable later for the
# robot-in-the-loop bring-up:
#     cmake .. -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/path/to/aarch64/kortex_hardware
option(KINOVA_ENABLE_KORTEX
    "Compile/link the real KortexTransport path (needs an aarch64 KORTEX C++ SDK)" OFF)

if(KINOVA_ENABLE_KORTEX)
    # --- locate kortex_hardware (vendored KORTEX SDK) -------------------------
    # Override with -DKORTEX_HW_DIR=/path/to/kortex_hardware if it lives elsewhere.
    set(KORTEX_HW_DIR "$ENV{HOME}/rtos_testing/cpp/kortex_hardware" CACHE PATH
        "Path to kortex_hardware (KORTEX API headers + static lib)")

    # Guard on a header that actually exists in this SDK layout. Only validated
    # when KORTEX is enabled, so a box without KORTEX headers can still build sim.
    if(NOT EXISTS "${KORTEX_HW_DIR}/include/client/RouterClient.h")
        message(FATAL_ERROR
            "KINOVA_ENABLE_KORTEX=ON but cannot find KORTEX API headers under "
            "${KORTEX_HW_DIR}/include. Pass -DKORTEX_HW_DIR=/path/to/kortex_hardware.")
    endif()

    set(KORTEX_INCLUDE_DIRS
        ${KORTEX_HW_DIR}/include
        ${KORTEX_HW_DIR}/include/client
        ${KORTEX_HW_DIR}/include/client_stubs
        ${KORTEX_HW_DIR}/include/common
        ${KORTEX_HW_DIR}/include/messages
        ${KORTEX_HW_DIR}/include/google
    )
    set(KORTEX_LIB ${KORTEX_HW_DIR}/lib/release/libKortexApiCpp.a)
endif()

# --- pinocchio ----------------------------------------------------------------
# Requires -DCMAKE_PREFIX_PATH pointing at the cmeel.prefix (see header above).
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# --- driver library -----------------------------------------------------------
# The KORTEX-dependent source (kortex_transport.cpp) is only compiled into the
# library when KINOVA_ENABLE_KORTEX=ON. The rest of the units are KORTEX-free
# (pimpl keeps KORTEX headers out of the public interface).
set(KINOVA_LIB_SOURCES
    src/dynamics.cpp
    src/telemetry.cpp
    src/telemetry_consumers.cpp
    src/rt_system.cpp
    src/sim_transport.cpp
    src/gravity_comp_mode.cpp
    src/rt_executor.cpp)
if(KINOVA_ENABLE_KORTEX)
    list(APPEND KINOVA_LIB_SOURCES src/kortex_transport.cpp)
endif()

add_library(kinova_lowlevel ${KINOVA_LIB_SOURCES})
target_include_directories(kinova_lowlevel PUBLIC include)
target_link_libraries(kinova_lowlevel PUBLIC pinocchio::pinocchio pthread)

if(KINOVA_ENABLE_KORTEX)
    # KORTEX SDK: only kortex_transport.cpp pulls these headers in. KORTEX is
    # built C++14; links into our C++17 build. _OS_UNIX selects the POSIX socket
    # headers inside the KORTEX transport headers.
    target_include_directories(kinova_lowlevel PRIVATE ${KORTEX_INCLUDE_DIRS})
    target_compile_definitions(kinova_lowlevel PRIVATE _OS_UNIX)
    target_link_libraries(kinova_lowlevel PRIVATE ${KORTEX_LIB} dl)
endif()

# --- apps ---------------------------------------------------------------------
# benchmark_grav_comp runs the gravity-comp mode through the RtExecutor and
# reports timing. Verified with --sim (SimTransport); the real KORTEX path is
# compiled in but only built/linked, never exercised while the robot is off.
add_executable(benchmark_grav_comp apps/benchmark_grav_comp.cpp)
target_link_libraries(benchmark_grav_comp PRIVATE kinova_lowlevel Eigen3::Eigen)
target_include_directories(benchmark_grav_comp PRIVATE include)
if(KINOVA_ENABLE_KORTEX)
    # Real-robot build: KortexTransport is compiled into the library and the
    # KORTEX include dirs / -D_OS_UNIX are needed where the app references the
    # KortexTransport header. The real --ip path is active (no KINOVA_NO_KORTEX).
    target_include_directories(benchmark_grav_comp PRIVATE ${KORTEX_INCLUDE_DIRS})
    target_compile_definitions(benchmark_grav_comp PRIVATE _OS_UNIX)
else()
    # SIM-ONLY build (default). KINOVA_NO_KORTEX #ifdef's out the KortexTransport
    # reference so no KORTEX symbols are demanded -- required on the Jetson, where
    # the vendored libKortexApiCpp.a is x86-64 only and would fail to link into an
    # aarch64 executable.
    target_compile_definitions(benchmark_grav_comp PRIVATE KINOVA_NO_KORTEX)
endif()

# --- tests --------------------------------------------------------------------
enable_testing()
find_package(GTest REQUIRED)

add_executable(unit_tests
    tests/smoke_test.cpp
    tests/joint_types_test.cpp
    tests/dynamics_test.cpp
    tests/telemetry_test.cpp
    tests/rt_system_test.cpp
    tests/sim_transport_test.cpp
    tests/gravity_comp_mode_test.cpp
    tests/rt_safety_test.cpp)
target_link_libraries(unit_tests PRIVATE GTest::gtest_main Eigen3::Eigen kinova_lowlevel)
target_include_directories(unit_tests PRIVATE include)

# URDF path for tests that load the Gen3 model (used by later tasks).
target_compile_definitions(unit_tests PRIVATE
    URDF_PATH="${CMAKE_SOURCE_DIR}/models/gen3_7dof.urdf")

add_test(NAME unit_tests COMMAND unit_tests)
