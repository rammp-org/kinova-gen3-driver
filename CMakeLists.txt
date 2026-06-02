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

# --- locate kortex_hardware (vendored KORTEX SDK) -----------------------------
# Override with -DKORTEX_HW_DIR=/path/to/kortex_hardware if it lives elsewhere.
set(KORTEX_HW_DIR "$ENV{HOME}/rtos_testing/cpp/kortex_hardware" CACHE PATH
    "Path to kortex_hardware (KORTEX API headers + static lib)")

# Guard on a header that actually exists in this SDK layout.
if(NOT EXISTS "${KORTEX_HW_DIR}/include/client/RouterClient.h")
    message(FATAL_ERROR
        "Cannot find KORTEX API headers under ${KORTEX_HW_DIR}/include. "
        "Pass -DKORTEX_HW_DIR=/path/to/kortex_hardware.")
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

# --- pinocchio ----------------------------------------------------------------
# Requires -DCMAKE_PREFIX_PATH pointing at the cmeel.prefix (see header above).
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# --- driver library -----------------------------------------------------------
# Later tasks will add KORTEX-dependent sources here, linking ${KORTEX_LIB},
# dl, and pthread (KORTEX is built with C++14 but links fine into a C++17
# consumer). For now the library only contains the Pinocchio Dynamics wrapper.
add_library(kinova_lowlevel
    src/dynamics.cpp
    src/telemetry.cpp
    src/telemetry_consumers.cpp
    src/rt_system.cpp
    src/sim_transport.cpp
    src/kortex_transport.cpp
    src/gravity_comp_mode.cpp
    src/rt_executor.cpp)
target_include_directories(kinova_lowlevel PUBLIC include)
target_link_libraries(kinova_lowlevel PUBLIC pinocchio::pinocchio pthread)

# KORTEX SDK: only kortex_transport.cpp pulls these headers in (pimpl keeps them
# out of public headers). KORTEX is built C++14; links into our C++17 build.
# _OS_UNIX selects the POSIX socket headers inside the KORTEX transport headers.
target_include_directories(kinova_lowlevel PRIVATE ${KORTEX_INCLUDE_DIRS})
target_compile_definitions(kinova_lowlevel PRIVATE _OS_UNIX)
target_link_libraries(kinova_lowlevel PRIVATE ${KORTEX_LIB} dl)

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
    tests/gravity_comp_mode_test.cpp)
target_link_libraries(unit_tests PRIVATE GTest::gtest_main Eigen3::Eigen kinova_lowlevel)
target_include_directories(unit_tests PRIVATE include)

# URDF path for tests that load the Gen3 model (used by later tasks).
target_compile_definitions(unit_tests PRIVATE
    URDF_PATH="${CMAKE_SOURCE_DIR}/models/gen3_7dof.urdf")

add_test(NAME unit_tests COMMAND unit_tests)
