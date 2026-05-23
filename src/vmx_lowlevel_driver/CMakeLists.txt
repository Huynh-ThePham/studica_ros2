cmake_minimum_required(VERSION 3.8)
project(vmx_lowlevel_driver VERSION 1.0.0 LANGUAGES CXX C)

# C++17 is required for std::clamp, std::any_of with lambdas, structured bindings
# in run_periodic helpers, etc.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# VMX boards are RAM-constrained; default to MinSizeRel so a fresh `colcon build`
# does not produce a debug binary. On-board builds: `scripts/build_vmx_release.sh`
# defaults to MinSizeRel (-Os) to keep peak compiler memory under ~400 MB on 1 GB Pi.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "Build type on single-configuration generators." FORCE)
endif()

find_package(ament_cmake REQUIRED)

# Studica VMX HAL is shipped as system-installed shared libs + headers under
# /usr/local on the official VMX image. Override these by passing -DVMX_HAL_PREFIX=
# if you install them elsewhere.
set(VMX_HAL_PREFIX "/usr/local" CACHE PATH "Prefix where the Studica VMX HAL is installed.")
set(VMX_HAL_INCLUDE_DIRS
  "${VMX_HAL_PREFIX}/include/vmxpi"
  "${VMX_HAL_PREFIX}/include/studica_drivers"
)
set(VMX_HAL_LIBS
  "${VMX_HAL_PREFIX}/lib/vmxpi/libvmxpi_hal_cpp.so"
  "${VMX_HAL_PREFIX}/lib/studica_drivers/libstudica_drivers.so"
)

add_executable(vmx_udp_lowlevel_daemon src/vmx_udp_lowlevel_daemon.cpp)
add_executable(titan_port_test src/titan_port_test.cpp)

target_include_directories(vmx_udp_lowlevel_daemon
  PRIVATE
    include
    ${VMX_HAL_INCLUDE_DIRS}
)
target_include_directories(titan_port_test PRIVATE ${VMX_HAL_INCLUDE_DIRS})

target_link_libraries(vmx_udp_lowlevel_daemon
  ${VMX_HAL_LIBS}
  atomic
  pthread
  rt
)

target_link_libraries(titan_port_test
  ${VMX_HAL_LIBS}
  atomic
  pthread
  rt
)

install(TARGETS
  vmx_udp_lowlevel_daemon
  titan_port_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES systemd/vmx-udp-lowlevel.service
  DESTINATION share/${PROJECT_NAME}/systemd
)

install(PROGRAMS
  scripts/build_vmx_release.sh
  scripts/run_vmx_udp_lowlevel.sh
  scripts/launch_vmx_udp_lowlevel.sh
  scripts/send_vmx_udp_stop.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_export_include_directories(include)

ament_package()
