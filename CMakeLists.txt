cmake_minimum_required(VERSION 3.16)
project(specialist_sdk VERSION 0.2.0 LANGUAGES CXX)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(SPECIALIST_IS_TOP_LEVEL ON)
else()
  set(SPECIALIST_IS_TOP_LEVEL OFF)
endif()

# The sources use C++20. The public headers need only C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(SPECIALIST_BUILD_EXAMPLES "Build the example program" ${SPECIALIST_IS_TOP_LEVEL})
option(SPECIALIST_BUILD_PYTHON "Build the Python extension module" OFF)
option(SPECIALIST_INSTALL "Generate the install and CMake export rules" ON)

find_package(Threads REQUIRED)

find_package(feetech_driver 0.2 QUIET)
if(NOT feetech_driver_FOUND)
  set(FEETECH_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(FEETECH_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  # The exported target links feetech::feetech, so the fetched copy must install too.
  set(FEETECH_INSTALL ${SPECIALIST_INSTALL} CACHE BOOL "" FORCE)
  if(NOT FETCHCONTENT_SOURCE_DIR_FEETECH_DRIVER AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../feetech_driver/CMakeLists.txt")
    set(FETCHCONTENT_SOURCE_DIR_FEETECH_DRIVER "${CMAKE_CURRENT_SOURCE_DIR}/../feetech_driver")
  endif()
  include(FetchContent)
  FetchContent_Declare(feetech_driver
    GIT_REPOSITORY https://github.com/specialist-ai/feetech_driver.git
    GIT_TAG 246b6177c3c17e9f91dcb31a7a4bb18f48bcfe1b  # feetech_driver main
  )
  FetchContent_MakeAvailable(feetech_driver)
endif()

add_library(specialist src/log.cpp src/gripper.cpp)
add_library(specialist::specialist ALIAS specialist)
target_include_directories(specialist PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(specialist PUBLIC cxx_std_17)
target_link_libraries(specialist PUBLIC Threads::Threads PRIVATE feetech::feetech)
set_target_properties(specialist PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  POSITION_INDEPENDENT_CODE ON
)

if(SPECIALIST_INSTALL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(TARGETS specialist EXPORT specialist_sdkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  set(_specialist_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/specialist_sdk)
  install(EXPORT specialist_sdkTargets NAMESPACE specialist:: DESTINATION ${_specialist_cmake_dir})
  write_basic_package_version_file(specialist_sdkConfigVersion.cmake COMPATIBILITY SameMinorVersion)
  configure_package_config_file(cmake/specialist_sdkConfig.cmake.in specialist_sdkConfig.cmake
    INSTALL_DESTINATION ${_specialist_cmake_dir}
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/specialist_sdkConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/specialist_sdkConfigVersion.cmake"
    DESTINATION ${_specialist_cmake_dir}
  )
endif()

if(SPECIALIST_BUILD_PYTHON)
  # scikit-build-core sets SKBUILD_SABI_COMPONENT when it builds a cp312-abi3 wheel.
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module ${SKBUILD_SABI_COMPONENT} REQUIRED)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core STABLE_ABI NB_STATIC python/bindings.cpp)
  target_link_libraries(_core PRIVATE specialist)
  install(TARGETS _core LIBRARY DESTINATION specialist_sdk)
  nanobind_add_stub(_core_stub
    INSTALL_TIME
    MODULE specialist_sdk._core
    OUTPUT specialist_sdk/_core.pyi
    MARKER_FILE specialist_sdk/py.typed
    PYTHON_PATH .
  )
endif()

if(SPECIALIST_BUILD_EXAMPLES)
  add_executable(specialist_open_close examples/open_close.cpp)
  target_link_libraries(specialist_open_close PRIVATE specialist)
endif()
