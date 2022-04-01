cmake_minimum_required(VERSION 3.19)
project(myCobotCppPrivate VERSION 0.1.0
    DESCRIPTION "C++ Interface for myCobot"
    LANGUAGES CXX
)

if("$ENV{QTDIR}" STREQUAL "")
    message(FATAL_ERROR "Please set env var QTDIR to Qt toolchain path")
endif()
find_program(QT_QMAKE_EXECUTABLE
    NAMES qmake
    PATHS "$ENV{QTDIR}/bin"
    REQUIRED
    NO_DEFAULT_PATH
)
set(QT_INSTALL_PREFIX "$ENV{QTDIR}")
execute_process(COMMAND ${QT_QMAKE_EXECUTABLE} "-query" "QT_INSTALL_PLUGINS"
    OUTPUT_VARIABLE QT_INSTALL_PLUGINS
    ERROR_VARIABLE _stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
)
set(CMAKE_PREFIX_PATH "${QT_INSTALL_PREFIX}")

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds not allowed. Please run mkdir build; cd build; cmake ..")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(UNIX)
    # UNIX like, APPLE or CYGWIN
    add_compile_definitions(OS_UNIX)
endif()
if(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # both 32-bit and 64-bit Windows
    add_compile_definitions(OS_WINDOWS)
endif()
if(APPLE)
    # macOS, iOS, tvOS, watchOS
    add_compile_definitions(OS_APPLE)
endif()
if(CYGWIN)
    add_compile_definitions(OS_CYGWIN)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_compile_definitions(OS_LINUX)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    add_compile_definitions(OS_MACOS)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    add_compile_definitions(COMPILER_GCC)
elseif(CMAKE_CXX_COMPILER_ID MATCHES AppleClang)
    add_compile_definitions(COMPILER_APPLECLANG)
elseif(CMAKE_CXX_COMPILER_ID MATCHES Clang)
    add_compile_definitions(COMPILER_CLANG)
elseif(CMAKE_CXX_COMPILER_ID MATCHES MSVC)
    add_compile_definitions(COMPILER_MSVC)
else()
    add_compile_definitions(COMPILER_OTHER)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "i[3456]86")
    add_compile_definitions(CPU_X86)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_compile_definitions(CPU_X64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm.*")
    add_compile_definitions(CPU_ARM)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_definitions(CPU_AARCH64)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
endif()
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    add_compile_definitions(DEBUG)
else()
    add_compile_definitions(NDEBUG)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "ON" FORCE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# check for LTO (Link-Time Optimization)
#XXX: LTO on wingw windows is broken, even though it says it is supported
#TODO: add compiler detection (gcc, clang, msvc, mingw, cygwin)
if(NOT(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Windows"))
    include(CheckIPOSupported)
    check_ipo_supported(RESULT lto_supported OUTPUT lto_supported_output)
    if(lto_supported)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    else()
        message(WARNING "LTO is not supported: ${lto_supported_output}")
    endif()
else()
    message(WARNING "LTO is not supported on MinGW")
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache found: ${CCACHE_PROGRAM}")
endif()

include(CMakePrintHelpers)
include(CMakePrintSystemInformation)
include(CMakeDependentOption)

# Install
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
else()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")
foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
    message(STATUS "Installing ${p} components to ${_path}")
    unset(_path)
endforeach()

# Generate folders for IDE targets (e.g., VisualStudio solutions)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(IDE_FOLDER "")

enable_testing()
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

##################
# Build Options
##################

if(WIN32)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

#######
# Qt
#######

find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Qt5 REQUIRED COMPONENTS SerialPort)

####################
# Target Settings
####################

add_library(myCobotCpp SHARED)
set_target_properties(myCobotCpp
    PROPERTIES
        SOVERSION ${PROJECT_VERSION_MAJOR}
        DEBUG_POSTFIX "_d"
        MACOSX_RPATH ON
        WINDOWS_EXPORT_ALL_SYMBOLS ON
)
target_sources(myCobotCpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include/log/Log.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/log/LogReader.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/mycobot/MyCobot.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/mycobot/MyCobotExport.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/Common.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/MyCobot.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/robosignal_global.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/SystemInfo.hpp
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/log/Log.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/log/LogReader.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/mycobot/MyCobot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Common.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Firmata.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Firmata.hpp
        ${CMAKE_CURRENT_LIST_DIR}/src/MyCobot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/SystemInfo.cpp
)
target_include_directories(myCobotCpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_compile_definitions(myCobotCpp
    PRIVATE
        MYCOBOTCPP_DSO
        MYCOBOTCPP_DSO_EXPORTS
        ROBOSIGNAL_LIBRARY
        ROBOT_MYCOBOT
        QT_DISABLE_DEPRECATED_BEFORE=0x060000
        QUERY_STATUS_INTERVAL=600
        COMMAND_INTERVAL=100
)
target_compile_options(myCobotCpp PRIVATE
    -Wall
)
if(NOT(CMAKE_CXX_COMPILER_ID MATCHES MSVC))
    target_compile_options(myCobotCpp PRIVATE
        -Wextra
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wlogical-op
        -Wdouble-promotion
        -Wformat=2
        -Werror=return-type
        -Werror=switch
        -Werror=switch-enum
        -Wundef
        -Wfloat-equal
        -Winit-self
        -Wunreachable-code
        -Wctor-dtor-privacy
        -Werror=shadow
        -Werror=switch-default
        -Werror=switch-unreachable
        -Wmisleading-indentation
        -Wduplicated-cond
        -Wduplicated-branches
        -Wnull-dereference
        -Wno-useless-cast
        -Wno-expansion-to-defined
        -Weffc++
        -fexceptions
    )
endif()
target_link_options(myCobotCpp PRIVATE -s)
target_link_libraries(myCobotCpp PRIVATE
    Qt5::Core
    Qt5::SerialPort
)

install(
    TARGETS
        myCobotCpp
    RUNTIME
        DESTINATION ${INSTALL_BINDIR}
        COMPONENT bin
    LIBRARY
        DESTINATION ${INSTALL_LIBDIR}
        COMPONENT lib
    ARCHIVE
        DESTINATION ${INSTALL_LIBDIR}
        COMPONENT lib
    INCLUDES
        DESTINATION ${INSTALL_INCLUDEDIR}/include/mycobot
    PUBLIC_HEADER
        DESTINATION ${INSTALL_INCLUDEDIR}/mycobot
        COMPONENT dev
    BUNDLE
        DESTINATION ${INSTALL_BINDIR}
)
