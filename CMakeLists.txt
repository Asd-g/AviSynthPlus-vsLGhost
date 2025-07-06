cmake_minimum_required(VERSION 3.28)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type" FORCE)
endif()

project(vsLGhost VERSION 1.0.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(AvisynthPlus REQUIRED)

add_library(${PROJECT_NAME} MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost_AVX2.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost_AVX512.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost_SSE2.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME} PRIVATE AvisynthPlus::headers)

if(CMAKE_CXX_COMPILER_ID STREQUAL "IntelLLVM")
    target_link_libraries(${PROJECT_NAME} PRIVATE "libmmds")
endif()

set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost_AVX2.cpp" PROPERTIES COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:MSVC>:/arch:AVX2>$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:mavx2;-mfma>")
set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/src/vsLGhost_AVX512.cpp" PROPERTIES COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:MSVC>:/arch:AVX512>$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:mavx512f;-mavx512bw;-mavx512dq;-mavx512vl;-mavx512cd;-mfma>")

if(CMAKE_CXX_COMPILER_ID STREQUAL "IntelLLVM")
    target_compile_options(${PROJECT_NAME} PRIVATE "/fp:precise")
endif()

if(MINGW)
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
elseif(UNIX)
    string(TOLOWER "${PROJECT_NAME}" FILE_NAME_OUTPUT)
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${FILE_NAME_OUTPUT}.${PROJECT_VERSION}")
endif()

if (WIN32)
    string(REGEX MATCH "^([0-9.]+)" CORE_VERSION_STRING "${PROJECT_VERSION}")
    if(NOT CORE_VERSION_STRING)
        message(WARNING "Could not parse core version from '${PROJECT_VERSION}'. Defaulting to 0.0.0.")
        set(CORE_VERSION_STRING "0.0.0")
    endif()

    string(REPLACE "." ";" VERSION_NUMERIC_PARTS_LIST "${CORE_VERSION_STRING}")
    list(LENGTH VERSION_NUMERIC_PARTS_LIST NUM_PARTS)

    set(VERSION_MAJOR 0)
    set(VERSION_MINOR 0)
    set(VERSION_PATCH 0)
    set(VERSION_BUILD 0)

    if(NUM_PARTS GREATER_EQUAL 1)
        list(GET VERSION_NUMERIC_PARTS_LIST 0 VERSION_MAJOR)
    endif()

    if(NUM_PARTS GREATER_EQUAL 2)
        list(GET VERSION_NUMERIC_PARTS_LIST 1 VERSION_MINOR)
    endif()

    if(NUM_PARTS GREATER_EQUAL 3)
        list(GET VERSION_NUMERIC_PARTS_LIST 2 VERSION_PATCH)
    endif()

    if(NUM_PARTS GREATER_EQUAL 4)
        list(GET VERSION_NUMERIC_PARTS_LIST 3 VERSION_BUILD)
    endif()

    if(PROJECT_VERSION MATCHES "-")
        set(PROJECT_IS_PRERELEASE 1)
    else()
        set(PROJECT_IS_PRERELEASE 0)
    endif()

    if(NOT CMAKE_CONFIGURATION_TYPES)
        if(CMAKE_BUILD_TYPE MATCHES "^Debug$")
            set(PROJECT_DEBUG_BUILD_VALUE 1)
        else()
            set(PROJECT_DEBUG_BUILD_VALUE 0)
        endif()
    else()
        set(PROJECT_DEBUG_BUILD_VALUE -1)
    endif()

    set(PROJECT_DEBUG_BUILD ${PROJECT_DEBUG_BUILD_VALUE})

    set(FILE_DESCRIPTION "Luminance Ghost reduction filter")
    set(INTERNAL_NAME "${PROJECT_NAME}")
    set(ORIGINAL_FILENAME "${PROJECT_NAME}.dll")
    set(PRODUCT_NAME "${PROJECT_NAME}")
    set(PROJECT_VERSION_STRING_FULL "${PROJECT_VERSION}")

    set(RC_FILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.rc") # Output to build dir

    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.rc.in"
        "${RC_FILE_OUT}"
        @ONLY
    )

    target_sources(${PROJECT_NAME} PRIVATE "${RC_FILE_OUT}")

    if(MSVC)
        set_source_files_properties("${RC_FILE_OUT}" PROPERTIES
            VS_RESOURCE_GENERATOR "RC")
    endif()
endif()

if(UNIX)
    include(GNUInstallDirs)

    INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/avisynth")

    # uninstall target
    if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
    endif()
endif()
