# NOTE Change the project name and edit the list of source files to match your project.

# Specify the minimum required version of CMake to build the project
cmake_minimum_required( VERSION 3.0 )

# Set the project name
project( Gagnon2019 )

############################
# Initialize Houdini (16+) #
############################

if( DEFINED ENV{HFS} )
        # Display the HFS environment variable
        message( STATUS "HFS directory: " $ENV{HFS} )

        # Locate Houdini's libraries and header files
        list( APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake" )
else()
    # If the environment variable HFS is not defined,
    # use -DCMAKE_PREFIX_PATH=/path/to/hfs/toolkit/cmake
    # (or any path to HoudiniConfig.cmake)
    message( STATUS "HFS not set, looking into CMAKE_PREFIX_PATH: " ${CMAKE_PREFIX_PATH} )
    if( NOT CMAKE_PREFIX_PATH )
        message( FATAL_ERROR "Environment variable HFS undefined.
            Please make sure the HFS environment variable is set or
            set CMAKE_PREFIX_PATH to HoudiniConfig.cmake path ({HFS}/toolkit/cmake)." )
    endif()
endif()

# Register an imported library target named 'Houdini'
# (this will use the HoudiniConfig.cmake file)
find_package( Houdini REQUIRED )

message( STATUS "Found Houdini " ${Houdini_VERSION_MAJOR} "." ${Houdini_VERSION_MINOR} "." ${Houdini_VERSION_PATCH} )

# We need version 16 minimum
if( Houdini_VERSION_MAJOR LESS 16 )
    message( FATAL_ERROR "Houdini 16+ required." )
endif()

##########################
# Initialize the sources #
##########################

# Set the source path
set( SRC_DIR . )

# List the source files
# (except the ones with the Houdini operators)
# NOTE The use of GLOB is not recommended
file( GLOB SOURCES
    ${SRC_DIR}/*.h
    ${SRC_DIR}/*.C
    ${SRC_DIR}/*.cpp
    ${SRC_DIR}/*.hpp
    ${SRC_DIR}/Set/*.h
    ${SRC_DIR}/Set/SpacialGrid.hpp
    ${SRC_DIR}/HoudiniPlugins/*
    ${SRC_DIR}/HoudiniInterfaces/*
    ${SRC_DIR}/OldApproach/*
    ${SRC_DIR}/Strategies/*.[hHcC]*
    ${SRC_DIR}/Strategies/TextureSynthesis/*
    ${SRC_DIR}/Strategies/SurfaceTextureSynthesis/*
    ${SRC_DIR}/Strategies/StrategyPatchSurfaceSynthesis/*
    ${SRC_DIR}/Core/*.[hHcC]*
    ${SRC_DIR}/Core/Deformations/*
    ${SRC_DIR}/Core/Atlas/*
    ${SRC_DIR}/PoissonDisk/*
    ${SRC_DIR}/Images/*
    ${SRC_DIR}/Math/*
)

# TODO List the files containing DS definitions (*.C)
# These will generate header files
set( PROTO_SOURCES
#    ${SRC_DIR}/*.C
)

##########################
# Register the libraries #
##########################

# Register OpenCV

# In Windows, the OPENCV_DIR environment variable should have been set
# (see https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html)
# and can be used to find OpenCVConfig.cmake
if( DEFINED ENV{OPENCV_DIR} )
    message( STATUS "OpenCV directory: " $ENV{OPENCV_DIR} )
    list( APPEND CMAKE_PREFIX_PATH "$ENV{OPENCV_DIR}/lib" )
endif()
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add a library with the same name as the project
set( library_name ${PROJECT_NAME} )

# Code generation for the embedded DS files in all node class definitions
houdini_generate_proto_headers( FILES ${PROTO_SOURCES} )

# Use all the source files
add_library( ${library_name} SHARED ${SOURCES} )

# Link against the libraries,
# and add required include directories and compile definitions
target_link_libraries( ${library_name}
    Houdini
    ${OpenCV_LIBS}
)

# Include ${CMAKE_CURRENT_BINARY_DIR} for the generated headers
# and other dependencies
target_include_directories( ${library_name} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${SRC_DIR}
    ${Math}
    ${Set}
    ${Images}
)

# Configure several common target properties
# We leave the default (Houdini's default dso directory)
houdini_configure_target( ${library_name} )
# Otherwise, we can set the output directory (INSTDIR) manually
#houdini_configure_target( ${library_name} INSTDIR ${CMAKE_CURRENT_LIST_DIR}/dso )
