# NOTE Change the project name and edit the list of source files to match your project.
# For ANN, you have two options. You can:
#  - install the libann-dev package
#  - download ANN (http://www.cs.umd.edu/~mount/ANN/),
#    extract in the project directory and compile it,
#    then set ANN_DIR and register the library (uncomment the lines below),
#    and change 'ann' to 'ANN' in target_link_libraries
#  - set ANN_DIR to another directory containing the compiled ANN library,
#    (for example /prod/tools/rd/ann1.1.2), and also
#    register the include/lib directories and change target_link_libraries

# Specify the minimum required version of CMake to build the project
cmake_minimum_required( VERSION 3.0 )

# Set the project name
project( Gagnon2019 )
set(ProjectSource .)
# Set the path to the ANN library (uncomment if you are not using the package)
#set( ANN_DIR "${CMAKE_CURRENT_LIST_DIR}/ann_1.1.2" )
#set( ANN_DIR "/prod/tools/rd/ann1.1.2" )

############################
# Initialize Houdini (15+) #
############################

if( DEFINED ENV{HFS} )
        # Display the HFS environment variable
        message( STATUS "HFS directory: " $ENV{HFS} )

        # Locate Houdini's libraries and header files
        list( APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake" )
else()
    message( STATUS "HFS not set, looking into CMAKE_PREFIX_PATH: " ${CMAKE_PREFIX_PATH} )
    if( NOT CMAKE_PREFIX_PATH )
        message( FATAL_ERROR "Environment variable HFS undefined.
            Please set CMAKE_PREFIX_PATH to HoudiniConfig.cmake path ({HFS}/toolkit/cmake)." )
    endif()
endif()

# Register an imported library target named 'Houdini'
# (this will use the HoudiniConfig.cmake file)
# Note: if the environment variable HFS is not defined,
# use -DCMAKE_PREFIX_PATH=/path/to/hfs/toolkit/cmake
# (or any path to HoudiniConfig.cmake)
find_package( Houdini REQUIRED )

message( STATUS "Found Houdini " ${Houdini_VERSION_MAJOR} "." ${Houdini_VERSION_MINOR} "." ${Houdini_VERSION_PATCH} )

# We need version 15 minimum
if( Houdini_VERSION_MAJOR LESS 15 )
    message( FATAL_ERROR "Houdini 15+ required." )
endif()

##########################
# Initialize the sources #
##########################

# Set the source path
set( SRC_DIR src )

# List the source files
# (except the ones with the Houdini operators)
file(GLOB SOURCES ${ProjectSource}/*.h ${ProjectSource}/*.C ${ProjectSource}/*.cpp ${ProjectSource}/*.hpp ${ProjectSource}/Set/*.h
            ${ProjectSource}/Set/SpacialGrid.hpp
            ${ProjectSource}/HoudiniPlugins/*
            ${ProjectSource}/HoudiniInterfaces/*
            ${ProjectSource}/OldApproach/*
            ${ProjectSource}/Strategies/*.[hHcC]*
            ${ProjectSource}/Strategies/TextureSynthesis/*
            ${ProjectSource}/Strategies/SurfaceTextureSynthesis/*
            ${ProjectSource}/Strategies/StrategyPatchSurfaceSynthesis/*
            ${ProjectSource}/Core/*.[hHcC]*
            ${ProjectSource}/Core/Deformations/*
            ${ProjectSource}/Core/Atlas/*
            ${ProjectSource}/PoissonDisk/*
            ${ProjectSource}/Images/*
            ${ProjectSource}/Math/*
)

include_directories(${ProjectSource})
include_directories(${Math})
include_directories(${Set})
include_directories(${Images})
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

# If Houdini 16+
if( NOT (Houdini_VERSION_MAJOR LESS 16) )
    # Code generation for the embedded DS files in all node class definitions
    houdini_generate_proto_headers( FILES ${PROTO_SOURCES} )
endif()

# Use all the source files
add_library( ${library_name} SHARED ${SOURCES} )

# Link against the libraries,
# and add required include directories and compile definitions
target_link_libraries( ${library_name}
    Houdini
    ${OpenCV_LIBS}
)

# If Houdini 16+
if( NOT (Houdini_VERSION_MAJOR LESS 16) )
    # Include ${CMAKE_CURRENT_BINARY_DIR} for the generated headers
    target_include_directories( ${library_name} PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()

# Configure several common target properties
# We set the output directory (INSTDIR) in the context of a Rez package,
# otherwise we leave the default (Houdini's default dso directory)
houdini_configure_target( ${library_name} )
#houdini_configure_target( ${library_name} INSTDIR ${CMAKE_CURRENT_LIST_DIR}/dso )
