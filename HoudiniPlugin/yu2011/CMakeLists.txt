cmake_minimum_required(VERSION 2.8)

#include(RezBuild)

project (Yu2011)

set(ProjectName Yu2011)

set(ProjectSource .)
set(Set Set)
set(Math  Math)
set(Eigen eigen)
set(Images Images)

set(HoudiniPlugins HoudiniPlugins)
project(${ProjectName})

set(module_path ${PROJECT_SOURCE_DIR}/../cmake/)
MESSAGE("--------------------SurfaceTextureSynthesis----------------------")
MESSAGE("module_path" ${module_path})
MESSAGE("CMAKE BINARY DIR" ${CMAKE_BINARY_DIR})
set(CMAKE_MODULE_PATH ${module_path})
set(HDKCONFIG "${PROJECT_SOURCE_DIR}/../hdk-config.py")


MESSAGE("------------testing---------------")
############# FIND PACKAGE ###############
find_package(HDK)
find_package(OpenCV REQUIRED core highgui)     # OpenCV root path must be in CMAKE_PREFIX_PATH environment variable.
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(HDK)


############## HOUDINI ###################

# run our helper process to grab the compile and link flags from hcustom
execute_process(COMMAND ${HDKCONFIG} --notag --cmake=${CMAKE_BINARY_DIR}/FindHDK.cmake)

MESSAGE("HDK=")
#MESSAGE(${HDK})
MESSAGE(${HDK_INCLUDE_DIRS})

# now get the HDK definitions from the FindHDK.cmake module that was just generated
set(CMAKE_MODULE_PATH "${PROJECT_BINARY_DIR}")


################# SOURCE ##################

file(GLOB source ${ProjectSource}/*.h ${ProjectSource}/*.C ${ProjectSource}/*.cpp ${ProjectSource}/*.hpp ${ProjectSource}/Set/*.h
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
)

set(CMAKE_INSTALL_PREFIX "${HDK_HIH_DIR}")


# setup to compile in the correct HDK environment
add_definitions(${HDK_DEFINITIONS})


########### INCLUDES #################

include_directories(${HDK_INCLUDE_DIRS})
include_directories(${ProjectSource})
include_directories(${Math})
include_directories(${Set})
include_directories(${Eigen})
include_directories(${Images})
include_directories(${HoudiniPlugins})



########## LINKS ######################

# Default to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

link_directories(${HDK_LIBRARY_DIRS})


# this command calls hdk-config.py to build a special file that will be the repository of the taginfo
add_custom_command(OUTPUT sesitag.C
                   DEPENDS ${source}
                   COMMAND ${HDKCONFIG} --tagfile=${CMAKE_CURRENT_BINARY_DIR}/sesitag.C)

if (APPLE)
    set(LTYPE MODULE)
else (APPLE)
    set(LTYPE SHARED)
endif (APPLE)


#External Libraries

# this tells cmake to buid the DSO

add_library(${ProjectName} ${LTYPE} ${source}  sesitag.C)
# houdini doesn't want the DSO to have "lib" as it's prefix
set_target_properties(${ProjectName} PROPERTIES PREFIX "")

# unfortunately cmake's MODULE library type generates a ".so" file on OSX
if (APPLE)
    set_target_properties(${ProjectName} PROPERTIES SUFFIX ".dylib")
endif (APPLE)

#LINK_DIRECTORIES(/prod/tools/rd/libx264/lib)
add_library(libx264 SHARED IMPORTED)
set_target_properties( libx264 PROPERTIES IMPORTED_LOCATION /prod/tools/rd/libx264/lib/libx264.so )

add_library(image SHARED IMPORTED)
#set_target_properties( image PROPERTIES IMPORTED_LOCATION /prod/tools/rez/packages/int/rd_image/1.2.0/./libImages.so )
set_target_properties( image PROPERTIES IMPORTED_LOCATION /prod/tools/rez/packages/int/rd_image/1.1.0/./libImages.so )
#set(IMAGESLAB /home/jgagnon/packages/rd_image/1.2.0/libImages.so)
#set_target_properties( image PROPERTIES IMPORTED_LOCATION "${IMAGESLAB}" )


#target_link_libraries (${ProjectName} ${FluidsCore})
MESSAGE("-------------------- Target_link_libraries --------------")
target_link_libraries(${ProjectName}
               ${HDK_LIBRARIES}
                image
               ${OpenCV_LIBS}
               libx264
)

# finally tell cmake where to put the results
install(TARGETS ${ProjectName} LIBRARY DESTINATION dso)
install(FILES ${ProjectName}.png DESTINATION config/Icons)
MESSAGE("-------------------- Installation done --------------")

