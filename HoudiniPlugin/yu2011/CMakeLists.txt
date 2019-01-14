cmake_minimum_required(VERSION 2.8)

#include(RezBuild)

project (Yu2011)

set(ProjectName Yu2011)

set(ProjectSource .)
set(Set Set)
set(Math  Math)
set(Eigen eigen)
set(Images Images)
#set(ANN  "/prod/tools/rd/ann1.1.2/include/ANN")
set(HoudiniPlugins HoudiniPlugins)
project(${ProjectName})
set(HDK "/prod/software/sidefx/hfs17.0.416")
#set(HDK "/prod/software/sidefx/hfs16.5.439")
#set(HDK "opt/hfs14.0.590")
#set(HDK "/opt/hfs15.5.565")
set(module_path ${PROJECT_SOURCE_DIR}/../cmake/)
MESSAGE("--------------------SurfaceTextureSynthesis----------------------")
MESSAGE("module_path" ${module_path})
MESSAGE("CMAKE BINARY DIR" ${CMAKE_BINARY_DIR})
set(CMAKE_MODULE_PATH ${module_path})
set(HDKCONFIG "${PROJECT_SOURCE_DIR}/../hdk-config.py")


#target_link_libraries(RD_SurfaceTextureSynthesis CoreLib)
MESSAGE("------------testing---------------")
############# FIND PACKAGE ###############
find_package(HDK)
#find_package(OpenCV)
find_package(OpenCV REQUIRED core highgui)     # OpenCV root path must be in CMAKE_PREFIX_PATH environment variable.
#find_package(OpenCV REQUIRED core)     # OpenCV root path must be in CMAKE_PREFIX_PATH environment variable.
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    add_definitions(-D_GLIBCXX_PARALLEL) # Currently multiplying compute time by three
endif()

find_package(HDK)
find_package(ANN)

############## HOUDINI ###################

# run our helper process to grab the compile and link flags from hcustom
execute_process(COMMAND ${HDKCONFIG} --notag --cmake=${CMAKE_BINARY_DIR}/FindHDK.cmake)

MESSAGE("HDK=")
MESSAGE(${HDK})
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
#include_directories(${ProjectSource}/../WaterComputation/)
include_directories(${Eigen})
include_directories(${Images})
include_directories(${HoudiniPlugins})
include_directories(${ANN_INCLUDE_DIR})

#ADD_SUBDIRECTORY(${Images} "${ProjectSource}/Images")
#ADD_SUBDIRECTORY(${Images} "${CMAKE_CURRENT_BINARY_DIR}/src/Images")
#ADD_SUBDIRECTORY(${Math} "${CMAKE_CURRENT_BINARY_DIR}/src/Math")


#configure_file( version.h ${${ProjectSource}}/version.h )
#include_directories( ${${ProjectSource}}/generated/ ) # Make sure it can be included...


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


add_library(image SHARED IMPORTED)
#set_target_properties( image PROPERTIES IMPORTED_LOCATION /prod/tools/rez/packages/int/rd_image/1.1.0/libImages.so )
set_target_properties( image PROPERTIES IMPORTED_LOCATION /home/jgagnon/packages/rd_image/1.1.1/./libImages.so )


#target_link_libraries (${ProjectName} ${FluidsCore})
MESSAGE("-------------------- Target_link_libraries --------------")
target_link_libraries(${ProjectName}
	       ${HDK_LIBRARIES}
          image
	       ${OpenCV_LIBS}
         #${ANN_LIBRARIES}
)

# finally tell cmake where to put the results
install(TARGETS ${ProjectName} LIBRARY DESTINATION dso)
install(FILES ${ProjectName}.png DESTINATION config/Icons)
MESSAGE("-------------------- Installation done --------------")
#rez_install_files(${ProjectName}.so DESTINATION ${CMAKE_BINARY_DIR})
