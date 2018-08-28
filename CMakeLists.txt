cmake_minimum_required(VERSION 2.8)

include(RezBuild)



MESSAGE('=========================== RD YU2011 ===================================')

#============================= DSO =============================
set(directory ${PROJECT_BINARY_DIR}/dso/)
set(source_dir ${CMAKE_CURRENT_SOURCE_DIR}/HoudiniPlugin/build-yu2011-Desktop-Default)
set(dso_file "Yu2011.so")


MESSAGE('make directory '+"${directory}")
file(MAKE_DIRECTORY "${directory}")

MESSAGE('copy ')
MESSAGE(${source_dir}/${dso_file})
MESSAGE('to')
MESSAGE(${PROJECT_BINARY_DIR}/dso/${dso_file})

execute_process(COMMAND "cp" "-R" "${source_dir}/${dso_file}" "${PROJECT_BINARY_DIR}/dso/${dso_file}")

file(GLOB_RECURSE plugin_files "${PROJECT_BINARY_DIR}/dso/*.so" )

rez_install_files(
	${plugin_files}
	RELATIVE "build/houdini-16/"
	DESTINATION .
)


