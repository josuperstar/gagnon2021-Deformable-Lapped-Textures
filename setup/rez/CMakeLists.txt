cmake_minimum_required(VERSION 2.8)

include(RezBuild)



MESSAGE('=========================== RD YU2011 ===================================')

#============================= DSO =============================
set(directory ${PROJECT_BINARY_DIR}/dso/)
set(source_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../HoudiniPlugin/build-yu2011-Desktop-Default)
set(dso_file "Yu2011.so")


MESSAGE('make directory '+"${directory}")
file(MAKE_DIRECTORY "${directory}")

MESSAGE('copy ')
MESSAGE(${source_dir}/${dso_file})
MESSAGE('to')
MESSAGE(${directory}/${dso_file})

execute_process(COMMAND "cp" "-R" "${source_dir}/${dso_file}" "${directory}/${dso_file}")

file(GLOB_RECURSE plugin_files "${directory}/*.so" )

rez_install_files(
	${plugin_files}
	RELATIVE "build/houdini-17/"
	DESTINATION .
)


#============================= OTL =============================
set(directory_otl ${PROJECT_BINARY_DIR}/otl/)
set(source_dir_otl ${CMAKE_CURRENT_SOURCE_DIR}/../../HoudiniPlugin/otl)
set(otl_file "yu2011.hda")

MESSAGE('make directory '+"${directory_otl}")
file(MAKE_DIRECTORY "${directory_otl}")

MESSAGE('copy ')
MESSAGE(${source_dir_otl}/${otl_file})
MESSAGE('to')
MESSAGE(${PROJECT_BINARY_DIR}/dso/${otl_file})
execute_process(COMMAND "cp" "-R" "${source_dir_otl}/${otl_file}" "${directory_otl}/${otl_file}")

file(GLOB_RECURSE plugin_files "${directory_otl}/*.hda" )

rez_install_files(
	${plugin_files}
	RELATIVE "build/houdini-17/"
	DESTINATION .
)