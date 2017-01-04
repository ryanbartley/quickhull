if( NOT TARGET quickhull )
	get_filename_component( quickhull_SOURCE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../src" ABSOLUTE )
	get_filename_component( CINDER_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE )

	add_library( quickhull ${quickhull_SOURCE_PATH}/QuickHull.cpp )

	target_include_directories( quickhull PUBLIC "${OSC_SOURCE_PATH}" )
	
endif()



