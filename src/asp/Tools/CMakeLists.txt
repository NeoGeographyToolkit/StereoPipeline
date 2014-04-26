
# --- Bayer application --------------------------------------------

include_directories(${Boost_INCLUDE_DIR})

# TODO: Set this earlier for all applications
link_directories(${Boost_LIBRARY_DIR})


add_executable(lronacjitreg      lronacjitreg.cc) 

set(FULL_LIBRARY_LIST
	${LAPACK_LIBRARIES} 
	${JPEG_LIBRARIES} 
	${GDAL_LIBRARY}
	${PROJ4_LIBRARY}
	${VISIONWORKBENCH_LIBRARIES}	
    SpCore
    SpIsisIO
    SpSpiceIO
	${Boost_LIBRARIES} 
)

#message(${FULL_LIBRARY_LIST})

target_link_libraries(lronacjitreg     ${FULL_LIBRARY_LIST}) 









