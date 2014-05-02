 


set(CNTK_LIBRARY_LIST
	${LAPACK_LIBRARIES} 
	${JPEG_LIBRARIES} 
	${GDAL_LIBRARY}
	${PROJ4_LIBRARY}
	${VISIONWORKBENCH_LIBRARIES}	
	${ISIS_LIBRARIES}
    SpCore
    SpIsisIO
#    SpSpiceIO
    SpSessions
	${Boost_LIBRARIES} 
)

# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present

add_executable(cnet_build cnet_build.cc) 
target_link_libraries(cnet_build ${CNTK_LIBRARY_LIST}) 

add_executable(cnet_convert cnet_convert.cc) 
target_link_libraries(cnet_convert ${CNTK_LIBRARY_LIST}) 

add_executable(cnet_merge cnet_merge.cc) 
target_link_libraries(cnet_merge ${CNTK_LIBRARY_LIST}) 

add_executable(reduce_match reduce_match.cc) 
target_link_libraries(reduce_match ${CNTK_LIBRARY_LIST}) 

add_executable(render_gcp render_gcp.cc) 
target_link_libraries(render_gcp ${CNTK_LIBRARY_LIST}) 

# Missing boost algorithms!
#add_executable(vw_position_extract vw_position_extract.cc) 
#target_link_libraries(vw_position_extract ${CNTK_LIBRARY_LIST}) 


