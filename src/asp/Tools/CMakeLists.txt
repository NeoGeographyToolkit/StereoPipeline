
# TODO: Don't just use this for all tools
set(FULL_LIBRARY_LIST
	${LAPACK_LIBRARIES} 
	${JPEG_LIBRARIES} 
	${GDAL_LIBRARY}
	${PROJ4_LIBRARY}
	${VISIONWORKBENCH_LIBRARIES}	
	${ISIS_LIBRARIES}
    SpCore
    SpIsisIO
    SpSpiceIO
    SpSessions
	${Boost_LIBRARIES} 
)



# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present

# Problem calling StereoSession::create(std::string)
#add_executable(bundle_adjust bundle_adjust.cc) 
#target_link_libraries(bundle_adjust ${FULL_LIBRARY_LIST}) 

add_executable(bundlevis bundlevis.cc) 
target_link_libraries(bundlevis ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})

add_executable(dem_geoid dem_geoid.cc) 
target_link_libraries(dem_geoid ${FULL_LIBRARY_LIST}) 

# What happened to this one?
#add_executable(dg_mosaic dg_mosaic.cc) 
#target_link_libraries(dg_mosaic ${FULL_LIBRARY_LIST}) 

add_executable(disparitydebug disparitydebug.cc) 
target_link_libraries(disparitydebug ${FULL_LIBRARY_LIST}) 

add_executable(geodiff geodiff.cc) 
target_link_libraries(geodiff ${FULL_LIBRARY_LIST}) 

add_executable(hsv_merge hsv_merge.cc) 
target_link_libraries(hsv_merge ${FULL_LIBRARY_LIST}) 

add_executable(isis_adjust isis_adjust.cc) 
target_link_libraries(isis_adjust ${FULL_LIBRARY_LIST}) 

add_executable(lronacjitreg lronacjitreg.cc) 
target_link_libraries(lronacjitreg ${FULL_LIBRARY_LIST}) 

add_executable(mapproject mapproject.cc) 
target_link_libraries(mapproject ${FULL_LIBRARY_LIST}) 

add_executable(mer2camera mer2camera.cc) 
target_link_libraries(mer2camera ${FULL_LIBRARY_LIST}) 

add_executable(orbitviz orbitviz.cc) 
target_link_libraries(orbitviz ${FULL_LIBRARY_LIST}) 

add_executable(orthoproject orthoproject.cc) 
target_link_libraries(orthoproject ${FULL_LIBRARY_LIST}) 

add_executable(pc_align pc_align.cc) 
target_link_libraries(pc_align ${FULL_LIBRARY_LIST} ${LIBPOINTMATCHER_LIBRARY}) 

add_executable(point2dem point2dem.cc) 
target_link_libraries(point2dem ${FULL_LIBRARY_LIST}) 

add_executable(point2las point2las.cc) 
target_link_libraries(point2las ${FULL_LIBRARY_LIST} ${LIBLAS_LIBRARY}) 

add_executable(point2mesh point2mesh.cc) 
target_link_libraries(point2mesh ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})

add_executable(rpc_gen rpc_gen.cc) 
target_link_libraries(rpc_gen ${FULL_LIBRARY_LIST}) 

add_executable(stereo_corr stereo_corr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_corr ${FULL_LIBRARY_LIST}) 

add_executable(stereo_fltr stereo_fltr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_fltr ${FULL_LIBRARY_LIST}) 

add_executable(stereo_parse stereo_parse.cc stereo.h stereo.cc) 
target_link_libraries(stereo_parse ${FULL_LIBRARY_LIST}) 

add_executable(stereo_pprc stereo_pprc.cc stereo.h stereo.cc) 
target_link_libraries(stereo_pprc ${FULL_LIBRARY_LIST}) 

add_executable(stereo_rfne stereo_rfne.cc stereo.h stereo.cc) 
target_link_libraries(stereo_rfne ${FULL_LIBRARY_LIST}) 

add_executable(stereo_tri stereo_tri.cc stereo.h stereo.cc) 
target_link_libraries(stereo_tri ${FULL_LIBRARY_LIST}) 

add_executable(tif_mosaic tif_mosaic.cc) 
target_link_libraries(tif_mosaic ${FULL_LIBRARY_LIST}) 





