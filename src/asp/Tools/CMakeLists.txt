


# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present



# Install all of the python files.
set(PYTHON_TOOLS cam2map4stereo.py    hiedr2mosaic.py
                 lronac2mosaic.py     merge_orbitviz.py
                 dg_mosaic            parallel_stereo
                 sparse_disp          stereo
                 time_trials          camera_calibrate
                 camera_solve         parallel_sfs
                 mapproject)
set(PYTHON_LIBS crs2crs2grid.py stereo_utils.py)

foreach(p ${PYTHON_TOOLS})
  INSTALL(FILES ${p}  DESTINATION bin)
endforeach()
foreach(p ${PYTHON_LIBS})
  INSTALL(FILES ${p}  DESTINATION libexec)
endforeach()

# Does anything use this header?
## Install common header
#INSTALL(FILES "stereo.h" DESTINATION include/asp/Tools)

add_executable(aster2asp aster2asp.cc) 
target_link_libraries(aster2asp ${FULL_LIBRARY_LIST}) 
install(TARGETS aster2asp DESTINATION bin)

add_executable(bundle_adjust bundle_adjust.cc bundle_adjust_cost_functions.h) 
target_link_libraries(bundle_adjust ${FULL_LIBRARY_LIST}) 
install(TARGETS bundle_adjust DESTINATION bin)

add_executable(opencv_imagelist_creator opencv_imagelist_creator.cpp) 
target_link_libraries(opencv_imagelist_creator ${FULL_LIBRARY_LIST}) 
install(TARGETS opencv_imagelist_creator DESTINATION libexec)

add_executable(bundlevis bundlevis.cc) 
target_link_libraries(bundlevis ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS bundlevis DESTINATION bin)

add_executable(dem_geoid dem_geoid.cc) 
target_link_libraries(dem_geoid ${FULL_LIBRARY_LIST}) 
install(TARGETS dem_geoid DESTINATION bin)


add_executable(camera_footprint camera_footprint.cc) 
target_link_libraries(camera_footprint ${FULL_LIBRARY_LIST}) 
install(TARGETS camera_footprint DESTINATION bin)


add_executable(disparitydebug disparitydebug.cc) 
target_link_libraries(disparitydebug ${FULL_LIBRARY_LIST}) 
install(TARGETS disparitydebug DESTINATION bin)

add_executable(datum_convert_cpp datum_convert_cpp.cc) 
target_link_libraries(datum_convert_cpp ${FULL_LIBRARY_LIST}) 
install(TARGETS datum_convert_cpp DESTINATION bin)

add_executable(geodiff geodiff.cc) 
target_link_libraries(geodiff ${FULL_LIBRARY_LIST}) 
install(TARGETS geodiff DESTINATION bin)

add_executable(hsv_merge hsv_merge.cc) 
target_link_libraries(hsv_merge ${FULL_LIBRARY_LIST}) 
install(TARGETS hsv_merge DESTINATION bin)

add_executable(lronacjitreg lronacjitreg.cc) 
target_link_libraries(lronacjitreg ${FULL_LIBRARY_LIST}) 
install(TARGETS lronacjitreg DESTINATION bin)

add_executable(mapproject_single mapproject_single.cc) 
target_link_libraries(mapproject_single ${FULL_LIBRARY_LIST}) 
install(TARGETS mapproject_single DESTINATION bin)

add_executable(mer2camera mer2camera.cc) 
target_link_libraries(mer2camera ${FULL_LIBRARY_LIST}) 
install(TARGETS mer2camera DESTINATION bin)

add_executable(orbitviz orbitviz.cc) 
target_link_libraries(orbitviz ${FULL_LIBRARY_LIST}) 
install(TARGETS orbitviz DESTINATION bin)

add_executable(pc_align pc_align.cc pc_align_utils.h pc_align_utils.tcc) 
target_link_libraries(pc_align ${FULL_LIBRARY_LIST} ${LIBPOINTMATCHER_LIBRARY}) 
install(TARGETS pc_align DESTINATION bin)

add_executable(pansharp pansharp.cc) 
target_link_libraries(pansharp ${FULL_LIBRARY_LIST}) 
install(TARGETS pansharp DESTINATION libexec)

add_executable(point2dem point2dem.cc) 
target_link_libraries(point2dem ${FULL_LIBRARY_LIST}) 
install(TARGETS point2dem DESTINATION bin)

add_executable(point2las point2las.cc) 
target_link_libraries(point2las ${FULL_LIBRARY_LIST} ${LIBLAS_LIBRARY}) 
install(TARGETS point2las DESTINATION bin)

add_executable(point2mesh point2mesh.cc) 
target_link_libraries(point2mesh ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS point2mesh DESTINATION bin)

add_executable(add_spot_rpc add_spot_rpc.cc) 
target_link_libraries(add_spot_rpc ${FULL_LIBRARY_LIST}) 
install(TARGETS add_spot_rpc DESTINATION libexec)

add_executable(cam2rpc cam2rpc.cc) 
target_link_libraries(cam2rpc ${FULL_LIBRARY_LIST}) 
install(TARGETS cam2rpc DESTINATION libexec)

add_executable(rpc_gen rpc_gen.cc) 
target_link_libraries(rpc_gen ${FULL_LIBRARY_LIST}) 
install(TARGETS rpc_gen DESTINATION libexec)

add_executable(sfs sfs.cc) 
target_link_libraries(sfs ${FULL_LIBRARY_LIST} ${LIBLAS_LIBRARY}) 
install(TARGETS sfs DESTINATION bin)

add_executable(stereo_blend stereo_blend.cc stereo.h stereo.cc) 
target_link_libraries(stereo_blend ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_blend DESTINATION bin)

add_executable(stereo_corr stereo_corr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_corr ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_corr DESTINATION bin)

add_executable(stereo_fltr stereo_fltr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_fltr ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_fltr DESTINATION bin)

add_executable(stereo_gui stereo_gui.cc stereo.h stereo.cc) 
target_link_libraries(stereo_gui ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_gui DESTINATION bin)

add_executable(stereo_parse stereo_parse.cc stereo.h stereo.cc) 
target_link_libraries(stereo_parse ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_parse DESTINATION libexec)

add_executable(stereo_pprc stereo_pprc.cc stereo.h stereo.cc) 
target_link_libraries(stereo_pprc ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_pprc DESTINATION bin)

add_executable(stereo_rfne stereo_rfne.cc stereo.h stereo.cc) 
target_link_libraries(stereo_rfne ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_rfne DESTINATION bin)

add_executable(stereo_tri stereo_tri.cc stereo.h stereo.cc jitter_adjust.cc jitter_adjust.h) 
target_link_libraries(stereo_tri ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_tri DESTINATION bin)

add_executable(tif_mosaic tif_mosaic.cc) 
target_link_libraries(tif_mosaic ${FULL_LIBRARY_LIST}) 
install(TARGETS tif_mosaic DESTINATION libexec)

add_executable(wv_correct wv_correct.cc) 
target_link_libraries(wv_correct ${FULL_LIBRARY_LIST}) 
install(TARGETS wv_correct DESTINATION bin)


