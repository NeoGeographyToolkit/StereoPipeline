


# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present

# Convert stereo_utils.py.in to stereo_utils.py with variable substitution.
# - Currently we only insert the version number, but this is the place to
#   insert other CMake variables into Python scripts.
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
set(ASP_VERSION ${PACKAGE_VERSION})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/stereo_utils.py.in 
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo_utils.py)
message("DONE")

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
target_link_libraries(aster2asp aspSessions)
install(TARGETS aster2asp DESTINATION bin)

add_executable(bundle_adjust bundle_adjust.cc bundle_adjust_cost_functions.h) 
target_link_libraries(bundle_adjust aspSessions ${CERES_LIBRARIES})
install(TARGETS bundle_adjust DESTINATION bin)

add_executable(opencv_imagelist_creator opencv_imagelist_creator.cpp) 
target_link_libraries(opencv_imagelist_creator aspCore)
install(TARGETS opencv_imagelist_creator DESTINATION libexec)

add_executable(bundlevis bundlevis.cc)
target_link_libraries(bundlevis ${FULL_LIBRARY_LIST} aspCore ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS bundlevis DESTINATION bin)

add_executable(dem_geoid dem_geoid.cc)
target_link_libraries(dem_geoid aspCore ${GEOID_LIBRARIES})
install(TARGETS dem_geoid DESTINATION bin)


add_executable(camera_footprint camera_footprint.cc)
target_link_libraries(camera_footprint aspSessions)
install(TARGETS camera_footprint DESTINATION bin)


add_executable(disparitydebug disparitydebug.cc)
target_link_libraries(disparitydebug aspCore)
install(TARGETS disparitydebug DESTINATION bin)

add_executable(datum_convert_cpp datum_convert_cpp.cc)
target_link_libraries(datum_convert_cpp aspCore)
install(TARGETS datum_convert_cpp DESTINATION bin)

add_executable(geodiff geodiff.cc)
target_link_libraries(geodiff aspCore)
install(TARGETS geodiff DESTINATION bin)

add_executable(hsv_merge hsv_merge.cc)
target_link_libraries(hsv_merge aspCore)
install(TARGETS hsv_merge DESTINATION bin)

add_executable(lronacjitreg lronacjitreg.cc)
target_link_libraries(lronacjitreg aspIsisIO)
install(TARGETS lronacjitreg DESTINATION bin)

add_executable(mapproject_single mapproject_single.cc)
target_link_libraries(mapproject_single aspSessions)
install(TARGETS mapproject_single DESTINATION bin)

add_executable(mer2camera mer2camera.cc)
target_link_libraries(mer2camera aspCore)
install(TARGETS mer2camera DESTINATION bin)

add_executable(orbitviz orbitviz.cc)
target_link_libraries(orbitviz aspSessions)
install(TARGETS orbitviz DESTINATION bin)

add_executable(pc_align pc_align.cc pc_align_utils.h pc_align_utils.tcc)
target_link_libraries(pc_align aspSessions ${CERES_LIBRARIES} ${LIBPOINTMATCHER_LIBRARIES} )
install(TARGETS pc_align DESTINATION bin)

add_executable(pansharp pansharp.cc)
target_link_libraries(pansharp aspSessions)
install(TARGETS pansharp DESTINATION libexec)

add_executable(point2dem point2dem.cc)
target_link_libraries(point2dem aspCore)
install(TARGETS point2dem DESTINATION bin)

add_executable(point2las point2las.cc)
target_link_libraries(point2las aspCore ${LIBLAS_LIBRARIES}) 
install(TARGETS point2las DESTINATION bin)

add_executable(point2mesh point2mesh.cc)
target_link_libraries(point2mesh aspCore ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS point2mesh DESTINATION bin)

add_executable(add_spot_rpc add_spot_rpc.cc)
target_link_libraries(add_spot_rpc aspCamera)
install(TARGETS add_spot_rpc DESTINATION libexec)

add_executable(cam2rpc cam2rpc.cc)
target_link_libraries(cam2rpc aspSessions)
install(TARGETS cam2rpc DESTINATION libexec)

add_executable(rpc_gen rpc_gen.cc)
target_link_libraries(rpc_gen aspSessions)
install(TARGETS rpc_gen DESTINATION libexec)

add_executable(sfs sfs.cc)
target_link_libraries(sfs aspSessions ${CERES_LIBRARIES})
install(TARGETS sfs DESTINATION bin)

add_executable(stereo_blend stereo_blend.cc stereo.h stereo.cc)
target_link_libraries(stereo_blend aspSessions)
install(TARGETS stereo_blend DESTINATION bin)

add_executable(stereo_corr stereo_corr.cc stereo.h stereo.cc)
target_link_libraries(stereo_corr aspSessions)
install(TARGETS stereo_corr DESTINATION bin)

add_executable(stereo_fltr stereo_fltr.cc stereo.h stereo.cc)
target_link_libraries(stereo_fltr aspSessions)
install(TARGETS stereo_fltr DESTINATION bin)

add_executable(stereo_gui stereo_gui.cc stereo.h stereo.cc)
target_link_libraries(stereo_gui aspSessions aspGUI)
install(TARGETS stereo_gui DESTINATION bin)

add_executable(stereo_parse stereo_parse.cc stereo.h stereo.cc) 
target_link_libraries(stereo_parse aspSessions)
install(TARGETS stereo_parse DESTINATION libexec)

add_executable(stereo_pprc stereo_pprc.cc stereo.h stereo.cc) 
target_link_libraries(stereo_pprc aspSessions)
install(TARGETS stereo_pprc DESTINATION bin)

add_executable(stereo_rfne stereo_rfne.cc stereo.h stereo.cc) 
target_link_libraries(stereo_rfne aspSessions)
install(TARGETS stereo_rfne DESTINATION bin)

add_executable(stereo_tri stereo_tri.cc stereo.h stereo.cc jitter_adjust.cc jitter_adjust.h) 
target_link_libraries(stereo_tri aspSessions ${CERES_LIBRARIES})
install(TARGETS stereo_tri DESTINATION bin)

add_executable(tif_mosaic tif_mosaic.cc) 
target_link_libraries(tif_mosaic aspCore)
install(TARGETS tif_mosaic DESTINATION libexec)

add_executable(wv_correct wv_correct.cc) 
target_link_libraries(wv_correct aspCamera)
install(TARGETS wv_correct DESTINATION bin)


