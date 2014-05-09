


# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present


# Install python tools
INSTALL(FILES "cam2map4stereo.py"  DESTINATION bin)
INSTALL(FILES "dg_mosaic"          DESTINATION bin)
INSTALL(FILES "hiedr2mosaic.py"    DESTINATION bin)
INSTALL(FILES "lronac2mosaic.py"   DESTINATION bin)
INSTALL(FILES "parallel_stereo"    DESTINATION bin)
INSTALL(FILES "sparse_disp"        DESTINATION bin)
INSTALL(FILES "stereo"             DESTINATION bin)
INSTALL(FILES "time_trials"        DESTINATION bin)

INSTALL(FILES "stereo_utils.py"    DESTINATION libexec)
INSTALL(FILES "stereo_utils.pyc"   DESTINATION libexec)

# Install common header
INSTALL(FILES "stereo.h" DESTINATION include/asp/Tools)

# Problem calling StereoSession::create(std::string)
#add_executable(bundle_adjust bundle_adjust.cc) 
#target_link_libraries(bundle_adjust ${FULL_LIBRARY_LIST}) 
#install(TARGETS bundle_adjust DESTINATION bin)

add_executable(bundlevis bundlevis.cc) 
target_link_libraries(bundlevis ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS bundlevis DESTINATION bin)

add_executable(dem_geoid dem_geoid.cc) 
target_link_libraries(dem_geoid ${FULL_LIBRARY_LIST}) 
install(TARGETS dem_geoid DESTINATION bin)

# What happened to this one?
#add_executable(dg_mosaic dg_mosaic.cc) 
#target_link_libraries(dg_mosaic ${FULL_LIBRARY_LIST}) 
#install(TARGETS dg_mosaic DESTINATION bin)

add_executable(disparitydebug disparitydebug.cc) 
target_link_libraries(disparitydebug ${FULL_LIBRARY_LIST}) 
install(TARGETS disparitydebug DESTINATION bin)

add_executable(geodiff geodiff.cc) 
target_link_libraries(geodiff ${FULL_LIBRARY_LIST}) 
install(TARGETS geodiff DESTINATION bin)

add_executable(hsv_merge hsv_merge.cc) 
target_link_libraries(hsv_merge ${FULL_LIBRARY_LIST}) 
install(TARGETS hsv_merge DESTINATION bin)

add_executable(isis_adjust isis_adjust.cc) 
target_link_libraries(isis_adjust ${FULL_LIBRARY_LIST}) 
install(TARGETS isis_adjust DESTINATION bin)

add_executable(lronacjitreg lronacjitreg.cc) 
target_link_libraries(lronacjitreg ${FULL_LIBRARY_LIST}) 
install(TARGETS lronacjitreg DESTINATION bin)

add_executable(mapproject mapproject.cc) 
target_link_libraries(mapproject ${FULL_LIBRARY_LIST}) 
install(TARGETS mapproject DESTINATION bin)

add_executable(mer2camera mer2camera.cc) 
target_link_libraries(mer2camera ${FULL_LIBRARY_LIST}) 
install(TARGETS mer2camera DESTINATION bin)

add_executable(orbitviz orbitviz.cc) 
target_link_libraries(orbitviz ${FULL_LIBRARY_LIST}) 
install(TARGETS orbitviz DESTINATION bin)

add_executable(orthoproject orthoproject.cc) 
target_link_libraries(orthoproject ${FULL_LIBRARY_LIST}) 
install(TARGETS orthoproject DESTINATION bin)

add_executable(pc_align pc_align.cc) 
target_link_libraries(pc_align ${FULL_LIBRARY_LIST} ${LIBPOINTMATCHER_LIBRARY}) 
install(TARGETS pc_align DESTINATION bin)

add_executable(point2dem point2dem.cc) 
target_link_libraries(point2dem ${FULL_LIBRARY_LIST}) 
install(TARGETS point2dem DESTINATION bin)

add_executable(point2las point2las.cc) 
target_link_libraries(point2las ${FULL_LIBRARY_LIST} ${LIBLAS_LIBRARY}) 
install(TARGETS point2las DESTINATION bin)

add_executable(point2mesh point2mesh.cc) 
target_link_libraries(point2mesh ${FULL_LIBRARY_LIST} ${OPENSCENEGRAPH_LIBRARIES})
install(TARGETS point2mesh DESTINATION bin)

add_executable(rpc_gen rpc_gen.cc) 
target_link_libraries(rpc_gen ${FULL_LIBRARY_LIST}) 
install(TARGETS rpc_gen DESTINATION libexec)

add_executable(stereo_corr stereo_corr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_corr ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_corr DESTINATION bin)

add_executable(stereo_fltr stereo_fltr.cc stereo.h stereo.cc) 
target_link_libraries(stereo_fltr ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_fltr DESTINATION bin)

add_executable(stereo_parse stereo_parse.cc stereo.h stereo.cc) 
target_link_libraries(stereo_parse ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_parse DESTINATION libexec)

add_executable(stereo_pprc stereo_pprc.cc stereo.h stereo.cc) 
target_link_libraries(stereo_pprc ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_pprc DESTINATION bin)

add_executable(stereo_rfne stereo_rfne.cc stereo.h stereo.cc) 
target_link_libraries(stereo_rfne ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_rfne DESTINATION bin)

add_executable(stereo_tri stereo_tri.cc stereo.h stereo.cc) 
target_link_libraries(stereo_tri ${FULL_LIBRARY_LIST}) 
install(TARGETS stereo_tri DESTINATION bin)

add_executable(tif_mosaic tif_mosaic.cc) 
target_link_libraries(tif_mosaic ${FULL_LIBRARY_LIST}) 
install(TARGETS tif_mosaic DESTINATION libexec)

add_executable(wv_correct wv_correct.cc) 
target_link_libraries(wv_correct ${FULL_LIBRARY_LIST}) 
install(TARGETS wv_correct DESTINATION bin)


