# ---------------------------------------------------------------
# Define each of the tools to build

#TODO: Turn them off when components are not present

# Install all of the python files.
# Note: When packaged, the files with a .py extension will
# not get a shell script wrapper. See add_executable() 
# in BinaryDist.py.
set(PYTHON_TOOLS cam2map4stereo.py    hiedr2mosaic.py
                 lronac2mosaic.py     parse_match_file.py
                 dg_mosaic            parallel_stereo
                 sparse_disp          stereo
                 time_trials          camera_calibrate
                 camera_solve         parallel_sfs
                 mapproject           parallel_bundle_adjust
                 historical_helper.py datum_convert
                 bathy_threshold_calc.py 
                 scale_bathy_mask.py
                 crs2crs2grid.py 
                 orbit_plot.py)
set(PYTHON_LIBS stereo_utils.py)

foreach(p ${PYTHON_TOOLS})
  INSTALL(FILES ${p} PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
    DESTINATION bin)
endforeach()
foreach(p ${PYTHON_LIBS})
  INSTALL(FILES ${p}  DESTINATION libexec)
endforeach()

add_executable(aster2asp aster2asp.cc)
target_link_libraries(aster2asp AspSessions)
install(TARGETS aster2asp DESTINATION bin)

add_executable(bundle_adjust bundle_adjust.cc bundle_adjust_cost_functions.h)
target_link_libraries(bundle_adjust AspSessions ${SOLVER_LIBRARIES})

install(TARGETS bundle_adjust DESTINATION bin)

add_executable(opencv_calibrate opencv_calibrate.cpp) 
target_link_libraries(opencv_calibrate AspCore)
install(TARGETS opencv_calibrate DESTINATION libexec)

add_executable(opencv_imagelist_creator opencv_imagelist_creator.cpp)
target_link_libraries(opencv_imagelist_creator AspCore)
install(TARGETS opencv_imagelist_creator DESTINATION libexec)

add_executable(dem_geoid dem_geoid.cc)
target_link_libraries(dem_geoid AspCore ${GEOID_LIBRARIES})
install(TARGETS dem_geoid DESTINATION bin)

add_executable(camera_footprint camera_footprint.cc)
target_link_libraries(camera_footprint AspSessions)
install(TARGETS camera_footprint DESTINATION bin)

add_executable(coverage_fraction coverage_fraction.cc)
target_link_libraries(coverage_fraction AspCore)
install(TARGETS coverage_fraction DESTINATION bin)

add_executable(disparitydebug disparitydebug.cc)
target_link_libraries(disparitydebug AspCore)
install(TARGETS disparitydebug DESTINATION bin)

add_executable(datum_convert_cpp datum_convert_cpp.cc)
target_link_libraries(datum_convert_cpp AspCore)
install(TARGETS datum_convert_cpp DESTINATION libexec)

add_executable(geodiff geodiff.cc)
target_link_libraries(geodiff AspCore)
install(TARGETS geodiff DESTINATION bin)

add_executable(hsv_merge hsv_merge.cc)
target_link_libraries(hsv_merge AspCore)
install(TARGETS hsv_merge DESTINATION bin)

add_executable(lronacjitreg lronacjitreg.cc)
target_link_libraries(lronacjitreg AspIsisIO)
install(TARGETS lronacjitreg DESTINATION bin)

add_executable(mapproject_single mapproject_single.cc)
target_link_libraries(mapproject_single AspSessions)
install(TARGETS mapproject_single DESTINATION bin)

add_executable(mer2camera mer2camera.cc)
target_link_libraries(mer2camera AspCore)
install(TARGETS mer2camera DESTINATION bin)

add_executable(orbitviz orbitviz.cc)
target_link_libraries(orbitviz AspSessions)
install(TARGETS orbitviz DESTINATION bin)

add_executable(pc_merge pc_merge.cc)
target_link_libraries(pc_merge AspSessions )
install(TARGETS pc_merge DESTINATION bin)

add_executable(pc_align pc_align.cc pc_align_utils.h pc_align_utils.tcc)
target_link_libraries(pc_align AspSessions ${SOLVER_LIBRARIES} 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES} ${FASTGLOBALREGISTRATION_LIBRARIES} )
install(TARGETS pc_align DESTINATION bin)

add_executable(n_align n_align.cc pc_align_utils.h pc_align_utils.tcc)
target_link_libraries(n_align AspSessions ${SOLVER_LIBRARIES} 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES} ${FASTGLOBALREGISTRATION_LIBRARIES} )
install(TARGETS n_align DESTINATION bin)

add_executable(pansharp pansharp.cc)
target_link_libraries(pansharp AspSessions)
install(TARGETS pansharp DESTINATION bin)

add_executable(image_calc image_calc.cc)
target_link_libraries(image_calc AspCore)
install(TARGETS image_calc DESTINATION bin)

add_executable(point2dem point2dem.cc)
target_link_libraries(point2dem AspCore)
install(TARGETS point2dem DESTINATION bin)

add_executable(point2las point2las.cc)
target_link_libraries(point2las AspCore ${LIBLAS_LIBRARIES}) 
install(TARGETS point2las DESTINATION bin)

add_executable(point2mesh point2mesh.cc)
target_link_libraries(point2mesh AspCore)
install(TARGETS point2mesh DESTINATION bin)

add_executable(add_spot_rpc add_spot_rpc.cc)
target_link_libraries(add_spot_rpc AspCamera)
install(TARGETS add_spot_rpc DESTINATION bin)

add_executable(cam2rpc cam2rpc.cc)
target_link_libraries(cam2rpc AspSessions)
install(TARGETS cam2rpc DESTINATION bin)

add_executable(rpc_gen rpc_gen.cc)
target_link_libraries(rpc_gen AspSessions)
install(TARGETS rpc_gen DESTINATION libexec)

add_executable(sfs sfs.cc)
target_link_libraries(sfs AspSessions ${SOLVER_LIBRARIES} )
install(TARGETS sfs DESTINATION bin)

add_executable(sfs_blend sfs_blend.cc)
target_link_libraries(sfs_blend AspSessions)
install(TARGETS sfs_blend DESTINATION bin)

add_executable(stereo_blend stereo_blend.cc stereo.h stereo.cc)
target_link_libraries(stereo_blend AspSessions)
install(TARGETS stereo_blend DESTINATION bin)

add_executable(stereo_corr stereo_corr.cc stereo.h stereo.cc)
target_link_libraries(stereo_corr AspSessions)
install(TARGETS stereo_corr DESTINATION bin)

add_executable(stereo_fltr stereo_fltr.cc stereo.h stereo.cc)
target_link_libraries(stereo_fltr AspSessions AspGotcha)
install(TARGETS stereo_fltr DESTINATION bin)

add_executable(stereo_gui stereo_gui.cc stereo.h stereo.cc)
target_link_libraries(stereo_gui AspSessions AspGUI)
install(TARGETS stereo_gui DESTINATION bin)

add_executable(stereo_parse stereo_parse.cc stereo.h stereo.cc) 
target_link_libraries(stereo_parse AspSessions)
install(TARGETS stereo_parse DESTINATION bin)

add_executable(stereo_pprc stereo_pprc.cc stereo.h stereo.cc) 
target_link_libraries(stereo_pprc AspSessions)
install(TARGETS stereo_pprc DESTINATION bin)

add_executable(stereo_rfne stereo_rfne.cc stereo.h stereo.cc) 
target_link_libraries(stereo_rfne AspSessions)
install(TARGETS stereo_rfne DESTINATION bin)

add_executable(stereo_tri stereo_tri.cc stereo.h stereo.cc jitter_adjust.cc jitter_adjust.h) 
target_link_libraries(stereo_tri AspSessions ${SOLVER_LIBRARIES})
install(TARGETS stereo_tri DESTINATION bin)

add_executable(jitter_solve jitter_solve.cc) 
target_link_libraries(jitter_solve AspSessions ${SOLVER_LIBRARIES})
install(TARGETS jitter_solve DESTINATION bin)

add_executable(xyzi2csv xyzi2csv.cc) 
target_link_libraries(xyzi2csv AspCore)
install(TARGETS xyzi2csv DESTINATION bin)

add_executable(dem_mosaic dem_mosaic.cc) 
target_link_libraries(dem_mosaic AspCore)
install(TARGETS dem_mosaic DESTINATION bin)

add_executable(image_mosaic image_mosaic.cc) 
target_link_libraries(image_mosaic AspCore)
install(TARGETS image_mosaic DESTINATION bin)

add_executable(image_align image_align.cc) 
target_link_libraries(image_align AspCore)
install(TARGETS image_align DESTINATION bin)

add_executable(tif_mosaic tif_mosaic.cc) 
target_link_libraries(tif_mosaic AspCore)
install(TARGETS tif_mosaic DESTINATION libexec)

add_executable(wv_correct wv_correct.cc) 
target_link_libraries(wv_correct AspCamera)
install(TARGETS wv_correct DESTINATION bin)

add_executable(cam_gen cam_gen.cc) 
target_link_libraries(cam_gen AspSessions)
install(TARGETS cam_gen DESTINATION bin)

add_executable(bathy_plane_calc bathy_plane_calc.cc) 
target_link_libraries(bathy_plane_calc AspSessions)
install(TARGETS bathy_plane_calc DESTINATION bin)

add_executable(cam_test cam_test.cc) 
target_link_libraries(cam_test AspSessions)
install(TARGETS cam_test DESTINATION bin)

add_executable(sat_sim sat_sim.cc) 
target_link_libraries(sat_sim AspSessions)
install(TARGETS sat_sim DESTINATION bin)

add_executable(ccd_solve ccd_solve.cc) 
target_link_libraries(ccd_solve AspSessions ${SOLVER_LIBRARIES})
install(TARGETS ccd_solve DESTINATION bin)

add_executable(otsu_threshold otsu_threshold.cc) 
target_link_libraries(otsu_threshold AspCore)
install(TARGETS otsu_threshold DESTINATION bin)

add_executable(corr_eval corr_eval.cc) 
target_link_libraries(corr_eval AspCore)
install(TARGETS corr_eval DESTINATION bin)

if(ASP_HAVE_PKG_ISISIO)
    # PCL gets imported via ISIS
    add_executable(pc_filter pc_filter.cc)
    target_link_libraries(pc_filter AspPclIO)
    install(TARGETS pc_filter DESTINATION bin)
endif()
    
# add_executable(nonlin_trans_corr nonlin_trans_corr.cc) 
# target_link_libraries(nonlin_trans_corr AspCore ${VTK_LIBRARIES})
# install(TARGETS nonlin_trans_corr DESTINATION bin)
