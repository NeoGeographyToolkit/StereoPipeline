add_library(AspPcAlignCeres SHARED pc_align_ceres.cc pc_align_utils.h 
            pc_align_utils.cc NuthFit.h NuthFit.cc)
target_link_libraries(AspPcAlignCeres 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES}
  AspSessions ${SOLVER_LIBRARIES})
install(TARGETS AspPcAlignCeres DESTINATION lib)

add_library(AspPcAlignFgr SHARED pc_align_fgr.cc pc_align_utils.h pc_align_utils.cc)
target_link_libraries(AspPcAlignFgr 
  AspSessions ${FASTGLOBALREGISTRATION_LIBRARIES}
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES})
install(TARGETS AspPcAlignFgr DESTINATION lib)
