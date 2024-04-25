# This must be compiled with C++14. This is temporary, until FLANN can be
# updated to use C++17.
add_library(AspPcAlignFgr SHARED pc_align_fgr.cc pc_align_utils.h pc_align_utils.cc)
set_target_properties(AspPcAlignFgr PROPERTIES LINKER_LANGUAGE CXX)
target_compile_options(AspPcAlignFgr PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++14>)
target_link_libraries(AspPcAlignFgr 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES}
  AspSessions ${FASTGLOBALREGISTRATION_LIBRARIES})
install(TARGETS AspPcAlignFgr DESTINATION lib)

# This must surely be complied with C++17.
add_library(AspPcAlignCeres SHARED pc_align_ceres.cc pc_align_utils.h pc_align_utils.cc)
set_target_properties(AspPcAlignCeres PROPERTIES LINKER_LANGUAGE CXX)
target_compile_options(AspPcAlignCeres PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
target_link_libraries(AspPcAlignCeres 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES}
  AspSessions ${SOLVER_LIBRARIES})
install(TARGETS AspPcAlignCeres DESTINATION lib)

