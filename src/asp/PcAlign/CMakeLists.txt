# This must be compiled with C++14. This is temporary, until FLANN can be
# updated to use C++17.
add_library(PcAlignFgr SHARED pc_align_fgr.cc pc_align_utils.h pc_align_utils.cc)
set_target_properties(PcAlignFgr PROPERTIES LINKER_LANGUAGE CXX)
target_compile_options(PcAlignFgr PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++14>)
target_link_libraries(PcAlignFgr 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES}
  AspSessions ${FASTGLOBALREGISTRATION_LIBRARIES})
install(TARGETS PcAlignFgr DESTINATION lib)

# This must surely be complied with C++17.
add_library(PcAlignCeres SHARED pc_align_ceres.cc pc_align_utils.h pc_align_utils.cc)
set_target_properties(PcAlignCeres PROPERTIES LINKER_LANGUAGE CXX)
target_compile_options(PcAlignCeres PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
target_link_libraries(PcAlignCeres 
  ${LIBPOINTMATCHER_LIBRARIES} ${LIBNABO_LIBRARIES}
  AspSessions ${SOLVER_LIBRARIES})
install(TARGETS PcAlignCeres DESTINATION lib)

