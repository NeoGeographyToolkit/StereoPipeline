# Use wrapper function at this level to avoid code duplication

add_library_wrapper(AspCamera "${ASP_CAMERA_SRC_FILES}" "${ASP_CAMERA_TEST_FILES}"
    "${ASP_CAMERA_LIB_DEPENDENCIES}")

# This is needed for LinescanFit.cc. Not sure where a better place for it is.
target_link_libraries(AspCamera ${SOLVER_LIBRARIES})
