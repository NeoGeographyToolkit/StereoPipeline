# Use wrapper function at this level to avoid code duplication

add_library_wrapper(AspRig "${ASP_RIG_SRC_FILES}" "${ASP_RIG_TEST_FILES}"
    "${ASP_RIG_LIB_DEPENDENCIES}")

#target_link_libraries(AspRig ${SOLVER_LIBRARIES})
