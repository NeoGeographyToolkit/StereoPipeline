# Use wrapper function at this level to avoid code duplication

add_library_wrapper(AspPcAlign "${ASP_PC_ALIGN_SRC_FILES}" "${ASP_PC_ALIGN_TEST_FILES}"
    "${ASP_PC_ALIGN_LIB_DEPENDENCIES}")
