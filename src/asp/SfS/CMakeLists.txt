# Use wrapper function at this level to avoid code duplication

add_library_wrapper(AspSfS "${ASP_SfS_SRC_FILES}" "${ASP_SfS_TEST_FILES}"
    "${ASP_SfS_LIB_DEPENDENCIES}")

