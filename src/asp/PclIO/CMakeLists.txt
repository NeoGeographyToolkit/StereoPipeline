# Use wrapper function at this level to avoid code duplication
add_library_wrapper(AspPclIO "${ASP_PCLIO_SRC_FILES}" "${ASP_PCLIO_TEST_FILES}" 
                    "${ASP_PCLIO_LIB_DEPENDENCIES}")
