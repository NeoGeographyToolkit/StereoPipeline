# Use wrapper function at this level to avoid code duplication

add_library_wrapper(AspPcAlign "${ASP_PC_ALIGN_SRC_FILES}" "${ASP_PC_ALIGN_TEST_FILES}"
    "${ASP_PC_ALIGN_LIB_DEPENDENCIES}")

# This must be built with C++14 as otherwise it fails. Must be set
# after the standard is set for the entire library.
set_source_files_properties(
    pc_align_fgr.cc
    PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF # Keep consistent if desired
    COMPILE_FLAGS "-std=c++14"
)

