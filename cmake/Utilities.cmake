# This file contains functions used in other parts of the project.

# Obtains a file list with all the files in a directory properly formatted
function(get_all_source_files relativePath outputFileList)

  # Load all matching files into TEMP
  file(GLOB TEMP
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.h"
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.cc"
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.cxx"      
      "${CMAKE_CURRENT_SOURCE_DIR}/${relativePath}/*.tcc"
  )
  set(fileList) # Empty list
  foreach(f ${TEMP}) # Iterate through TEMP
    get_filename_component(FILENAME ${f} NAME) # Extract just the file name
    set(fileList ${fileList} ${FILENAME}) # Append to the list
  endforeach(f)
  set(${outputFileList} ${fileList} PARENT_SCOPE) 
endfunction(get_all_source_files)

# Look for a library dependency, starting with the BinaryBuilder folder.
# Look for it in search_folder if provided.
# If the header files are in a subfolder of "include", specify that one
# in "inc_subfolder".
function(find_external_library name search_folder inc_subfolder libNameList required)

  # Define the variable names we will create
  set(FOUND_NAME "${name}_FOUND")
  set(LIB_NAME   "${name}_LIBRARIES")
  set(INC_NAME   "${name}_INCLUDE_DIR")
  set(ASP_NAME   "ASP_HAVE_PKG_${name}") # TODO: Remove VW/ASP name!

  # Look in the search folder if it was provided, otherwise
  #  make halfhearted attempt to find the dependency.
  if(search_folder)
    set(${FOUND_NAME} 1)

    set(ext ".so")
    if (APPLE)
      set(ext ".dylib")
    endif()

    # Add each lib file that was provided.
    set(${${LIB_NAME}} "")
    foreach(lib ${libNameList})
      set(FULL_NAME "lib${lib}${ext}")
      set(FULL_PATH "${search_folder}/lib/${FULL_NAME}")
      if (NOT EXISTS ${FULL_PATH})
          # Try to see if maybe the lib is with an extension
          file(GLOB LIB_FILES ${FULL_PATH}*)
          list(GET LIB_FILES 0 FULL_PATH2) # get zero-th element
          if (EXISTS ${FULL_PATH2})
              set(FULL_PATH ${FULL_PATH2})
          else()
              message(STATUS "Missing library file: ${FULL_PATH}")
              set(${FOUND_NAME} 0)
              continue()
          endif()
      endif()
      set(${LIB_NAME} ${${LIB_NAME}} ${FULL_PATH})
    endforeach()
    
    set(${INC_NAME} ${search_folder}/include/${inc_subfolder})
  else()
    # TODO: Provide effective findX.cmake files to handle these.
    find_package(${name} REQUIRED)
  endif()
  # Check and display our results
  if(${FOUND_NAME})
    set(${ASP_NAME} 1)
    message(STATUS "Found include files for ${name} at ${${INC_NAME}}")
    include_directories("${${INC_NAME}}")
  else()
    if (${required})
      message( FATAL_ERROR "Failed to find REQUIRED library ${name}." )
    else()
      message(STATUS "Failed to find ${name}")
    endif()
  endif()

  # Pass the results back up to the parent function
  set(${FOUND_NAME} ${${FOUND_NAME}} PARENT_SCOPE)
  set(${LIB_NAME}   ${${LIB_NAME}}   PARENT_SCOPE)
  set(${INC_NAME}   ${${INC_NAME}}   PARENT_SCOPE)
  set(${ASP_NAME}   ${${ASP_NAME}}   PARENT_SCOPE)
 
  message(STATUS "Found libraries for ${name} at ${${LIB_NAME}}")

endfunction(find_external_library)

# Define a custom make target that will run all tests with normal gtest output.
# - Normally you can run 'make test' to run all tests but the output is brief.
# - With this you can run 'make gtest_all' to run all tests with more output.
if (NOT TARGET gtest_all)
  add_custom_target(gtest_all)
endif()
# Call this function once for each gtest target.
macro(add_to_custom_test_target test_target)
  add_custom_target(${test_target}_runtest
                    COMMAND ${test_target} #cmake 2.6 required
                    DEPENDS ${test_target}
                    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
  add_dependencies(gtest_all ${test_target}_runtest)
endmacro()


## Add the shared precompiled header to the current target.
## - Build it for the first target, then reuse it for all later targets.
#function(add_precompiled_header_to_target target)
  
#  #set(PCH_PATH "${CMAKE_HOME_DIRECTORY}/src/vw/stdafx.h")
#  set(PCH_PATH "../stdafx.h")
#  message("PCH_PATH = ${PCH_PATH}")
#  message("target = ${target}")
#  get_property(pchFirstLibrary GLOBAL PROPERTY storedPchFirstLibrary)
  
#  if(${pchFirstLibrary} STREQUAL "NA")
#    # First time this is called, don't reuse the PCH compilation.
#    set_property(GLOBAL PROPERTY storedPchFirstLibrary ${target})
#    target_precompiled_header(${target} ${PCH_PATH})
#  else()
#     target_precompiled_header(${target} ${PCH_PATH} REUSE ${pchFirstLibrary})
#  endif()
  
#endfunction(add_precompiled_header_to_target)


# Function to add a library to the project.
# - This is called in each library folder directory.
function(add_library_wrapper libName fileList testFileList dependencyList)

  # Set up the library
  add_library(${libName} SHARED ${fileList})

  set_target_properties(${libName} PROPERTIES LINKER_LANGUAGE CXX)
  
  #message("For ${libName}, linking DEPS: ${dependencyList}")
  target_link_libraries(${libName} ${dependencyList})

  # All libraries share the same precompiled header.
  #add_precompiled_header_to_target(${libName})

  install(TARGETS ${libName} DESTINATION lib)

  # Set all the header files to be installed to the include directory
  foreach(f ${fileList})
    get_filename_component(extension ${f} EXT) # Get file extension  
    string( TOLOWER "${extension}" extensionLower )
    if( extensionLower STREQUAL ".h" OR extensionLower STREQUAL ".hpp" OR extensionLower STREQUAL ".tcc")
      set(fullPath "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
      STRING(REGEX MATCH "vw/.*/" dir ${fullPath})
      INSTALL(FILES ${f} DESTINATION include/${dir})
    endif()
  endforeach(f)


  # Add unit test for each test file given
  set(TEST_MAIN_PATH "${CMAKE_SOURCE_DIR}/src/test/test_main.cc")
  foreach(f ${testFileList})

    get_filename_component(filename ${f} NAME_WE) # Get file name without extension
    set(executableName "${libName}_${filename}")   # Generate a name for the executable   

    #message("Adding test target ${executableName}")

    # Add executable with shared main file and this file
    # - This executeable should not be built unless running tests.
    add_executable( ${executableName} EXCLUDE_FROM_ALL  ${TEST_MAIN_PATH} ./tests/${f} )      

    # Link test executable against current library, gtest, and gtest_main
    #target_link_libraries(${executableName} gtest "${libName}" ${GTEST_BOTH_LIBRARIES})
    #message("For ${executableName}, linking DEPS: ${dependencyList};${libName}")
    target_link_libraries(${executableName} gtest gtest_main ${dependencyList} ${libName})

    target_compile_definitions(${executableName} PRIVATE GTEST_USE_OWN_TR1_TUPLE=1)
    target_compile_definitions(${executableName} PRIVATE "TEST_OBJDIR=\"${CMAKE_CURRENT_SOURCE_DIR}/tests\"")
    target_compile_definitions(${executableName} PRIVATE "TEST_SRCDIR=\"${CMAKE_CURRENT_SOURCE_DIR}/tests\"")


    # These variables need to be set for each test directory
    #set_property (TARGET ${executableName} APPEND PROPERTY COMPILE_DEFINITIONS "TEST_OBJDIR=\"${CMAKE_CURRENT_SOURCE_DIR}/tests\"")
    #set_property (TARGET ${executableName} APPEND PROPERTY COMPILE_DEFINITIONS "TEST_SRCDIR=\"${CMAKE_CURRENT_SOURCE_DIR}/tests\"")

    add_test(${executableName} ${executableName}) 
    add_to_custom_test_target(${executableName})  # Add to the verbose test make target.
  endforeach(f)

endfunction( add_library_wrapper )


