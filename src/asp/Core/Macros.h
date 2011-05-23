// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file Macros.h
///

#ifndef __ASP_CORE_MACROS_H__
#define __ASP_CORE_MACROS_H__

#define ASP_STANDARD_CATCHES                                \
    catch ( const ArgumentErr& e ) {                        \
    vw_out() << e.what() << std::endl;                      \
    return 1;                                               \
  } catch ( const Exception& e ) {                          \
    std::cerr << "\n\nVW Error: " << e.what() << std::endl; \
    return 1;                                               \
  } catch ( const std::bad_alloc& e ) {                     \
    std::cerr << "\n\nError: Ran out of Memory!" << std::endl; \
    return 1;                                               \
  } catch ( const std::exception& e ) {                     \
    std::cerr << "\n\nError: " << e.what() <<  std::endl;   \
    return 1;                                               \
  }

#endif//__ASP_CORE_MACROS_H__
