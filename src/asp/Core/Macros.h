// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
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
