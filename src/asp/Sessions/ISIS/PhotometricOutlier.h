// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file PhotometricOutlier.h
///
/// Warning: This code was written with only the Apollo Metric data in mind

#ifndef __STEREO_SESSION_ISIS_OUTLIER_H__
#define __STEREO_SESSION_ISIS_OUTLIER_H__

#include <string>

namespace vw {

  void photometric_outlier_rejection( std::string const& prefix,
                                      std::string const& input_disparity,
                                      std::string & output_disparity,
                                      int kernel_size );

}

#endif//__STEREO_SESSION_ISIS_OUTLIER_H__
