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

/// \file CameraErrorPropagation.cc
///

#include <asp/Camera/CameraErrorPropagation.h>
#include <asp/Camera/LinescanPleiadesModel.h>
#include <asp/Camera/RPCModel.h>

namespace asp {

  // Read the horizontal error from some camera models
  double horizontalStDevFromCamera(boost::shared_ptr<vw::camera::CameraModel> camera_model,
                                   bool & message_printed) {
    
  // Try to create horizontal stddev based on the RPC camera file
  const asp::RPCModel *rpc
    = dynamic_cast<const asp::RPCModel*>(vw::camera::unadjusted_model(camera_model.get()));
  if (rpc != NULL) {
    if (!message_printed) {
      vw::vw_out() << "Input horizontal stddev values needed for error propagation "
                   << "were not specified. Will create them based on the ERRBIAS "
                   << "and ERRRAND fields from the RPC cameras.\n";
      message_printed = true;
    }
    
    double bias = rpc->m_err_bias, rand = rpc->m_err_rand;
    if (bias > 0 && rand > 0) {
      return sqrt(bias * bias + rand * rand);
    } else {
      vw::vw_throw(vw::ArgumentErr()
                    << "Cannot propagate errors, as no input horizontal "
                    << "stddev values were specified, and some RPC camera models lack "
                    << "the necessary ERRBIAS and ERRRAND fields that could be used "
                    << "instead.\n");
    }
  }
    
  // Try to create horizontal stddev based on the Pleiades camera files
  const asp::PleiadesCameraModel *pleiades
    = dynamic_cast<const asp::PleiadesCameraModel*>
    (vw::camera::unadjusted_model(camera_model.get()));
  if (pleiades != NULL) {
    double accuracy = pleiades->m_accuracy_stdv;
    if (!message_printed) {
      vw::vw_out() << "Input horizontal stddev values needed for error propagation "
                   << "were not specified. Will create them based on the ACCURACY_STDV "
                   << "field from the Pleiades cameras.\n";
      message_printed = true;
    }
    
    if (accuracy > 0) {
      return accuracy;
    } else {
      vw::vw_throw(vw::ArgumentErr()
                    << "Cannot propagate errors, as no input horizontal "
                    << "stddev values were specified, and some Pleiades camera "
                    << "models lack the necessary ACCURACY_STDV field that could "
                    << "be used instead.\n");
    }
  }
    
  return 0;
}

// Some sanity checks and printing of horizontal stddevs
void horizontalStdDevCheck(vw::Vector<double> const& stddevs, std::string const& session) {
  
  bool isDg = (session == "dg" || session == "dgmaprpc");
  
  // See how many stddevs are positive. Throw an error if some are negative.
  int numTotal = stddevs.size();
  int numPositive = 0;
  for (int i = 0; i < (int)stddevs.size(); i++) {

    if (stddevs[i] < 0)
      vw::vw_throw(vw::ArgumentErr() << "Cannot have negative horizontal stddev.\n");

    if (stddevs[i] > 0) 
      numPositive++;
  }

  if (numPositive > 0 && numPositive < numTotal) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot have some positive and some zero "
                 "horizontal sttdev.\n");
  
  bool allPositive = (numPositive == numTotal);
  if (allPositive) {
    vw::vw_out() << "Horizontal stddev for the cameras: " << stddevs << " (meters).\n";
    if (isDg) 
      vw::vw_out() << "Will use these to find the point cloud stddev, rather than "
                   << "satellite position and orientation covariances.\n";
  } else {
    // For DG we will use satellite position and orientation covariances if no luck
    if (!isDg)
      vw::vw_throw(vw::ArgumentErr()
                    << "Cannot propagate errors, as no input horizontal "
                    << "stddev values were specified.\n");
  }
}

} // end namespace asp
