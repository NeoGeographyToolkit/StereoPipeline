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

/// \file BundleAdjustIsis.cc

// Utilities for handling ISIS's jigsaw control network format.

#include <asp/Camera/BundleAdjustIsis.h>

#include <vw/Core/Exception.h>

#include <isis/ControlNet.h>
#include <isis/SurfacePoint.h>
#include <isis/Progress.h>
#include <isis/SerialNumberList.h>
#include <isis/Latitude.h>
#include <isis/Longitude.h>
#include <isis/Distance.h>
#include <isis/BundleImage.h>

#include <boost/shared_ptr.hpp>

#include <string>
#include <iostream>

// Note: The output cnet file contains the updated control network with the final coordinates of the control points and residuals for each measurement.

namespace asp {
  void bundle_adjust_isis() {
    std::string cubeList = "cube.lis"; 
    std::string cnetFile = "control_ba2.net";
    std::cout << "now in bundle_adjust_isis" << std::endl;
    std::cout << "--will read cnet file: " << cnetFile << std::endl;
    //SurfacePoint::Latitudinal;
    
    // Report any ISIS-related surface points as lat/lon/radius, which is
    // the jigsaw default.
    QString qCnetFile = QString::fromStdString(cnetFile);
    Isis::SurfacePoint::CoordinateType coord_type = Isis::SurfacePoint::Latitudinal;
    Isis::Progress progress;
    Isis::ControlNetQsp m_controlNet 
      = Isis::ControlNetQsp(new Isis::ControlNet(qCnetFile, &progress, coord_type));

    // Create m_serialNumberList as a shared pointer so it can be safely 
    // deallocated when it goes out of scope.
    QString qCubeList = QString::fromStdString(cubeList);
    boost::shared_ptr<Isis::SerialNumberList> 
      m_serialNumberList(new Isis::SerialNumberList(qCubeList));
    
    int numImages = m_serialNumberList.get()->size();
    std::cout << "numImages = " << numImages << std::endl;
    
    for (int i = 0; i < numImages; i++) {
      Isis::Camera *camera = m_controlNet->Camera(i);
      QString observationNumber = m_serialNumberList.get()->observationNumber(i);
      QString instrumentId = m_serialNumberList.get()->spacecraftInstrumentId(i);
      QString serialNumber = m_serialNumberList.get()->serialNumber(i);
      QString fileName = m_serialNumberList.get()->fileName(i);
      
      std::cout << "\nindex is " << i << std::endl;
      std::cout << "observationNumber = " << observationNumber.toStdString() << std::endl;
      std::cout << "instrumentId = " << instrumentId.toStdString() << std::endl;
      std::cout << "serialNumber = " << serialNumber.toStdString() << std::endl;
      std::cout << "fileName = " << fileName.toStdString() << std::endl;
      
      // create a new BundleImage and add to new (or existing if observation mode is on)
      // BundleObservation
      Isis::BundleImageQsp image 
        = Isis::BundleImageQsp(new Isis::BundleImage(camera, serialNumber, fileName));
      
      if (!image) 
        vw::vw_throw(vw::ArgumentErr() 
                     << "Failed to load image: " << fileName.toStdString() << "\n");
      
    }
     
    int numControlPoints = m_controlNet->GetNumPoints();
    std::cout << "numControlPoints = " << numControlPoints << std::endl;
    for (int i = 0; i < numControlPoints; i++) {
      Isis::ControlPoint *point = m_controlNet->GetPoint(i);
      if (point->IsIgnored()) {
        continue;
      }

      if (point->IsIgnored()) {
        std::cout << "point " << i << " is ignored" << std::endl;
        continue;
      }
      std::cout << "point " << i << " is not ignored" << std::endl;
      
      
      Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();
      
      // Note: Weight and sigma only makes sense for constrained points.
      // This can throw exceptions.
       double sigma0 = point->GetAprioriSurfacePoint().
        GetSigmaDistance(coord_type, Isis::SurfacePoint::One).meters();
      double weight0 = point->GetAprioriSurfacePoint().
        GetWeight(coord_type, Isis::SurfacePoint::One);
      std::cout << "--sigma0, weight0 = " << sigma0 << " " << weight0 << std::endl;
      double sigma1 = point->GetAdjustedSurfacePoint().
        GetSigmaDistance(coord_type, Isis::SurfacePoint::Two).meters();
      double weight1 = point->GetAdjustedSurfacePoint().
        GetWeight(coord_type, Isis::SurfacePoint::Two);
      std::cout << "--sigma1, weight1 = " << sigma1 << " " << weight1 << std::endl;
      
      double sigma2 = point->GetAdjustedSurfacePoint().
        GetSigmaDistance(coord_type, Isis::SurfacePoint::Three).meters();
      double weight2 = point->GetAdjustedSurfacePoint().
        GetWeight(coord_type, Isis::SurfacePoint::Three);
      std::cout << "--sigma2, weight2 = " << sigma2 << " " << weight2 << std::endl;
            
      double lat = P.GetLatitude().degrees();
      double lon = P.GetLongitude().degrees();
      double radius = P.GetLocalRadius().meters();
      std::cout << "lon, lat, radius_m = " << lon << " " << lat << " " << radius << std::endl;
      if (point->GetType() == Isis::ControlPoint::Constrained) {
        std::cout << "--constrained point" << std::endl;
      } else if (point->GetType() == Isis::ControlPoint::Free) {
        std::cout << "--free point" << std::endl;
      } else if (point->GetType() == Isis::ControlPoint::Fixed) {
        std::cout << "--fixed point, no weights" << std::endl;
      } else {
        std::cout << "--unknown point type" << std::endl;
      }
        
      int numMeasures = point->GetNumMeasures();
      std::cout << "numMeasures = " << numMeasures << std::endl;
      for (int j = 0; j < numMeasures; j++) {
        Isis::ControlMeasure *controlMeasure = point->GetMeasure(j);
        // Get serial number as std::string
        // This serial number is same as for the image it came from
        QString qCubeSerialNumber = controlMeasure->GetCubeSerialNumber();
        std::string cubeSerialNumber = qCubeSerialNumber.toStdString();
        std::cout << "cubeSerialNumber = " << cubeSerialNumber << std::endl;
        
        // These have 0.5 added to them
        double line = controlMeasure->GetLine();
        double sample = controlMeasure->GetSample();
        std::cout << "line, sample = " << line << " " << sample << std::endl;
        double diameter = controlMeasure->GetDiameter();
        // Note: The diameter can be non-positive, which likely means it is not set.
        std::cout << "diameter = " << diameter << std::endl;
        
        // TODO(oalexan1): There is also apriori line and sample. When 
        // populating back the control network, we should set those.

      }
      
    }
  }
} 
