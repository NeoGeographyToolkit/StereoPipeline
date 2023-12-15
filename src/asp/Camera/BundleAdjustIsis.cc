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
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Core/Common.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
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

// Note: The output cnet file must contain the updated control network with the
// final coordinates of the control points and residuals for each measurement.
// It also must have serial numbers, apriori sigma, apriori surface points, etc.
namespace asp {

// ISIS cnet measures have an additional 0.5 added to them
const double ISIS_CNET_TO_ASP_OFFSET = -0.5;

// Load an ISIS cnet file and copy it to an ASP control network.
// The ISIS cnet will be used when saving the updated cnet.  
void loadIsisCnet(std::string const& isisCnetFile,
                  std::string const& outputPrefix, 
                  std::vector<std::string> const& image_files,
                  // Outputs
                  vw::ba::ControlNetwork & cnet,
                  IsisCnetData & isisCnetData) {

  // Reset the outputs
  cnet = vw::ba::ControlNetwork("ASP_control_network");
  isisCnetData = IsisCnetData();
  
  // Aliases
  Isis::ControlNetQsp & isisCnet = isisCnetData.isisCnet;
  boost::shared_ptr<Isis::SerialNumberList> & isisImgData = isisCnetData.isisImgData;
  std::map<int, int> & isisToAspControlPointId = isisCnetData.isisToAspControlPointId;

  // Report any ISIS-related surface points as lat/lon/radius, which is
  // the jigsaw default.
  Isis::SurfacePoint::CoordinateType coord_type = Isis::SurfacePoint::Latitudinal;

  // Read the ISIS control network
  vw::vw_out() << "Reading ISIS control network: " << isisCnetFile << "\n";
  QString qCnetFile = QString::fromStdString(isisCnetFile);
  Isis::Progress progress;
  isisCnet = Isis::ControlNetQsp(new Isis::ControlNet(qCnetFile, &progress, coord_type));
  
  // Create a list of cub files. Need this to find the serial numbers
  std::string cubeList = outputPrefix + "-list.txt";
  vw::vw_out() << "Writing image list: " << cubeList << std::endl;
  asp::write_list(cubeList, image_files); 
  
  // Read the info about images, and add them to the cnet
  QString qCubeList = QString::fromStdString(cubeList);
  isisImgData.reset(new Isis::SerialNumberList(qCubeList));
  std::map<std::string, int> serialNumberToImageIndex;
  int numImages = isisImgData.get()->size();
  for (int i = 0; i < numImages; i++) {
    QString fileName = isisImgData.get()->fileName(i);
    QString serialNumber = isisImgData.get()->serialNumber(i);
    serialNumberToImageIndex[serialNumber.toStdString()] = i;
    cnet.add_image_name(fileName.toStdString());
  }
   
  // Ensure we have as many serial numbers as images
  if (serialNumberToImageIndex.size() != image_files.size())
    vw_throw(vw::ArgumentErr() << "Found images with the same serial number. "
             << "Check your input data.\n");
     
  // Add the control points
  int numControlPoints = isisCnet->GetNumPoints();
  int aspControlPointId = 0;
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = isisCnet->GetPoint(i);
    
    // Do not add outliers to the ASP cnet. This makes the ASP cnet
    // be out of sync with the ISIS cnet. Later, the ASP cnet may also
    // have GCP, so new points. To keep track of all that, later
    // we will have a map from ISIS to ASP control point ids.
    if (point->IsIgnored()) 
      continue;

    // Triangulated point  
    Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();

    // Treat, for now, constrained points as free points. Only flag
    // fixed points as GCPs.
    vw::ba::ControlPoint cpoint(vw::ba::ControlPoint::TiePoint); // free
    if (point->GetType() == Isis::ControlPoint::Constrained) {
    } else if (point->GetType() == Isis::ControlPoint::Free) {
    } else if (point->GetType() == Isis::ControlPoint::Fixed) {
      cpoint.set_type(vw::ba::ControlPoint::GroundControlPoint); // gcp
    }
    
    // Set point position. 
    // TODO(oalexan1): Must set the point sigma. For now it is 1.0.
    vw::Vector3 ecef(P.GetX().meters(), P.GetY().meters(), P.GetZ().meters());
    cpoint.set_position(ecef);
    
    int numMeasures = point->GetNumMeasures();
    for (int j = 0; j < numMeasures; j++) {

      Isis::ControlMeasure *controlMeasure = point->GetMeasure(j);

      // Get serial number as std::string. This is unique to each image.
      QString qCubeSerialNumber = controlMeasure->GetCubeSerialNumber();
      std::string cubeSerialNumber = qCubeSerialNumber.toStdString();
      
      // These have 0.5 added to them, which we will remove
      double sample = controlMeasure->GetSample();
      double line = controlMeasure->GetLine();
      double col = sample + ISIS_CNET_TO_ASP_OFFSET;
      double row = line   + ISIS_CNET_TO_ASP_OFFSET;

      // These sigmas can turn out to be negative
      double sample_sigma = controlMeasure->GetSampleSigma();
      double line_sigma   = controlMeasure->GetLineSigma();
      if (sample_sigma <= 0 || std::isnan(sample_sigma) || std::isinf(sample_sigma))
        sample_sigma = 1.0;
      if (line_sigma <= 0 || std::isnan(line_sigma) || std::isinf(line_sigma))
        line_sigma = 1.0;
      
      // Find the image index
      auto it = serialNumberToImageIndex.find(cubeSerialNumber);
      if (it == serialNumberToImageIndex.end())
        vw_throw(vw::ArgumentErr() << "Could not find image with serial number: "
                 << cubeSerialNumber << ".\n");
      int image_id = it->second;
      cpoint.add_measure(vw::ba::ControlMeasure(col, row, sample_sigma, line_sigma,
                                                image_id));
    }
    
    // Need this so later can update the ISIS cnet with the ASP cnet.
    if (cpoint.size() > 0) {
      cnet.add_control_point(cpoint);
      isisToAspControlPointId[i] = aspControlPointId;
      aspControlPointId++;
    }
  }    

  return;    
}

// Update an ISIS cnet with the latest info on triangulated points
// and outliers, and write it to disk at <outputPrefix>.net.
void saveUpdatedIsisCnet(std::string const& outputPrefix, 
                         asp::BAParams const& param_storage,
                         IsisCnetData & isisCnetData) {

  // Aliases
  Isis::ControlNetQsp const& isisCnet = isisCnetData.isisCnet;
  boost::shared_ptr<Isis::SerialNumberList> const& isisImgData = isisCnetData.isisImgData;
  std::map<int, int> const& isisToAspControlPointId = isisCnetData.isisToAspControlPointId;
  
  // Iterate over the ISIS control points
  int numControlPoints = isisCnet->GetNumPoints();
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = isisCnet->GetPoint(i);
    if (point->IsIgnored() || point->IsRejected()) {
      continue;
    }
    
    auto it = isisToAspControlPointId.find(i);
    if (it == isisToAspControlPointId.end())
      vw_throw(vw::ArgumentErr() 
               << "Could not find ASP control point for ISIS control point: "
               << i << ".\n");
    int aspControlPointId = it->second;

    if (param_storage.get_point_outlier(aspControlPointId)) {
      // Must flag as outlier in ISIS. It appears that 'ignored'
      // and 'rejected' are different things, but in the ISIS code
      // both are passed over when some calculations are done.
      point->SetRejected(true);
      point->SetIgnored(true);
      continue;
    }
    
    // ECEF coordinates of the triangulated point
    const double * asp_point = param_storage.get_point_ptr(aspControlPointId);
    vw::Vector3 ecef(asp_point[0], asp_point[1], asp_point[2]);
    
    // Copy this over to ISIS
    Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();
    P.SetRectangular(Isis::Displacement(asp_point[0], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[1], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[2], Isis::Displacement::Meters),
                     P.GetXSigma(), P.GetYSigma(), P.GetZSigma());
    
    // Set the updated point position
    point->SetAdjustedSurfacePoint(P);
  }
  
  std::string cnetFile = outputPrefix + ".net";
  vw::vw_out() << "Writing ISIS control network: " << cnetFile << "\n"; 
  // convert to QString
  QString qCnetFile = QString::fromStdString(cnetFile);
  isisCnet->Write(qCnetFile);
  
}
# if 0
{
  // Report any ISIS-related surface points as lat/lon/radius, which is
  // the jigsaw default.
  Isis::SurfacePoint::CoordinateType coord_type = Isis::SurfacePoint::Latitudinal;

  // Create a new ISIS cnet
  Isis::ControlNet newIsisCnet;
  newIsisCnet.SetUserName(isisCnet->GetUserName());
  newIsisCnet.SetCreatedDate(isisCnet->GetCreatedDate());
  newIsisCnet.SetModifiedDate(isisCnet->GetModifiedDate());
  newIsisCnet.SetDescription(isisCnet->GetDescription());
  newIsisCnet.SetLastModifiedBy(isisCnet->GetLastModifiedBy());
  newIsisCnet.SetComment(isisCnet->GetComment());
  newIsisCnet.SetTarget(isisCnet->GetTarget());
  newIsisCnet.SetSource(isisCnet->GetSource());
  newIsisCnet.SetInstrument(isisCnet->GetInstrument());
  newIsisCnet.SetProductId(isisCnet->GetProductId());
  newIsisCnet.SetProductType(isisCnet->GetProductType());
  newIsisCnet.SetMission(isisCnet->GetMission());
  newIsisCnet.SetUtcStartTime(isisCnet->GetUtcStartTime());
  newIsisCnet.SetUtcStopTime(isisCnet->GetUtcStopTime());
  newIsisCnet.SetStartTime(isisCnet->GetStartTime());
  newIsisCnet.SetStopTime(isisCnet->GetStopTime());
  newIsisCnet.SetSpatialQuality(isisCnet->GetSpatialQuality());
  newIsisCnet.SetImageId(isisCnet->GetImageId());
  newIsisCnet.SetProductId(isisCnet->GetProductId());
  newIsisCnet.SetProductType(isisCnet->GetProduct
}
#endif

// TODO(oalexan1): When creating an ISIS cnet from scratch,
// must populate apriori sigma, apriori surface point, etc.
  
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

} // end namespace asp
