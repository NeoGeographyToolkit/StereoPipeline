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
#include <isis/Pvl.h>
#include <isis/Target.h>
#include <isis/CameraFactory.h>
#include <isis/Camera.h>
#include <isis/Cube.h>
#include <isis/FileName.h>

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
    if (point->IsIgnored() || point->IsRejected())
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

// Set the Isis cnet target based on the image name. If that fails,
// as for Earth, set it to the datum name.
void setIsisCnetTarget(std::string const& image_name, 
                       vw::cartography::Datum const& datum,
                       Isis::ControlNet & icnet) {

  try {
    Isis::Pvl cubeLab(QString::fromStdString(image_name));
    Isis::Pvl maplab;
    maplab.addGroup(Isis::PvlGroup("Mapping"));
    Isis::PvlGroup &mapGroup = maplab.findGroup("Mapping");
    mapGroup = Isis::Target::radiiGroup(cubeLab, mapGroup);
    icnet.SetTarget(maplab);
  } catch (...) {
    try {
      // Set the target to the datum name
      icnet.SetTarget(QString::fromStdString(datum.name()));
    } catch (...) {
      // If really no luck, just use Mars and hope for the best
      icnet.SetTarget(QString::fromStdString("Mars"));
    }
  }
}

// Create and save ISIS cnet from a given control network and latest param
// values.
void saveIsisCnet(std::string const& outputPrefix, 
                  vw::ba::ControlNetwork const& cnet,
                  vw::cartography::Datum const& datum,
                  asp::BAParams const& param_storage) {

  // Sanity check
  if (param_storage.num_points() != cnet.size())
    vw::vw_throw(vw::ArgumentErr() 
             << "saveIsisCnet: number of points in param_storage and cnet do not match.\n");

  // Create a list of cub files. Need this to find the serial numbers
  std::vector<std::string> image_files = cnet.get_image_list();
  std::string cubeList = outputPrefix + "-list.txt";
  vw::vw_out() << "Writing image list: " << cubeList << std::endl;
  asp::write_list(cubeList, image_files); 
  
  // Find the serial numbers and cameras. This may fail for non-ISIS images.
  // Then use the image names and null cameras.
  std::vector<std::string> serialNumbers;
  std::vector<boost::shared_ptr<Isis::Camera>> cameras;
  try {
    Isis::SerialNumberList serial_list(QString::fromStdString(cubeList));
    for (size_t i = 0; i < image_files.size(); i++) {
      QString fileName = serial_list.fileName(i);
      QString serialNumber = serial_list.serialNumber(i);
      serialNumbers.push_back(serialNumber.toStdString());
      Isis::Cube cube(Isis::FileName(fileName), "r");
      Isis::Camera *cam = Isis::CameraFactory::Create(cube);
      cameras.push_back(boost::shared_ptr<Isis::Camera>(cam));
    }
  } catch (...) {
    serialNumbers = image_files;
    for (size_t i = 0; i < image_files.size(); i++)
      cameras.push_back(boost::shared_ptr<Isis::Camera>(NULL));
  }

  // Initialize the isis cnet
  Isis::ControlNet icnet;

  setIsisCnetTarget(image_files[0], datum, icnet);
  icnet.SetNetworkId(QString::fromStdString("bundle_adjust"));
  icnet.SetUserName(QString::fromStdString("bundle_adjust"));
  icnet.SetDescription(QString::fromStdString("bundle_adjust"));

  // Iterate over the ASP control points
  for (int ipt = 0; ipt < cnet.size(); ipt++) {
   
    // Skip outliers
    if (param_storage.get_point_outlier(ipt))
      continue;
    
    Isis::ControlPoint *point = new Isis::ControlPoint();
    point->SetRejected(false);
    point->SetIgnored(false);
    
    // For now, no constrained points are supported
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint) 
      point->SetType(Isis::ControlPoint::Fixed);
    else
      point->SetType(Isis::ControlPoint::Free);
    
    std::ostringstream os;
    os << "point_" << ipt;
    point->SetId(QString::fromStdString(os.str()));
    
    Isis::SurfacePoint P;
    const double * asp_point = param_storage.get_point_ptr(ipt); // ecef, meters
    Isis::Distance s(1.0, Isis::Distance::Meters); // sigma, meters
    P.SetRectangular(Isis::Displacement(asp_point[0], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[1], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[2], Isis::Displacement::Meters),
                     s, s, s);
    point->SetAdjustedSurfacePoint(P);

    // Add the measures
    for (auto m_iter = cnet[ipt].begin(); m_iter != cnet[ipt].end(); m_iter++) {
      int cid = m_iter->image_id();
      
      // Must add 0.5 to the ASP measure to get the ISIS measure
      double col = m_iter->position()[0] - ISIS_CNET_TO_ASP_OFFSET;
      double row = m_iter->position()[1] - ISIS_CNET_TO_ASP_OFFSET;
      
      Isis::ControlMeasure *measurement = new Isis::ControlMeasure();
      measurement->SetCoordinate(col, row, Isis::ControlMeasure::RegisteredSubPixel);
      measurement->SetType(Isis::ControlMeasure::RegisteredSubPixel);
      measurement->SetAprioriSample(col);
      measurement->SetAprioriLine(row);
      measurement->SetIgnored(false);
      measurement->SetRejected(false);
      measurement->SetSampleSigma(1.0);
      measurement->SetLineSigma(1.0);
      measurement->SetResidual(0.0, 0.0);
      measurement->SetCubeSerialNumber(QString::fromStdString(serialNumbers[cid]));
      measurement->SetCamera(cameras[cid].get());

      point->Add(measurement);
    }
    
    icnet.AddPoint(point);
  }
  
  vw::vw_out() << "Control network number of points: " << icnet.GetNumPoints() << "\n";
  vw::vw_out() << "Target: " << icnet.GetTarget().toStdString() << std::endl;

  std::string cnetFile = outputPrefix + ".net";
  vw::vw_out() << "Writing ISIS control network: " << cnetFile << "\n"; 
  icnet.Write(QString::fromStdString(cnetFile));

  return;
}
} // end namespace asp
