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

// Read the serial numbers of the images. For non-cub files,
// these will be the image names. 
void readSerialNumbers(std::vector<std::string> const& image_files,
                       std::vector<std::string> & serialNumbers) {

  // Wipe the output
  serialNumbers.clear();

  try {
    Isis::SerialNumberList serial_list;
    for (size_t i = 0; i < image_files.size(); i++) {
      QString fileName = QString::fromStdString(image_files[i]);
      serial_list.add(fileName);
    }
    for (size_t i = 0; i < image_files.size(); i++) {
      QString serialNumber = serial_list.serialNumber(i);
      serialNumbers.push_back(serialNumber.toStdString());
    }
  } catch (...) {
    // The above will fail for non-ISIS images. 
    serialNumbers.clear();
    for (size_t i = 0; i < image_files.size(); i++)
      serialNumbers.push_back(image_files[i]);
  }
}

// Load an ISIS cnet file and copy it to an ASP control network.
// The ISIS cnet will be used when saving the updated cnet.  
void loadIsisCnet(std::string const& isisCnetFile,
                  std::vector<std::string> const& image_files,
                  // Outputs
                  vw::ba::ControlNetwork & cnet,
                  IsisCnetData & isisCnetData) {

  // Reset the outputs
  cnet = vw::ba::ControlNetwork("ASP_control_network");
  isisCnetData = IsisCnetData();
  
  // Aliases
  Isis::ControlNetQsp & icnet = isisCnetData.isisCnet;
  std::map<int, int> & isisToAspControlPointId = isisCnetData.isisToAspControlPointId;

  // Report any ISIS-related surface points as lat/lon/radius, which is
  // the jigsaw default.
  // This should not matter as when points are later queried we explicitly
  // ask for ECEF coordinates (rectangular).
  Isis::SurfacePoint::CoordinateType coord_type = Isis::SurfacePoint::Latitudinal;

  // Read the ISIS control network
  QString qCnetFile = QString::fromStdString(isisCnetFile);
  Isis::Progress progress;
  icnet = Isis::ControlNetQsp(new Isis::ControlNet(qCnetFile, &progress, coord_type));
  
  // We will not reset the points rejected by jigsaw. Keep them rejected.
  // We won't even bother to read them into ASP. Such a points are tiny
  // minority, unlikely to have any effect. 
    
  // Create the map from image serial number to image index
  std::vector<std::string> serialNumbers;
  readSerialNumbers(image_files, serialNumbers);
  std::map<std::string, int> serialNumberToImageIndex;
  for (size_t i = 0; i < image_files.size(); i++) {
    serialNumberToImageIndex[serialNumbers[i]] = i;
    cnet.add_image_name(image_files[i]);
  }
   
  // Ensure we have as many serial numbers as images
  if (serialNumberToImageIndex.size() != image_files.size())
    vw_throw(vw::ArgumentErr() << "Found images with the same serial number. "
             << "Check your input data.\n");
     
  // Add the control points
  int numControlPoints = icnet->GetNumPoints();
  int aspControlPointId = 0;
  int numSemiFree = 0, numConstrained = 0, numFixed = 0;
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = icnet->GetPoint(i);
    
    // Do not add outliers to the ASP cnet. This makes the ASP cnet
    // be out of sync with the ISIS cnet. Later, the ASP cnet may also
    // have GCP, so new points. To keep track of all that, later
    // we will have a map from ISIS to ASP control point ids.
    if (point->IsIgnored() || point->IsRejected())
      continue;

    // Triangulated point and apriori point
    Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();
    Isis::SurfacePoint A = point->GetAprioriSurfacePoint();
    
    // Set the cnet tri point to the prior point. The surface point
    // will be triangulated by bundle_adjust.
    // By default, a point is free, and its sigma is nominal and will not be
    // used. Will adjust below for constrained and fixed points.
    // TODO(oalexan1): It is not clear if the prior point always exists
    // Check with the jigsaw code.
    vw::ba::ControlPoint cpoint(vw::ba::ControlPoint::TiePoint); // free
    vw::Vector3 a(A.GetX().meters(), A.GetY().meters(), A.GetZ().meters());
    cpoint.set_position(a);

    // Set sigma. This will be used only for constrained points. 
    // For free points sometimes these are positive, but sometimes not.
    double xs = A.GetXSigma().meters();
    double ys = A.GetYSigma().meters();
    double zs = A.GetZSigma().meters();
    cpoint.set_sigma(vw::Vector3(xs, ys, zs));

    // std::cout << "priori xs ys zs = " << xs << ' ' << ys << ' ' << zs << std::endl;
    // // Do same for the adjusted surface point
    // double px = P.GetXSigma().meters();
    // double py = P.GetYSigma().meters();
    // double pz = P.GetZSigma().meters();
    // std::cout << "adjusted px py pz = " << px << ' ' << py << ' ' << pz << std::endl;

    // The actual surface point will not be used    
    // vw::Vector3 p(P.GetX().meters(), P.GetY().meters(), P.GetZ().meters());

    if (point->GetType() == Isis::ControlPoint::Constrained) {
      
      // Treat partially constrained points as unconstrained.
      int numConstr = int(point->IsCoord1Constrained()) +
                      int(point->IsCoord2Constrained()) +
                      int(point->IsCoord3Constrained());
      if (numConstr < 3) {
        numSemiFree++;
      } else {
        // Fully constrained point
        numConstrained++;
        if (xs <= 0 || ys <= 0 || zs <= 0) 
          vw_throw(vw::ArgumentErr() << "loadIsisCnet: ISIS constrained point with index "
                    << i << " has a non-positive sigma.\n");
        // Set as gcp, but with given sigma, rather than tiny sigma.
        cpoint.set_type(vw::ba::ControlPoint::GroundControlPoint); // gcp
      }
    } else if (point->GetType() == Isis::ControlPoint::Free) {
       // Nothing to do here. The point and sigma is already set. 
    } else if (point->GetType() == Isis::ControlPoint::Fixed) {
      numFixed++;
      cpoint.set_type(vw::ba::ControlPoint::GroundControlPoint); // gcp
      double s = asp::FIXED_GCP_SIGMA; // Later will keep fixed
      cpoint.set_sigma(vw::Vector3(s, s, s));
    }
    
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

  if (numSemiFree > 0)
    vw::vw_out(vw::WarningMessage) 
      << "loadIsisCnet: Treating " << numSemiFree
      << " partially constrained points as unconstrained.\n"; 
  if (numConstrained > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numConstrained
                 << " constrained points. Treated as GCP with given sigma.\n";
  if (numFixed > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numFixed
                 << " fixed points. Treated as fixed GCP.\n";
  return;    
}

// Update an ISIS cnet with the latest info on triangulated points
// and outliers, and write it to disk at <outputPrefix>.net.
void saveUpdatedIsisCnet(std::string const& outputPrefix, 
                         asp::BAParams const& param_storage,
                         IsisCnetData & isisCnetData) {

  // Aliases
  Isis::ControlNetQsp const& icnet = isisCnetData.isisCnet;
  std::map<int, int> const& isisToAspControlPointId = isisCnetData.isisToAspControlPointId;

  // Iterate over the ISIS control points
  int numControlPoints = icnet->GetNumPoints();
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = icnet->GetPoint(i);
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
  icnet->Write(qCnetFile);
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

// Create from scratch and and save an ISIS cnet from a given control network
// and latest param values.
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
  
  // Find the serial numbers
  std::vector<std::string> serialNumbers;
  readSerialNumbers(image_files, serialNumbers);

  // Read the cameras This may fail for non-ISIS images. Then use null cameras.
  std::vector<boost::shared_ptr<Isis::Camera>> cameras;
  for (size_t i = 0; i < image_files.size(); i++) {
    Isis::Camera *cam = NULL;
    try {
      QString fileName = QString::fromStdString(image_files[i]);
      Isis::Cube cube(Isis::FileName(fileName), "r");
      Isis::Camera *cam = Isis::CameraFactory::Create(cube);
    } catch (...) {}
    cameras.push_back(boost::shared_ptr<Isis::Camera>(cam));
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
    
    // Must handle constrained point!
    // TODO(oalexan1): Must add gcp!
    // TODO(oalexan1): Must set the a priori point!
    // must set sigma for surface point and for apriori point
    // Must set sigma for measures!
    // if cnet is modified, must make sure crn is updated!
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
      // Set the measure sigma?
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
