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
#include <asp/Camera/BaseCostFuns.h>
#include <asp/asp_config.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Core/Exception.h>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
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
#endif // ASP_HAVE_PKG_ISIS

#include <boost/shared_ptr.hpp>

#include <string>
#include <iostream>

// TODO(oalexan1): Must check that number of images agrees with number
// of cid in cnet.

// Note: The output cnet file must contain the updated control network with the
// final coordinates of the control points and residuals for each measurement.
// It also must have serial numbers, apriori sigma, apriori surface points, etc.
namespace asp {

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

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
      // If really no luck, just use WGS84, as it is more likely to fail for Earth
      icnet.SetTarget(QString::fromStdString("WGS84"));
    }
  }
}

// Read the cameras This may fail for non-ISIS images. Then use null cameras.
void readIsisCameras(std::vector<std::string> const& image_files,
                     std::vector<boost::shared_ptr<Isis::Camera>> & cameras) {

  // Wipe the output
  cameras.clear();
  
  for (size_t i = 0; i < image_files.size(); i++) {
    Isis::Camera *cam = NULL;
    try {
      QString fileName = QString::fromStdString(image_files[i]);
      Isis::Cube cube(Isis::FileName(fileName), "r");
      Isis::Camera *cam = Isis::CameraFactory::Create(cube);
    } catch (...) {}
    cameras.push_back(boost::shared_ptr<Isis::Camera>(cam));
  }
}

// Add a given control point to the ISIS cnet. Update the outlier counter.
void addIsisControlPoint(Isis::ControlNet & icnet,
                         vw::ba::ControlNetwork const& cnet,
                         asp::BAParams const& param_storage,
                         std::vector<boost::shared_ptr<Isis::Camera>> const& cameras,
                         std::vector<std::string> const& serialNumbers,
                         int ipt, int& numOutliers) {

  // Sanity check for cnet and param_storage
  if (cnet.size() != param_storage.num_points())
    vw_throw(vw::ArgumentErr() 
             << "addIsisControlPoint: cnet and param_storage have different sizes.\n");
  if (ipt >= cnet.size() || ipt >= param_storage.num_points())
    vw_throw(vw::ArgumentErr() 
             << "addIsisControlPoint: ipt is too large.\n");
    
  Isis::ControlPoint *point = new Isis::ControlPoint();
  if (param_storage.get_point_outlier(ipt)) {
    point->SetIgnored(true); // better set it to ignored as well
    point->SetRejected(true);
    numOutliers++;
  } else {    
    point->SetRejected(false);
  }
  
  // The bundle_adjust convention is that a point is fixed if its sigma is
  // asp::FIXED_GCP_SIGMA (a very tiny number). Otherwise it is variable but
  // constrained by sigma.
  vw::Vector3 sigma = cnet[ipt].sigma(); // point sigma
  double s = asp::FIXED_GCP_SIGMA;
  if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint) {
    if (sigma == vw::Vector3(s, s, s)) {
      point->SetType(Isis::ControlPoint::Fixed);
    } else {
      point->SetType(Isis::ControlPoint::Constrained);
    }
  } else {
    point->SetType(Isis::ControlPoint::Free);
  }
  
  // Set the point id
  std::ostringstream os;
  os << "point_" << ipt;
  point->SetId(QString::fromStdString(os.str()));
  
  // Set apriori position from the input cnet
  Isis::SurfacePoint A;
  vw::Vector3 a = cnet[ipt].position();
  A.SetRectangular(Isis::Displacement(a[0], Isis::Displacement::Meters),
                   Isis::Displacement(a[1], Isis::Displacement::Meters),
                   Isis::Displacement(a[2], Isis::Displacement::Meters),
                   Isis::Distance(sigma[0], Isis::Distance::Meters),
                   Isis::Distance(sigma[1], Isis::Distance::Meters),
                   Isis::Distance(sigma[2], Isis::Distance::Meters));
  point->SetAprioriSurfacePoint(A);
  
  // Set the optimized (adjusted) position  
  Isis::SurfacePoint P;
  const double * asp_point = param_storage.get_point_ptr(ipt); // ecef, meters
  P.SetRectangular(Isis::Displacement(asp_point[0], Isis::Displacement::Meters),
                    Isis::Displacement(asp_point[1], Isis::Displacement::Meters),
                    Isis::Displacement(asp_point[2], Isis::Displacement::Meters),
                    Isis::Distance(sigma[0], Isis::Distance::Meters),
                    Isis::Distance(sigma[1], Isis::Distance::Meters),
                    Isis::Distance(sigma[2], Isis::Distance::Meters));
  point->SetAdjustedSurfacePoint(P);

  // Add the measures, and their sigmas
  for (auto m_iter = cnet[ipt].begin(); m_iter != cnet[ipt].end(); m_iter++) {
    
    int cid = m_iter->image_id();
    vw::Vector2 sigma = m_iter->sigma();
    
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
    measurement->SetSampleSigma(sigma[0]);
    measurement->SetLineSigma(sigma[1]);
    measurement->SetResidual(0.0, 0.0);
    measurement->SetCubeSerialNumber(QString::fromStdString(serialNumbers[cid]));
    measurement->SetCamera(cameras[cid].get());

    point->Add(measurement);
  }
  
  icnet.AddPoint(point);
}

#endif // ASP_HAVE_PKG_ISIS

// Load an ISIS cnet file and copy it to an ASP control network. The ISIS cnet
// will be used when saving the updated cnet. Keep these cnets one-to-one,
// though later the ASP cnet may also have GCP.
void loadIsisCnet(std::string const& isisCnetFile,
                  std::vector<std::string> const& image_files,
                  // Outputs
                  vw::ba::ControlNetwork & cnet,
                  IsisCnetData & isisCnetData) {

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

  // Reset the outputs
  cnet = vw::ba::ControlNetwork("ASP_control_network");
  isisCnetData = IsisCnetData();
  
  // Aliases
  Isis::ControlNetQsp & icnet = isisCnetData.isisCnet;
  std::set<int> & isisOutliers = isisCnetData.isisOutliers;

  // Report any ISIS-related surface points as ECEF (rectangular). Not really used.
  Isis::SurfacePoint::CoordinateType coord_type = Isis::SurfacePoint::Rectangular;

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
  int numSemiFree = 0, numConstrained = 0, numFixed = 0, numRejected = 0, numIgnored = 0;
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = icnet->GetPoint(i);
    bool ignore = false;
    
    if (point->IsIgnored()) {
      numIgnored++;
      // We do not skip any points, to preserve the one-to-one correspondence.
      // Add to isisOutliers
      isisOutliers.insert(i);
      ignore = true;
    }
    if (point->IsRejected()) {
      numRejected++;
      // We do not skip any points, to preserve the one-to-one correspondence.
      // Add to isisOutliers
      isisOutliers.insert(i);
      ignore = true;
    }
      
    // Triangulated point and apriori point
    Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();
    Isis::SurfacePoint A = point->GetAprioriSurfacePoint();
    
    // Set the cnet tri point to the prior point. The surface point
    // will be triangulated by bundle_adjust.
    // By default, a point is free, and its sigma is nominal and will not be
    // used. Will adjust below for constrained and fixed points.
    vw::ba::ControlPoint cpoint(vw::ba::ControlPoint::TiePoint); // free
    vw::Vector3 a(A.GetX().meters(), A.GetY().meters(), A.GetZ().meters());
    cpoint.set_position(a);
    cpoint.set_ignore(ignore);

     // Set sigma. This will be used only for constrained points. 
     // Use the sigmas from the adjusted surface points, as they are 
     // more likely to be positive and perhaps up-to-date than the
     // prior sigmas.
     double xs = P.GetXSigma().meters();
     double ys = P.GetYSigma().meters();
     double zs = P.GetZSigma().meters();
     cpoint.set_sigma(vw::Vector3(xs, ys, zs));

    // The actual surface point will not be used. It will later be initialized
    // as the a priori point, and then optimized.    
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
          vw_throw(vw::ArgumentErr() 
                    << "loadIsisCnet: ISIS constrained point with index "
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

    // We do not skip any points, to preserve the one-to-one correspondence.
    // But points with no measures are flagged as outliers.    
    if (cpoint.size() == 0) {
      isisOutliers.insert(i);
      cpoint.set_ignore(true);
    }
    
    cnet.add_control_point(cpoint);
  }    

  if (numSemiFree > 0)
    vw::vw_out(vw::WarningMessage) 
      << "loadIsisCnet: Treat " << numSemiFree
      << " partially constrained points as unconstrained.\n"; 
  if (numConstrained > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numConstrained
                 << " constrained points. Use as GCP with given sigma.\n";
  if (numFixed > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numFixed
                 << " fixed points. Use as fixed GCP.\n";
  if (numRejected > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numRejected
                 << " rejected points. Flag as outliers.\n";
  if (numIgnored > 0)
    vw::vw_out() << "loadIsisCnet: Found " << numIgnored
                 << " ignored points. Flag as outliers.\n";
                 
  vw::vw_out() << "Loaded " << cnet.size() << " control points.\n";

#endif // ASP_HAVE_PKG_ISIS
  
  return;    
}

// Update an ISIS cnet with the latest info on triangulated points
// and outliers, and write it to disk at <outputPrefix>.net.
// We do not change here if a point is fixed, constrained, or free.
void saveUpdatedIsisCnet(std::string const& outputPrefix, 
                         vw::ba::ControlNetwork const& cnet,
                         asp::BAParams const& param_storage,
                         IsisCnetData & isisCnetData) {

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

  // Sanity check
  if (param_storage.num_points() != cnet.size())
    vw::vw_throw(vw::ArgumentErr() 
             << "saveIsisCnet: number of points in param_storage and cnet do not match.\n");

  // Find the serial numbers
  std::vector<std::string> image_files = cnet.get_image_list();
  std::vector<std::string> serialNumbers;
  readSerialNumbers(image_files, serialNumbers);
  // Read the cameras This may fail for non-ISIS images. Then use null cameras.
  std::vector<boost::shared_ptr<Isis::Camera>> cameras;
  readIsisCameras(image_files, cameras);

  // Aliases
  Isis::ControlNetQsp const& icnet = isisCnetData.isisCnet;

  // The ASP cnet may have more points than the ISIS cnet, as it may have GCP.
  int numControlPoints = icnet->GetNumPoints();
  if (numControlPoints > param_storage.num_points())
    vw_throw(vw::ArgumentErr() 
             << "saveUpdatedIsisCnet: Book-keeping failure. ASP cnet is too small.\n");
    
  // Iterate over the ISIS control points
  int numOutliers = 0;
  for (int i = 0; i < numControlPoints; i++) {

    Isis::ControlPoint *point = icnet->GetPoint(i);
    if (point->IsIgnored() || point->IsRejected() || param_storage.get_point_outlier(i)) {
      point->SetIgnored(true);
      point->SetRejected(true);
      numOutliers++;
      // On occasion there is failure below if continuing with bad points
      continue;
    }
    
    // Update the apriori position from the cnet. Note that this may 
    // have changed if a transform was applied to the cameras.
    // Get the apriori sigma too. Some bundle adjustment options change
    // that too, though not by default.
    vw::Vector3 a = cnet[i].position();
    vw::Vector3 sigma = cnet[i].sigma(); // point sigma
    Isis::SurfacePoint A = point->GetAprioriSurfacePoint();
    A.SetRectangular(Isis::Displacement(a[0], Isis::Displacement::Meters),
                     Isis::Displacement(a[1], Isis::Displacement::Meters),
                     Isis::Displacement(a[2], Isis::Displacement::Meters),
                     Isis::Distance(sigma[0], Isis::Distance::Meters),
                     Isis::Distance(sigma[1], Isis::Distance::Meters),
                     Isis::Distance(sigma[2], Isis::Distance::Meters));
    point->SetAprioriSurfacePoint(A);

    // Update the triangulated point. Use same sigmas as above. These
    // may have been changed by bundle adjustment.
    const double * asp_point = param_storage.get_point_ptr(i);
    Isis::SurfacePoint P = point->GetAdjustedSurfacePoint();
    P.SetRectangular(Isis::Displacement(asp_point[0], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[1], Isis::Displacement::Meters),
                     Isis::Displacement(asp_point[2], Isis::Displacement::Meters),
                     Isis::Distance(sigma[0], Isis::Distance::Meters),
                     Isis::Distance(sigma[1], Isis::Distance::Meters),
                     Isis::Distance(sigma[2], Isis::Distance::Meters));
    point->SetAdjustedSurfacePoint(P);
    
    // Update the sigmas for measures from the ASP cnet
    int numMeasures = point->GetNumMeasures();
    int aspNumMeasures = cnet[i].size();
    if (numMeasures != aspNumMeasures)
      vw_throw(vw::ArgumentErr() 
               << "saveUpdatedIsisCnet: Book-keeping failure in number of measures.\n");
      
    int meas_id = 0;   
    for (auto m_iter = cnet[i].begin(); m_iter != cnet[i].end(); m_iter++) {
      vw::Vector2 sigma = m_iter->sigma();
      Isis::ControlMeasure *measurement = point->GetMeasure(meas_id);
      measurement->SetSampleSigma(sigma[0]);
      measurement->SetLineSigma(sigma[1]);

      meas_id++;
    }
  }
  
  // Save any GCP that were later added to the ASP cnet. This creates new
  // points rather than updating existing ones.
  for (int i = numControlPoints; i < param_storage.num_points(); i++) {
    
    vw::ba::ControlPoint const& cpoint = cnet[i];
    if (cpoint.type() != vw::ba::ControlPoint::GroundControlPoint)
      vw_throw(vw::ArgumentErr() 
               << "saveUpdatedIsisCnet: Book-keeping failure. Expected GCP.\n");
      
    addIsisControlPoint(*icnet.get(), cnet, param_storage, cameras, serialNumbers, i, 
                        numOutliers);    
  }
  
  vw::vw_out() << "Number of points in control network: " << icnet->GetNumPoints() << "\n";
  vw::vw_out() << "Number of outliers: " << numOutliers << std::endl;
  vw::vw_out() << "Target: " << icnet->GetTarget().toStdString() << std::endl;
  
  std::string cnetFile = outputPrefix + ".net";
  vw::vw_out() << "Writing ISIS control network: " << cnetFile << "\n"; 
  QString qCnetFile = QString::fromStdString(cnetFile);
  icnet->Write(qCnetFile);
  
#endif // ASP_HAVE_PKG_ISIS

  return;  
}

// Create from scratch and and save an ISIS cnet from a given control network
// and latest param values.
void saveIsisCnet(std::string const& outputPrefix, 
                  vw::cartography::Datum const& datum,
                  vw::ba::ControlNetwork const& cnet,
                  asp::BAParams const& param_storage) {
  
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  
  // Sanity check
  if (param_storage.num_points() != cnet.size())
    vw::vw_throw(vw::ArgumentErr() 
             << "saveIsisCnet: number of points in param_storage and cnet do not match.\n");

  // Find the serial numbers
  std::vector<std::string> image_files = cnet.get_image_list();
  std::vector<std::string> serialNumbers;
  readSerialNumbers(image_files, serialNumbers);
  // Read the cameras This may fail for non-ISIS images. Then use null cameras.
  std::vector<boost::shared_ptr<Isis::Camera>> cameras;
  readIsisCameras(image_files, cameras);
  
  // Initialize the isis cnet
  Isis::ControlNet icnet;
  setIsisCnetTarget(image_files[0], datum, icnet);
  icnet.SetNetworkId(QString::fromStdString("bundle_adjust"));
  icnet.SetUserName(QString::fromStdString("bundle_adjust"));
  icnet.SetDescription(QString::fromStdString("bundle_adjust"));

  // Add a given control point to the ISIS cnet. Update the outlier counter.
  int numOutliers = 0;
  for (int i = 0; i < cnet.size(); i++)
    addIsisControlPoint(icnet, cnet, param_storage, cameras, serialNumbers, i, 
                        numOutliers);    
  
  vw::vw_out() << "Number of points in control network: " << icnet.GetNumPoints() << "\n";
  vw::vw_out() << "Number of outliers: " << numOutliers << std::endl;
  vw::vw_out() << "Target: " << icnet.GetTarget().toStdString() << std::endl;

  std::string cnetFile = outputPrefix + ".net";
  vw::vw_out() << "Writing ISIS control network: " << cnetFile << "\n"; 
  icnet.Write(QString::fromStdString(cnetFile));

#endif // ASP_HAVE_PKG_ISIS

  return;
}

} // end namespace asp
