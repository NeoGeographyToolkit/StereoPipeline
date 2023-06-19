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

#include <asp/Camera/CsmModel.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Core/Common.h>
#include <asp/Core/SatSim.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

#include <vw/Camera/PinholeModel.h>

#include <Eigen/Geometry>

namespace asp {

// A function to convert a 3x3 VW matrix to Eigen
Eigen::Matrix3d eigenMatrix(vw::Matrix3x3 const& m) {
  Eigen::Matrix3d result;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      result(r, c) = m(r, c);
  return result;
}

// Populate the CSM model with the given camera positions and orientations.
void populateSyntheticLinescan(SatSimOptions const& opt, 
                      double orbit_len, 
                      vw::cartography::GeoReference const & georef,    
                      std::vector<vw::Vector3>      const & positions,
                      std::vector<vw::Matrix3x3>    const & cam2world,
                      // Outputs
                      asp::CsmModel & model) {

  // Do not use a precision below 1.0-e8 as then the linescan model will return junk.
  model.m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISISON;

  model.m_semi_major_axis = georef.datum().semi_major_axis();
  model.m_semi_minor_axis = georef.datum().semi_minor_axis();

  std::cout.precision(17);
  std::cout << "semi major is " << model.m_semi_major_axis << std::endl;
  std::cout << "semi minor is " << model.m_semi_minor_axis << std::endl;

  // Create the linescan model
  model.m_gm_model.reset(new UsgsAstroLsSensorModel); // m_gm_model will manage the deallocation
  UsgsAstroLsSensorModel* ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>(model.m_gm_model.get()); // pointer to ls model
  if (ls_model == NULL)
  vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

  // This performs many initializations apart from the above
  ls_model->reset();

    // Override some initializations
    ls_model->m_nSamples         = opt.image_size[0]; 
    ls_model->m_nLines           = opt.image_size[1];
    ls_model->m_platformFlag     = 1; // Use 1, for order 8 Lagrange interpolation
    ls_model->m_minElevation     = -10000.0; // -10 km
    ls_model->m_maxElevation     =  10000.0; //  10 km
    ls_model->m_focalLength      = opt.focal_length;
    ls_model->m_zDirection       = 1.0;
    ls_model->m_halfSwath        = 1.0;
    ls_model->m_sensorIdentifier = "SyntheticLinescan";
    ls_model->m_majorAxis        = model.m_semi_major_axis;
    ls_model->m_minorAxis        = model.m_semi_minor_axis;
    
    // The choices below are copied from the DigitalGlobe CSM linescan model.
    // Better to keep same convention than dig deep inside UsAstroLsSensorModel.
    // Also keep in mind that a CSM pixel has extra 0.5 added to it.
    vw::Vector2 detector_origin;
    detector_origin[0]                 = -opt.optical_center[0]; 
    detector_origin[1]                 = 0.0;
    ls_model->m_iTransL[0]             = 0.0;  
    ls_model->m_iTransL[1]             = 0.0;
    ls_model->m_iTransL[2]             = 1.0;
    ls_model->m_iTransS[0]             = 0.0;
    ls_model->m_iTransS[1]             = 1.0;
    ls_model->m_iTransS[2]             = 0.0;
    ls_model->m_detectorLineOrigin     = 0.0;
    ls_model->m_detectorSampleOrigin   = 0.0;
    ls_model->m_detectorLineSumming    = 1.0;
    ls_model->m_startingDetectorLine   = detector_origin[1];
    ls_model->m_detectorSampleSumming  = 1.0;
    ls_model->m_startingDetectorSample = (detector_origin[0] - 0.5);

    // Set the time. The first camera will be at time 0. The last camera
    // will be at time depending on distance traveled and speed.
    double beg_t = 0.0;
    double end_t = orbit_len / opt.velocity;
    double dt = (end_t - beg_t) / (opt.image_size[1] - 1.0);
    ls_model->m_intTimeLines.push_back(1.0); // to offset CSM's quirky 0.5 additions in places
    ls_model->m_intTimeStartTimes.push_back(beg_t);
    ls_model->m_intTimes.push_back(dt);

    // Positions and velocities
    int num_pos = positions.size();
    ls_model->m_numPositions = 3 * num_pos; // concatenate all coordinates
    ls_model->m_t0Ephem = beg_t;
    ls_model->m_dtEphem = (end_t - beg_t) / (num_pos - 1.0);

    ls_model->m_positions.resize(ls_model->m_numPositions);
    ls_model->m_velocities.resize(ls_model->m_numPositions);
    for (int pos_it = 0; pos_it < num_pos; pos_it++) {
      for (int coord = 0; coord < 3; coord++) {
        ls_model->m_positions [3*pos_it + coord] = positions[pos_it][coord];
        ls_model->m_velocities[3*pos_it + coord] = 0.0; // should not be used
      }
    }

    // Orientations
    ls_model->m_numQuaternions = 4 * cam2world.size();
    ls_model->m_t0Quat = beg_t;
    ls_model->m_dtQuat = (end_t - beg_t) / (cam2world.size() - 1.0);
 
    ls_model->m_quaternions.resize(ls_model->m_numQuaternions);
    for (int pos_it = 0; pos_it < ls_model->m_numQuaternions / 4; pos_it++) {
        auto m = cam2world[pos_it];
        // Convert to Eigen
        Eigen::Matrix3d M = eigenMatrix(m);
        // Convert to Eigen quaternion
        Eigen::Quaterniond q(M);

        // CSM wants quaternions as x, y, z, w.
        int coord = 0;
        ls_model->m_quaternions[4*pos_it + coord] = q.x(); coord++;
        ls_model->m_quaternions[4*pos_it + coord] = q.y(); coord++;
        ls_model->m_quaternions[4*pos_it + coord] = q.z(); coord++;
        ls_model->m_quaternions[4*pos_it + coord] = q.w(); coord++;
    }

    // Re-creating the model from the state forces some operations to
    // take place which are inaccessible otherwise.
    std::string modelState = ls_model->getModelState();
    ls_model->replaceModelState(modelState);
  }

  // Allow finding the time at any line, even negative ones. Here a
  // simple slope-intercept formula is used rather than a table. 
  // This was a temporary function used for debugging
  // double get_time_at_line(double line) const {
  //     csm::ImageCoord csm_pix;
  //     asp::toCsmPixel(vw::Vector2(0, line), csm_pix);
  //     return ls_model->getImageTime(csm_pix);
  // }

  // The pointing vector in sensor coordinates, before applying cam2world. This
  // is for testing purposes. Normally CSM takes care of this internally.
  // This was a temporary function used for debugging
  // vw::Vector3 get_local_pixel_to_vector(vw::Vector2 const& pix) const {

  //   vw::Vector3 result(pix[0] + detector_origin[0], 
  //                       detector_origin[1], 
  //                       ls_model->m_focalLength);
  //   // Make the direction have unit length
  //   result = normalize(result);
  //   return result;
  // }


// Compare the camera center and direction with pinhole. A very useful
// test.
void PinLinescanTest(SatSimOptions              const & opt, 
                     asp::CsmModel              const & ls_cam,
                     std::vector<vw::Vector3>   const & positions,
                     std::vector<vw::Matrix3x3> const & cam2world) {
                        
  for (int i = 0; i < int(positions.size()); i++) {

    auto pin_cam = vw::camera::PinholeModel(positions[i], cam2world[i],
                                  opt.focal_length, opt.focal_length,
                                  opt.optical_center[0], opt.optical_center[1]);
  
    double line = (opt.image_size[1] - 1.0) * i / (positions.size() - 1.0);
  
    // Need care here
    vw::Vector2 pin_pix(opt.optical_center[0], opt.optical_center[1]);
    vw::Vector2 ls_pix (opt.optical_center[0], line);

    // The differences below must be 0
    vw::Vector3 ls_ctr  = ls_cam.camera_center(ls_pix);
    vw::Vector3 pin_ctr = pin_cam.camera_center(pin_pix);
    std::cout << "ls ctr and and pin - ls ctr diff: " << ls_ctr << " "
              << norm_2(pin_ctr - ls_ctr) << std::endl;

    vw::Vector3 ls_dir = ls_cam.pixel_to_vector(ls_pix);
    vw::Vector3 pin_dir = pin_cam.pixel_to_vector(pin_pix);
    std::cout << "ls dir and pin - ls dir diff: " << ls_dir << " "
              << norm_2(pin_dir - ls_dir) << std::endl;
  }
}

// Create and save a linescan camera with given camera positions and orientations.
// There will be just one of them, as all poses are part of the same linescan camera.
void genLinescanCameras(SatSimOptions                 const & opt, 
                        double                                orbit_len, 
                        vw::cartography::GeoReference const & georef,    
                        std::vector<vw::Vector3>      const & positions,
                        std::vector<vw::Matrix3x3>    const & cam2world,
                        // Outputs
                        std::vector<std::string>              & cam_names,
                        std::vector<vw::CamPtr>               & cams) {

  // Initialize the outputs
  cam_names.clear();
  cams.clear();

  // Create the camera. Will be later owned by a smart pointer.
  asp::CsmModel * ls_cam = new asp::CsmModel;
  populateSyntheticLinescan(opt, orbit_len, georef, positions, cam2world, *ls_cam); 

  // Sanity check (very useful)
  // PinLinescanTest(opt, *ls_cam, positions, cam2world);

  std::string filename = opt.out_prefix + ".json";
  ls_cam->saveState(filename);

  // Save the camera name and smart pointer to camera
  cam_names.push_back(filename);
  cams.push_back(vw::CamPtr(ls_cam));

  return;
}

// A function to read Linescan cameras from disk. There will
// be just one of them, but same convention is kept as for Pinhole
// where there is many of them. Note that the camera is created as of CSM type,
// rather than asp::CsmModel type. This is not important as we will
// abstract it right away to the base class.
void readLinescanCameras(SatSimOptions const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::CamPtr> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << std::endl;
  asp::read_list(opt.camera_list, cam_names);

  // Sanity checks
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");
  if (cam_names.size() != 1)
    vw::vw_throw(vw::ArgumentErr() << "Only one linescan camera is expected.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i] = vw::CamPtr(new asp::CsmModel(cam_names[i]));

  return;
}

} // end namespace asp

