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

// A synthetic linescan model as a wrapper around CSM. 
// TODO(oalexan1): This need not be its own class, 
// can just populate the CSM model.
class SyntheticLinescan: public asp::CsmModel {

public:
    
  // Constructor
  SyntheticLinescan(SatSimOptions const& opt, 
                      double orbit_len, 
                      vw::cartography::GeoReference const & georef,    
                      std::vector<vw::Vector3>      const & positions,
                      std::vector<vw::Matrix3x3>    const & cam2world):
                  m_opt(opt), m_orbit_len(orbit_len), m_georef(georef), 
                  m_positions(positions), m_cam2world(cam2world),
                  m_ls_model(NULL) {
      // Create the CSM model
      populateCsmModel();
  }

  virtual ~SyntheticLinescan() {}
  virtual std::string type() const { return "SyntheticLinescan"; }

  // Gives the camera position in world coordinates.    
  virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const {
      csm::ImageCoord csm_pix;
      asp::toCsmPixel(pix, csm_pix);

      double time = m_ls_model->getImageTime(csm_pix);
      csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);

      return vw::Vector3(ecef.x, ecef.y, ecef.z);
  }

  // Pixel_to_vector (camera direction in ECEF coordinates)
  virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const {
      csm::ImageCoord csm_pix;
      asp::toCsmPixel(pix, csm_pix);
      
      csm::EcefLocus locus = m_ls_model->imageToRemoteImagingLocus(csm_pix);
      return vw::Vector3(locus.direction.x, locus.direction.y, locus.direction.z);
  }

  // Go from ground to the camera
  vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {

      csm::EcefCoord ecef(point[0], point[1], point[2]);
      
      // Do not show warnings, it becomes too verbose
      double achievedPrecision = -1.0;
      csm::WarningList warnings;
      csm::WarningList * warnings_ptr = NULL;
      bool show_warnings = false;
      csm::ImageCoord csm_pix = m_ls_model->groundToImage(ecef, m_desired_precision,
                                                          &achievedPrecision, warnings_ptr);
      
      vw::Vector2 asp_pix;
      asp::fromCsmPixel(asp_pix, csm_pix);
      return asp_pix;
  }

  //  Camera pose at given pixel
  // Do not implement this for now, as it is not needed
  virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const {
    // This is not implemented for now for the CSM model
    vw::vw_throw(vw::NoImplErr() << "SyntheticLinescan: Camera pose not implemented.\n");
    return vw::Quaternion<double>();
  }

  // Allow finding the time at any line, even negative ones. Here a
  // simple slope-intercept formula is used rather than a table. 
  double get_time_at_line(double line) const {
      csm::ImageCoord csm_pix;
      asp::toCsmPixel(vw::Vector2(0, line), csm_pix);
      return m_ls_model->getImageTime(csm_pix);
  }

  // The pointing vector in sensor coordinates, before applying cam2world. This
  // is for testing purposes. Normally CSM takes care of this internally.
  vw::Vector3 get_local_pixel_to_vector(vw::Vector2 const& pix) const {

    vw::Vector3 result(pix[0] + m_detector_origin[0], 
                        m_detector_origin[1], 
                        m_ls_model->m_focalLength);

    // Make the direction have unit length
    result = normalize(result);

    return result;
}

// Write the CSM model state in .json format. This is not the same as the CSM
// ISD format, but it has all that is needed for stereo, mapprojection,
// and bundle adjustment.
void write(std::string const& filename) const {
  std::string modelState = m_ls_model->getModelState(); 
  vw::vw_out() << "Writing: " << filename << std::endl;
  std::ofstream ofs(filename.c_str());
  ofs << modelState << std::endl;
  ofs.close();
}

private:

  SatSimOptions               const& m_opt;
  std::vector<vw::Vector3>   const & m_positions;
  std::vector<vw::Matrix3x3> const & m_cam2world;
  vw::cartography::GeoReference m_georef; // make a local copy, safer that way
  double m_orbit_len; // in meters

  // Pointer to linescan sensor. It will be managed by CsmModel::m_gm_model.
  UsgsAstroLsSensorModel * m_ls_model;

  // These are used to find the look direction in camera coordinates at a given line
  vw::Vector2 m_detector_origin;

  void populateCsmModel() {

    // Do not use a precision below 1.0-e8 as then the linescan model will return junk.
    m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISISON;

    m_semi_major_axis = m_georef.datum().semi_major_axis();
    m_semi_minor_axis = m_georef.datum().semi_minor_axis();

    std::cout.precision(17);
    std::cout << "semi major is " << m_semi_major_axis << std::endl;
    std::cout << "semi minor is " << m_semi_minor_axis << std::endl;

    // Create the linescan model
    m_gm_model.reset(new UsgsAstroLsSensorModel); // m_gm_model will manage the deallocation
    m_ls_model = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get()); // pointer to ls model
    if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

    // This performs many initializations apart from the above
    m_ls_model->reset();

    // Override some initializations
    m_ls_model->m_nSamples = m_opt.image_size[0]; 
    m_ls_model->m_nLines   = m_opt.image_size[1];
    m_ls_model->m_platformFlag     = 1; // Use 1, for order 8 Lagrange interpolation
    m_ls_model->m_minElevation     = -10000.0; // -10 km
    m_ls_model->m_maxElevation     =  10000.0; //  10 km
    m_ls_model->m_focalLength      = m_opt.focal_length;
    m_ls_model->m_zDirection       = 1.0;
    m_ls_model->m_halfSwath        = 1.0;
    m_ls_model->m_sensorIdentifier = "SyntheticLinescan";
    m_ls_model->m_majorAxis        = m_semi_major_axis;
    m_ls_model->m_minorAxis        = m_semi_minor_axis;
    
    // The choices below are copied from the DigitalGlobe CSM linescan model.
    // Better to keep same convention than dig deep inside UsAstroLsSensorModel.
    // Also keep in mind that a CSM pixel has extra 0.5 added to it.
    m_detector_origin[0]                 = -m_opt.optical_center[0]; 
    m_detector_origin[1]                 = 0.0;
    m_ls_model->m_iTransL[0]             = 0.0;  
    m_ls_model->m_iTransL[1]             = 0.0;
    m_ls_model->m_iTransL[2]             = 1.0;
    m_ls_model->m_iTransS[0]             = 0.0;
    m_ls_model->m_iTransS[1]             = 1.0;
    m_ls_model->m_iTransS[2]             = 0.0;
    m_ls_model->m_detectorLineOrigin     = 0.0;
    m_ls_model->m_detectorSampleOrigin   = 0.0;
    m_ls_model->m_detectorLineSumming    = 1.0;
    m_ls_model->m_startingDetectorLine   = m_detector_origin[1];
    m_ls_model->m_detectorSampleSumming  = 1.0;
    m_ls_model->m_startingDetectorSample = (m_detector_origin[0] - 0.5);

    // Set the time. The first camera will be at time 0. The last camera
    // will be at time depending on distance traveled and speed.
    double beg_t = 0.0;
    double end_t = m_orbit_len / m_opt.velocity;
    double dt = (end_t - beg_t) / (m_opt.image_size[1] - 1.0);
    m_ls_model->m_intTimeLines.push_back(1.0); // to offset CSM's quirky 0.5 additions in places
    m_ls_model->m_intTimeStartTimes.push_back(beg_t);
    m_ls_model->m_intTimes.push_back(dt);

    // Positions and velocities
    int num_pos = m_positions.size();
    m_ls_model->m_numPositions = 3 * num_pos; // concatenate all coordinates
    m_ls_model->m_t0Ephem = beg_t;
    m_ls_model->m_dtEphem = (end_t - beg_t) / (num_pos - 1.0);

    m_ls_model->m_positions.resize(m_ls_model->m_numPositions);
    m_ls_model->m_velocities.resize(m_ls_model->m_numPositions);
    for (int pos_it = 0; pos_it < num_pos; pos_it++) {
      for (int coord = 0; coord < 3; coord++) {
        m_ls_model->m_positions [3*pos_it + coord] = m_positions[pos_it][coord];
        m_ls_model->m_velocities[3*pos_it + coord] = 0.0; // should not be used
      }
    }

    // Orientations
    m_ls_model->m_numQuaternions = 4 * m_cam2world.size();
    m_ls_model->m_t0Quat = beg_t;
    m_ls_model->m_dtQuat = (end_t - beg_t) / (m_cam2world.size() - 1.0);
 
    m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);
    for (int pos_it = 0; pos_it < m_ls_model->m_numQuaternions / 4; pos_it++) {
        auto m = m_cam2world[pos_it];
        // Convert to Eigen
        Eigen::Matrix3d M = eigenMatrix(m);
        // Convert to Eigen quaternion
        Eigen::Quaterniond q(M);

        // CSM wants quaternions as x, y, z, w.
        int coord = 0;
        m_ls_model->m_quaternions[4*pos_it + coord] = q.x(); coord++;
        m_ls_model->m_quaternions[4*pos_it + coord] = q.y(); coord++;
        m_ls_model->m_quaternions[4*pos_it + coord] = q.z(); coord++;
        m_ls_model->m_quaternions[4*pos_it + coord] = q.w(); coord++;
    }

    // Re-creating the model from the state forces some operations to
    // take place which are inaccessible otherwise.
    std::string modelState = m_ls_model->getModelState();
    m_ls_model->replaceModelState(modelState);
  }

}; // End class SyntheticLinescan

// Compare the camera center and direction with pinhole. A very useful
// test.
void PinLinescanTest(SatSimOptions              const & opt, 
                     SyntheticLinescan          const & ls_cam,
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
  SyntheticLinescan * ls_cam 
    = new SyntheticLinescan(opt, orbit_len, georef, positions, cam2world); 

  // Sanity check (very useful)
  // PinLinescanTest(opt, *ls_cam, positions, cam2world);

  std::string filename = opt.out_prefix + ".json";
  ls_cam->write(filename);

  // Save the camera name and smart pointer to camera
  cam_names.push_back(filename);
  cams.push_back(vw::CamPtr(ls_cam));

  return;
}

// A function to read Linescan cameras from disk. There will
// be just one of them, but same convention is kept as for Pinhole
// where there is many of them. Note that the camera is created as of CSM type,
// rather than SyntheticLinescan type. This is not important as we will
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

