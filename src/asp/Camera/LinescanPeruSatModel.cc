// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Core/StereoSettings.h>
#include <asp/Camera/PeruSatXML.h>
#include <asp/Camera/LinescanPeruSatModel.h>
#include <asp/Camera/CsmUtils.h>

#include <vw/Camera/OrbitalCorrections.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

namespace asp {

using vw::Vector3;

// CSM-based PeruSat camera model constructor
PeruSatCsmCameraModel::
PeruSatCsmCameraModel(double time_t0, double time_dt,
                      std::vector<vw::Vector3> const& positions,
                      std::vector<vw::Vector3> const& velocities,
                      double pos_t0, double pos_dt,
                      std::vector<vw::Quaternion<double>> const& quaternions,
                      double quat_t0, double quat_dt,
                      vw::Vector2                        const& tan_psi_x,
                      vw::Vector2                        const& tan_psi_y,
                      vw::Quaternion<double>             const& instrument_biases,
                      vw::Vector2i                       const& image_size,
                      double min_time, double max_time):
  m_time_t0(time_t0), m_time_dt(time_dt),
  m_positions(positions), m_velocities(velocities),
  m_pos_t0(pos_t0), m_pos_dt(pos_dt),
  m_quaternions(quaternions),
  m_quat_t0(quat_t0), m_quat_dt(quat_dt),
  m_tan_psi_x(tan_psi_x), m_tan_psi_y(tan_psi_y),
  m_instrument_biases(instrument_biases),
  m_image_size(image_size),
  m_min_time(min_time), m_max_time(max_time) {

  populateCsmModel();
}

// Populate the CSM linescan model from the raw PeruSat data.
// This follows the same pattern as PleiadesCameraModel::populateCsmModel().
void PeruSatCsmCameraModel::populateCsmModel() {

  // Populate CsmModel class members
  m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  vw::cartography::Datum datum("WGS84");
  m_semi_major_axis = datum.semi_major_axis();
  m_semi_minor_axis = datum.semi_minor_axis();

  // Create the linescan model
  m_gm_model = boost::make_shared<UsgsAstroLsSensorModel>();
  m_ls_model = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get());
  if (m_ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

  // This performs many initializations apart from the above
  m_ls_model->reset();

  // Override some initializations
  m_ls_model->m_nSamples = m_image_size[0];
  m_ls_model->m_nLines   = m_image_size[1];
  m_ls_model->m_platformFlag = 0; // order 4 Lagrange, matching the PeruSat doc and VW resampling
  m_ls_model->m_maxElevation =  10000.0; //  10 km
  m_ls_model->m_minElevation = -10000.0; // -10 km
  m_ls_model->m_focalLength  = 1.0;
  m_ls_model->m_zDirection   = 1.0;
  m_ls_model->m_halfSwath    = 1.0;
  m_ls_model->m_sensorIdentifier = "PeruSat";
  m_ls_model->m_majorAxis = m_semi_major_axis;
  m_ls_model->m_minorAxis = m_semi_minor_axis;

  m_ls_model->m_iTransL[0] = 0.0;
  m_ls_model->m_iTransL[1] = 1.0; // no scale
  m_ls_model->m_iTransL[2] = 0.0; // no skew
  m_ls_model->m_iTransS[0] = 0.0;
  m_ls_model->m_iTransS[1] = 0.0; // no skew
  m_ls_model->m_iTransS[2] = 1.0; // no scale

  m_ls_model->m_startingDetectorLine   = 0.0;
  m_ls_model->m_startingDetectorSample = 0.0;

  // PeruSat look angle formula:
  //   psi_x = tan_psi_x[0] * col + tan_psi_x[1]
  //   psi_y = tan_psi_y[0] * col + tan_psi_y[1]
  //   look = (tan(psi_y), -tan(psi_x), 1)
  //
  // In CSM, the linear detector model produces psi (the angle), and the
  // tan() is replaced by a transverse distortion polynomial from usgscsm.
  // The coefficients encode atan() as a 3rd-order Taylor polynomial: u - u^3/3.
  // For PeruSat angles (~0.01 rad), the approximation error is ~1e-12.
  //
  // The instrument biases quaternion is folded into each attitude quaternion.

  // Set detector params to give psi directly (the angle, not tangent).
  //
  // CSM image-to-ground computes (with identity iTransL/iTransS):
  //   detSample = (col+0.5) * sampleSumming + startingSample
  //   distortedY = detSample - sampleOrigin
  //   distortedX = 0 - lineOrigin = -lineOrigin (constant, detLine=0 for linescan)
  //   removeDistortion(distortedX, distortedY) -> (undistortedX, undistortedY)
  //   Look direction is proportional to (undistortedX, undistortedY, 1).
  //
  // We need: look = (tan(psi_y), -tan(psi_x), 1).
  // So: undistortedX = tan(psi_y), undistortedY = -tan(psi_x).
  // With transverse atan() distortion (odd function):
  //   distortedX = psi_y, distortedY = -psi_x
  //
  // For distortedY = -psi_x = -(tan_psi_x[0]*col + tan_psi_x[1]):
  //   distortedY = (col+0.5)*sampleSumming - sampleOrigin
  //              = -tan_psi_x[0]*col - tan_psi_x[1]
  //   sampleSumming = -tan_psi_x[0]
  //   sampleOrigin = 0.5*(-tan_psi_x[0]) + tan_psi_x[1]
  //                = tan_psi_x[1] - 0.5*tan_psi_x[0]
  m_ls_model->m_detectorSampleSumming = -m_tan_psi_x[0];
  m_ls_model->m_detectorSampleOrigin  = m_tan_psi_x[1]
    - 0.5 * m_tan_psi_x[0];

  // For distortedX = psi_y = tan_psi_y[1] (constant, slope is zero):
  //   -lineOrigin = tan_psi_y[1], so lineOrigin = -tan_psi_y[1]
  m_ls_model->m_detectorLineSumming  = 1.0;
  m_ls_model->m_detectorLineOrigin   = -m_tan_psi_y[1];

  // Transverse distortion: 20 coefficients (first 10 for x, last 10 for y).
  // Monomials: [1, ux, uy, ux^2, ux*uy, uy^2, ux^3, ux^2*uy, ux*uy^2, uy^3]
  // For x: dx = ux - ux^3/3  (atan(ux) Taylor approximation)
  // For y: dy = uy - uy^3/3
  m_ls_model->m_distortionType = DistortionType::TRANSVERSE;
  m_ls_model->m_opticalDistCoeffs.resize(20, 0.0);
  // x coefficients: [0, 1, 0, 0, 0, 0, -1/3, 0, 0, 0]
  m_ls_model->m_opticalDistCoeffs[1] = 1.0;
  m_ls_model->m_opticalDistCoeffs[6] = -1.0 / 3.0;
  // y coefficients: [0, 0, 1, 0, 0, 0, 0, 0, 0, -1/3]
  m_ls_model->m_opticalDistCoeffs[12] = 1.0;
  m_ls_model->m_opticalDistCoeffs[19] = -1.0 / 3.0;

  // Time
  m_ls_model->m_intTimeLines.push_back(1.0); // offset CSM's quirky 0.5 additions
  m_ls_model->m_intTimeStartTimes.push_back(m_time_t0);
  m_ls_model->m_intTimes.push_back(m_time_dt);

  int num_pos = m_positions.size();
  if ((size_t)num_pos != m_velocities.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many positions as velocities.\n");

  // Positions and velocities
  m_ls_model->m_numPositions = 3 * num_pos;
  m_ls_model->m_t0Ephem = m_pos_t0;
  m_ls_model->m_dtEphem = m_pos_dt;
  m_ls_model->m_positions.resize(m_ls_model->m_numPositions);
  m_ls_model->m_velocities.resize(m_ls_model->m_numPositions);
  for (int pos_it = 0; pos_it < num_pos; pos_it++) {
    for (int coord = 0; coord < 3; coord++) {
      m_ls_model->m_positions [3*pos_it + coord] = m_positions[pos_it][coord];
      m_ls_model->m_velocities[3*pos_it + coord] = m_velocities[pos_it][coord];
    }
  }

  // Quaternions. PeruSat provides tabulated quaternions at uniform time steps.
  // Fold the instrument biases into each quaternion so CSM sees the combined
  // rotation (body-to-sensor including biases).
  int num_quats = m_quaternions.size();
  m_ls_model->m_numQuaternions = 4 * num_quats;
  m_ls_model->m_t0Quat = m_quat_t0;
  m_ls_model->m_dtQuat = m_quat_dt;
  m_ls_model->m_quaternions.resize(m_ls_model->m_numQuaternions);

  vw::Quaternion<double> inv_biases = inverse(m_instrument_biases);
  for (int qi = 0; qi < num_quats; qi++) {
    // Compose: apply instrument biases to convert from sensor to body,
    // then the attitude quaternion from body to ECEF.
    // The old code does: result = inv_biases.rotate(look), then
    //   pose.rotate(result). This is equivalent to (pose * inv_biases)(look).
    // CSM applies the quaternion to the look vector directly, so store
    // the composed quaternion.
    vw::Quat q = m_quaternions[qi] * inv_biases;
    q = normalize(q);

    // ASP stores (w, x, y, z). CSM wants (x, y, z, w).
    m_ls_model->m_quaternions[4*qi + 0] = q.x();
    m_ls_model->m_quaternions[4*qi + 1] = q.y();
    m_ls_model->m_quaternions[4*qi + 2] = q.z();
    m_ls_model->m_quaternions[4*qi + 3] = q.w();
  }

  // Normalize and fix sign flips
  asp::normalizeQuaternions(m_ls_model);

  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = m_ls_model->getModelState();
  m_ls_model->replaceModelState(modelState);

  // Adjust the CSM model for velocity aberration and atmospheric
  // refraction if desired.
  double local_earth_radius = vw::DEFAULT_EARTH_RADIUS;
  double mean_ground_elevation = vw::DEFAULT_SURFACE_ELEVATION;
  asp::orbitalCorrections(this,
                          asp::stereo_settings().enable_velocity_aberration_correction,
                          asp::stereo_settings().enable_atmospheric_refraction_correction,
                          local_earth_radius, mean_ground_elevation);
}

// CSM-delegating virtual functions (same pattern as Pleiades)
vw::Vector3 PeruSatCsmCameraModel::camera_center(vw::Vector2 const& pix) const {
  csm::ImageCoord csm_pix;
  asp::toCsmPixel(pix, csm_pix);
  double time = m_ls_model->getImageTime(csm_pix);
  csm::EcefCoord ecef = m_ls_model->getSensorPosition(time);
  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}

vw::Vector3 PeruSatCsmCameraModel::pixel_to_vector(vw::Vector2 const& pix) const {
  csm::ImageCoord csm_pix;
  asp::toCsmPixel(pix, csm_pix);
  csm::EcefLocus locus = m_ls_model->imageToRemoteImagingLocus(csm_pix);
  return vw::Vector3(locus.direction.x, locus.direction.y, locus.direction.z);
}

vw::Vector2 PeruSatCsmCameraModel::point_to_pixel(vw::Vector3 const& point) const {
  csm::EcefCoord ecef(point[0], point[1], point[2]);
  double achievedPrecision = -1.0;
  csm::WarningList * warnings_ptr = NULL;
  csm::ImageCoord csm_pix = m_ls_model->groundToImage(ecef, m_desired_precision,
                                                       &achievedPrecision, warnings_ptr);
  vw::Vector2 asp_pix;
  asp::fromCsmPixel(asp_pix, csm_pix);
  return asp_pix;
}

// Load PeruSat camera as a CSM linescan model.
// Resampling is done here (not in PeruSatXML) so the XML class returns raw data.
// Positions/velocities: Lagrange order 4 (degree 3).
// Quaternions: SLERP.
boost::shared_ptr<PeruSatCsmCameraModel>
load_perusat_camera_model_from_xml(std::string const& path) {

  vw_out(vw::DebugMessage, "asp") << "Loading PeruSat camera file: " << path << "\n";

  PeruSatXML xml_reader;
  xml_reader.read_xml(path);

  // Extract raw (unresampled) data from the XML
  double time_t0 = 0, time_dt = 0;
  std::vector<Vector3> raw_positions, raw_velocities;
  double raw_pos_t0 = 0, raw_pos_dt = 0;
  std::vector<vw::Quaternion<double>> raw_quaternions;
  double raw_quat_t0 = 0, raw_quat_dt = 0;
  double min_time = 0, max_time = 0;
  xml_reader.extractRawData(time_t0, time_dt,
                            raw_positions, raw_velocities,
                            raw_pos_t0, raw_pos_dt,
                            raw_quaternions, raw_quat_t0, raw_quat_dt,
                            min_time, max_time);

  // Resample to a denser grid so CSM's Lagrange interpolation
  // closely reproduces VW's interpolation on the sparse raw data.
  int factor = 10; // resample to 10x denser grid
  int order = 4;   // Lagrange interpolation order (4 points, degree 3)

  // Resample positions and velocities with Lagrange (same as old VW path)
  std::vector<double> pos_times_raw, vel_times_raw;
  for (size_t i = 0; i < raw_positions.size(); i++)
    pos_times_raw.push_back(raw_pos_t0 + i * raw_pos_dt);
  for (size_t i = 0; i < raw_velocities.size(); i++)
    vel_times_raw.push_back(raw_pos_t0 + i * raw_pos_dt);

  std::vector<double> pos_out_times, vel_out_times;
  std::vector<Vector3> positions, velocities;
  asp::resampleVec3Lagrange(pos_times_raw, raw_positions, order, factor,
                            pos_out_times, positions);
  asp::resampleVec3Lagrange(vel_times_raw, raw_velocities, order, factor,
                            vel_out_times, velocities);

  double pos_t0 = pos_out_times.front();
  double pos_dt = (pos_out_times.back() - pos_out_times.front()) /
                  (pos_out_times.size() - 1.0);

  // Resample quaternions with SLERP (same as old VW path)
  std::vector<double> quat_times_raw;
  for (size_t i = 0; i < raw_quaternions.size(); i++)
    quat_times_raw.push_back(raw_quat_t0 + i * raw_quat_dt);

  std::vector<double> quat_out_times;
  std::vector<vw::Quaternion<double>> quaternions;
  asp::resampleQuatSlerp(quat_times_raw, raw_quaternions, factor,
                         quat_out_times, quaternions);

  double quat_t0 = quat_out_times.front();
  double quat_dt = (quat_out_times.back() - quat_out_times.front()) /
                   (quat_out_times.size() - 1.0);

  // Update min/max time based on resampled data
  min_time = std::max(pos_out_times.front(),
                      std::max(vel_out_times.front(), quat_out_times.front()));
  max_time = std::min(pos_out_times.back(),
                      std::min(vel_out_times.back(), quat_out_times.back()));

  boost::shared_ptr<PeruSatCsmCameraModel> cam
    (new PeruSatCsmCameraModel(time_t0, time_dt,
                               positions, velocities,
                               pos_t0, pos_dt,
                               quaternions, quat_t0, quat_dt,
                               xml_reader.m_tan_psi_x,
                               xml_reader.m_tan_psi_y,
                               xml_reader.m_instrument_biases,
                               xml_reader.m_image_size,
                               min_time, max_time));
  return cam;
}

} // end namespace asp

