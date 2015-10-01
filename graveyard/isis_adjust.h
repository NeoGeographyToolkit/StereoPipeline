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


/// \file isis_adjust.h
///
/// Utility for performing bundle adjustment of ISIS3 cube files. This
/// is a highly experimental program and reading of the bundle
/// adjustment chapter is required before use of this program.

// Standard
#include <stdlib.h>
#include <iostream>
#include <iomanip>

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Core/Debugging.h>
#include <vw/BundleAdjustment.h>

// Ames Stereo Pipeline
#include <asp/IsisIO.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/ISIS/StereoSessionIsis.h>

// ISIS Bundle Adjustment Model
// This is the Bundle Adjustment model
template <unsigned positionParam, unsigned poseParam>
class IsisBundleAdjustmentModel : public vw::ba::ModelBase< IsisBundleAdjustmentModel < positionParam,
                                                                                                        poseParam >,
                                                                            (positionParam+poseParam), 3>{
  typedef vw::Vector<double, positionParam+poseParam> camera_vector_t;
  typedef vw::Vector<double, 3> point_vector_t;

  std::vector< boost::shared_ptr<vw::camera::IsisAdjustCameraModel> > m_cameras;
  boost::shared_ptr<vw::ba::ControlNetwork> m_network;
  std::vector<camera_vector_t> m_cam_vec;
  //std::vector<point_vector_t> b; // b is not required as we'll be
                                   // using the cnet for storage.
  std::vector<camera_vector_t> m_cam_target_vec;
  std::vector<point_vector_t> m_point_target_vec;
  std::vector< std::string > m_files;
  int m_num_pixel_observations;
  float m_spacecraft_position_sigma;
  float m_spacecraft_pose_sigma;
  float m_gcp_scalar;

public:

  IsisBundleAdjustmentModel( std::vector< boost::shared_ptr< vw::camera::IsisAdjustCameraModel> > const& camera_models,
                             boost::shared_ptr<vw::ba::ControlNetwork> network,
                             std::vector< std::string > input_names,
                             float const& spacecraft_position_sigma,
                             float const& spacecraft_pose_sigma, float const& gcp_scalar ) :
  m_cameras( camera_models ), m_network(network), m_cam_vec( camera_models.size() ),
    m_cam_target_vec( camera_models.size() ),
    m_point_target_vec( network->size() ), m_files( input_names ),
    m_spacecraft_position_sigma(spacecraft_position_sigma),
    m_spacecraft_pose_sigma(spacecraft_pose_sigma), m_gcp_scalar(gcp_scalar) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the camera and point vectors, storing the initial values.
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      // I'm using what is already in the IsisAdjust camera file as
      // the orginal starting point for the problem. This way I can
      // nudge it with error and see what it is doing for debuging
      boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
      boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

      m_cam_vec[j] = camera_vector_t();
      // Setting new equations defined by cam_j
      for (unsigned n = 0; n < posF->size(); ++n)
        m_cam_vec[j][n] = (*posF)[n];
      for (unsigned n = 0; n < poseF->size(); ++n)
        m_cam_vec[j][n + posF->size()] = (*poseF)[n];
      m_cam_target_vec[j] = m_cam_vec[j];
    }

    // Setting up B vectors
    for (unsigned i = 0; i < network->size(); ++i) {
      m_point_target_vec[i] = (*m_network)[i].position();
    }

    // Checking to see if this Control Network is compatible with
    // IsisBundleAdjustmentModel
    if ( !(*m_network)[0][0].is_pixels_dominant() )
      vw_out(vw::WarningMessage,"asp") << "WARNING: Control Network doesn't appear to be using pixels" << std::endl;

  }

  // Return a reference to the camera and point parameters.
  camera_vector_t cam_params( int j ) const { return m_cam_vec[j]; }
  point_vector_t point_params( int i ) const { return (*m_network)[i].position(); }
  void set_cam_params(int j, camera_vector_t const& cam_j) { m_cam_vec[j] = cam_j; }
  void set_point_params(int i, point_vector_t const& point_i) { (*m_network)[i].set_position(point_i); }

  // Return Initial parameters. (Used by the bundle adjuster )
  camera_vector_t cam_target( int j ) const { return m_cam_target_vec[j]; }
  point_vector_t point_target( int i ) const { return m_point_target_vec[i]; }

  // Return general sizes
  size_t num_cameras() const { return m_cam_vec.size(); }
  size_t num_points() const { return m_network->size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return pixel observations -> supposedly used by Bundlevis
  // eventually i think
  size_t num_observations_of_point ( const int& i ) const { return (*m_network)[i].size(); }
  size_t corresponding_camera_for_measure( const int& i, const int& m ) const {
    return (*m_network)[i][m].image_id();
  }

  // Return the covariance of the camera parameters for camera j.
  inline vw::Matrix<double, (positionParam+poseParam), (positionParam+poseParam)> cam_inverse_covariance ( unsigned /*j*/ ) const {
    vw::Matrix< double, (positionParam+poseParam), (positionParam+poseParam) > result;
    for ( unsigned i = 0; i <positionParam; ++i )
      result(i,i) = 1/pow(m_spacecraft_position_sigma,2);
    for ( unsigned i = positionParam; i < (positionParam+poseParam); ++i )
      result(i,i) = 1/pow(m_spacecraft_pose_sigma,2);
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline vw::Matrix<double, 3, 3> point_inverse_covariance ( unsigned i ) const {
    vw::Matrix< double, 3, 3> result;
    vw::Vector3 sigmas = (*m_network)[i].sigma();
    sigmas = m_gcp_scalar*sigmas;
    for ( unsigned u = 0; u < 3; ++u)
      result(u,u) = 1/(sigmas[u]*sigmas[u]);

    // It is assumed that the GCP is defined in a local tangent frame
    // (East-North-Up)
    float lon = atan2((*m_network)[i].position()[1],
                      (*m_network)[i].position()[0]);
    float clon = cos(lon);
    float slon = sin(lon);
    float radius = sqrt( (*m_network)[i].position()[0] *
                         (*m_network)[i].position()[0] +
                         (*m_network)[i].position()[1] *
                         (*m_network)[i].position()[1] +
                         (*m_network)[i].position()[2] *
                         (*m_network)[i].position()[2] );
    float z_over_radius = (*m_network)[i].position()[2]/radius;
    float sqrt_1_minus = sqrt(1-z_over_radius*z_over_radius);
    vw::Matrix< double, 3, 3> ecef_to_local;
    ecef_to_local(0,0) = -slon;
    ecef_to_local(0,1) = clon;
    ecef_to_local(1,0) = -z_over_radius*clon;
    ecef_to_local(1,1) = -z_over_radius*slon;
    ecef_to_local(1,2) = sqrt_1_minus;
    ecef_to_local(2,0) = sqrt_1_minus*clon;
    ecef_to_local(2,1) = sqrt_1_minus*slon;
    ecef_to_local(2,2) = z_over_radius;

    // Rotatation matrix I want to use is ecef_to_local^T
    // How to rotate a covariance matrix = R*E*R^T

    // Rotating inverse covariance matrix
    result = transpose(ecef_to_local)*result*ecef_to_local;

    return result;
  }

  // This is for writing isis_adjust file for later
  void write_adjustment( int j, std::string const& filename ) const {
    std::ofstream ostr( filename.c_str() );

    write_equation( ostr, m_cameras[j]->position_func() );
    write_equation( ostr, m_cameras[j]->pose_func() );

    ostr.close();
  }

  boost::shared_ptr< vw::camera::IsisAdjustCameraModel >
  adjusted_camera( int j ) const {
    // Adjusting position and pose equations
    boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

    // Setting new equations defined by cam_j
    for (unsigned n = 0; n < posF->size(); ++n)
      (*posF)[n] = m_cam_vec[j][n];
    for (unsigned n = 0; n < poseF->size(); ++n)
      (*poseF)[n] = m_cam_vec[j][n + posF->size()];

    return m_cameras[j];
  }

  std::vector< boost::shared_ptr< vw::camera::CameraModel > >
  adjusted_cameras() const {
    std::vector< boost::shared_ptr<vw::camera::CameraModel> > cameras;
    for ( unsigned j = 0; j < m_cameras.size(); j++)
      cameras.push_back( adjusted_camera(j) );
    return cameras;
  }

  // Given the 'cam_j' vector ( camera model paramters ) for the j'th
  // image, and the 'b' vector (3D point location ) for the i'th
  // point, return the location of point_i on imager j in
  // millimeters. !!Warning!! This gives data back in millimeters
  // which is different from most implementations.
  vw::Vector2 operator() ( unsigned /*i*/, unsigned j,
                           camera_vector_t const& cam_j,
                           point_vector_t const& point_i ) const {

    // Loading equations
    boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

    // Applying new equation constants
    for (unsigned n = 0; n < posF->size(); ++n)
      (*posF)[n] = cam_j[n];
    for (unsigned n = 0; n < poseF->size(); ++n)
      (*poseF)[n] = cam_j[n + posF->size()];

    // Performing the forward projection. This is specific to the
    // IsisAdjustCameraModel. The first argument is really just
    // passing the time instance to load up a pinhole model for.
    vw::Vector2 forward_projection =
      m_cameras[j]->point_to_pixel( point_i );

    // Giving back the pixel measurement.
    return forward_projection;
  }

  void parse_camera_parameters(camera_vector_t cam_j,
                               vw::Vector3 &position_correction,
                               vw::Vector3 &pose_correction) const {
    position_correction = subvector(cam_j, 0, 3);
    pose_correction = subvector(cam_j, 3, 3);
  }

  // Errors on the image plane
  std::string image_unit() const { return "px"; }
  inline double image_compare( vw::Vector2 const& meas,
                               vw::Vector2 const& obj ) {
    return norm_2(meas-obj);
  }
  inline double position_compare( camera_vector_t const& meas,
                                  camera_vector_t const& obj ) {
    return norm_2(subvector(meas,0,3)-subvector(obj,0,3));
  }
  inline double pose_compare( camera_vector_t const& meas,
                              camera_vector_t const& obj ) {
    return norm_2(subvector(meas,3,3)-subvector(obj,3,3));
  }
  inline double gcp_compare( point_vector_t const& meas,
                             point_vector_t const& obj ) {
    return norm_2(meas-obj);
  }

  // Give access to the control network
  boost::shared_ptr<vw::ba::ControlNetwork> control_network() const {
    return m_network;
  }

  void bundlevis_cameras_append(std::string const& filename ) const {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    for ( unsigned j = 0; j < this->num_cameras(); ++j ) {
      boost::shared_ptr<vw::camera::IsisAdjustCameraModel> camera =
        this->adjusted_camera(j);
      float center_sample = camera->samples()/2;

      for ( int i = 0; i < camera->lines(); i+=(camera->lines()/4) ) {
        vw::Vector3 position =
          camera->camera_center( vw::Vector2(center_sample,i) );
        vw::Quat pose =
          camera->camera_pose( vw::Vector2(center_sample,i) );
        ostr << std::setprecision(18) << j << "\t" << position[0] << "\t"
             << position[1] << "\t" << position[2] << "\t";
        ostr << pose[0] << "\t" << pose[1] << "\t"
             << pose[2] << "\t" << pose[3] << "\n";
      }
    }
  }

  void bundlevis_points_append(std::string const& filename ) const {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    unsigned i = 0;
    BOOST_FOREACH( vw::ba::ControlPoint const& cp, *m_network ) {
      ostr << i++ << std::setprecision(18) << "\t" << cp.position()[0] << "\t"
           << cp.position()[1] << "\t" << cp.position()[2] << "\n";
    }
  }
};

template<typename In, typename Out, typename Pred>
Out copy_if(In first, In last, Out res, Pred Pr)
{
  while (first != last)
  {
    if (Pr(*first))
      *res++ = *first;
    ++first;
  }
  return res;
}

// This sifts out from a vector of strings, a listing of GCPs.  This
// should be useful for those programs who accept their data in a mass
// input vector.
bool IsGCP( std::string const& name ) {
  return boost::iends_with(name,".gcp");
}

template <class IContainT, class OContainT>
void sort_out_gcp( IContainT& input, OContainT& output ) {
  copy_if( input.begin(), input.end(),
           std::back_inserter(output), IsGCP );
  typename IContainT::iterator new_end =
    std::remove_if(input.begin(), input.end(), IsGCP);
  input.erase(new_end,input.end());
}

// This sifts out from a vector of strings, a listing of input CNET
// GCPs. This should be useful for those programs who accept their data
// in a mass input vector.
bool IsGCPCnet( std::string const& name ) {
  return boost::iends_with(name,".net") || boost::iends_with(name,".cnet");
}

template <class IContainT, class OContainT>
void sort_out_gcpcnets( IContainT& input, OContainT& output ) {
  copy_if( input.begin(), input.end(),
           std::back_inserter(output), IsGCPCnet );
  typename IContainT::iterator new_end =
    std::remove_if(input.begin(), input.end(), IsGCPCnet );
  input.erase(new_end,input.end());
}
