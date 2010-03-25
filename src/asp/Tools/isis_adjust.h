// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
#include <vw/Camera/ControlNetwork.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Core/Debugging.h>

// Ames Stereo Pipeline
#include <asp/Core/ControlNetworkLoader.h>
#include <asp/IsisIO.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/ISIS/StereoSessionIsis.h>

namespace vw {
namespace camera {

// ISIS Bundle Adjustment Model
// This is the Bundle Adjustment model
template <unsigned positionParam, unsigned poseParam>
class IsisBundleAdjustmentModel : public camera::BundleAdjustmentModelBase< IsisBundleAdjustmentModel < positionParam,
                                                                                                        poseParam >,
                                                                            (positionParam+poseParam), 3>{
  typedef Vector<double, positionParam+poseParam> camera_vector_t;
  typedef Vector<double, 3> point_vector_t;

  std::vector< boost::shared_ptr<IsisAdjustCameraModel> > m_cameras;
  boost::shared_ptr<ControlNetwork> m_network;
  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_initial;
  std::vector<point_vector_t> b_initial;
  std::vector< std::string > m_files;
  int m_num_pixel_observations;
  float m_spacecraft_position_sigma;
  float m_spacecraft_pose_sigma;
  float m_gcp_scalar;

public:

  IsisBundleAdjustmentModel( std::vector< boost::shared_ptr< vw::camera::IsisAdjustCameraModel> > const& camera_models,
                             boost::shared_ptr<ControlNetwork> network,
                             std::vector< std::string > input_names,
                             float const& spacecraft_position_sigma,
                             float const& spacecraft_pose_sigma, float const& gcp_scalar ) :
  m_cameras( camera_models ), m_network(network), a( camera_models.size() ),
    b( network->size()), a_initial( camera_models.size() ),
    b_initial( network->size() ), m_files( input_names ),
    m_spacecraft_position_sigma(spacecraft_position_sigma),
    m_spacecraft_pose_sigma(spacecraft_pose_sigma), m_gcp_scalar(gcp_scalar) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the A and B vectors, storing the initial values.
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      // I'm using what is already in the IsisAdjust camera file as
      // the orginal starting point for the problem. This way I can
      // nudge it with error and see what it is doing for debuging
      boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
      boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

      a[j] = camera_vector_t();
      // Setting new equations defined by a_j
      for (unsigned n = 0; n < posF->size(); ++n)
        a[j][n] = (*posF)[n];
      for (unsigned n = 0; n < poseF->size(); ++n)
        a[j][n + posF->size()] = (*poseF)[n];
      a_initial[j] = a[j];
    }

    // Setting up B vectors
    for (unsigned i = 0; i < network->size(); ++i) {
      b[i] = (*m_network)[i].position();
      b_initial[i] = b[i];
    }

    // Checking to see if this Control Network is compatible with
    // IsisBundleAdjustmentModel
    if ( !(*m_network)[0][0].is_pixels_dominant() )
      vw_out(WarningMessage,"asp") << "WARNING: Control Network doesn't appear to be using pixels" << std::endl;

  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters( int j ) const { return a[j]; }
  point_vector_t B_parameters( int i ) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) { a[j] = a_j; }
  void set_B_parameters(int i, point_vector_t const& b_i) { b[i] = b_i; }

  // Approximate the jacobian for small variations in the a_j
  // parameters ( camera parameters ).
  inline Matrix<double, 2, positionParam+poseParam> A_jacobian( unsigned i, unsigned j,
                                                                camera_vector_t const& a_j,
                                                                point_vector_t const& b_i ) {
    // Old implementation
    Matrix<double> partial_derivatives = camera::BundleAdjustmentModelBase< IsisBundleAdjustmentModel, positionParam+poseParam, 3>::A_jacobian(i, j, a_j, b_i);

    //TODO: Give me an analytical solution again, remember the
    //calculation of euler angles must match Michael's implementation

    return partial_derivatives;
  }

  // Analytically computed jacobian for variations in the b_i
  // parameters ( 3d point locations ).
  inline Matrix<double, 2, 3> B_jacobian ( unsigned i, unsigned j,
                                           camera_vector_t const& a_j,
                                           point_vector_t const& b_i ) {
    Matrix<double> partial_derivatives(2,3);
    partial_derivatives = camera::BundleAdjustmentModelBase< IsisBundleAdjustmentModel, positionParam+poseParam, 3>::B_jacobian(i, j, a_j, b_i);

    //TODO: Analytical solution

    return partial_derivatives;
  }

  // Return Initial parameters. (Used by the bundle adjuster )
  camera_vector_t A_initial( int j ) const { return a_initial[j]; }
  point_vector_t B_initial( int i ) const { return b_initial[i]; }

  // Return general sizes
  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return pixel observations -> supposedly used by Bundlevis
  // eventually i think
  unsigned num_observations_of_point ( const int& i ) const { return (*m_network)[i].size(); }
  unsigned corresponding_camera_for_measure( const int& i, const int& m ) {
    return (*m_network)[i][m].image_id();
  }

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double, (positionParam+poseParam), (positionParam+poseParam)> A_inverse_covariance ( unsigned /*j*/ ) {
    Matrix< double, (positionParam+poseParam), (positionParam+poseParam) > result;
    for ( unsigned i = 0; i <positionParam; ++i )
      result(i,i) = 1/pow(m_spacecraft_position_sigma,2);
    for ( unsigned i = positionParam; i < (positionParam+poseParam); ++i )
      result(i,i) = 1/pow(m_spacecraft_pose_sigma,2);
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double, 3, 3> B_inverse_covariance ( unsigned i ) {
    Matrix< double, 3, 3> result;
    Vector3 sigmas = (*m_network)[i].sigma();
    sigmas = m_gcp_scalar*sigmas;
    for ( unsigned u = 0; u < 3; ++u)
      result(u,u) = 1/(sigmas[u]*sigmas[u]);

    // It is assumed that the GCP is defined in a local tangent frame
    // (East-North-Up)
    float lon = atan2(b[i][1],b[i][0]);
    float clon = cos(lon);
    float slon = sin(lon);
    float radius = sqrt( b[i][0]*b[i][0] +
                         b[i][1]*b[i][1] +
                         b[i][2]*b[i][2] );
    float z_over_radius = b[i][2]/radius;
    float sqrt_1_minus = sqrt(1-z_over_radius*z_over_radius);
    Matrix< double, 3, 3> ecef_to_local;
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
  void write_adjustment( int j, std::string const& filename ) {
    std::ofstream ostr( filename.c_str() );

    write_equation( ostr, m_cameras[j]->position_func() );
    write_equation( ostr, m_cameras[j]->pose_func() );

    ostr.close();
  }

  std::vector< boost::shared_ptr< CameraModel > > adjusted_cameras() {

    std::vector< boost::shared_ptr<camera::CameraModel> > cameras;
    for ( unsigned i = 0; i < m_cameras.size(); i++) {
      cameras.push_back(boost::shared_dynamic_cast<CameraModel>(m_cameras[i]));
    }
    return cameras;
  }

  boost::shared_ptr< IsisAdjustCameraModel > adjusted_camera( int j ) {

    // Adjusting position and pose equations
    boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

    // Setting new equations defined by a_j
    for (unsigned n = 0; n < posF->size(); ++n)
      (*posF)[n] = a[j][n];
    for (unsigned n = 0; n < poseF->size(); ++n)
      (*poseF)[n] = a[j][n + posF->size()];

    return m_cameras[j];
  }

  // Given the 'a' vector ( camera model paramters ) for the j'th
  // image, and the 'b' vector (3D point location ) for the i'th
  // point, return the location of b_i on imager j in
  // millimeters. !!Warning!! This gives data back in millimeters
  // which is different from most implementations.
  Vector2 operator() ( unsigned i, unsigned j, camera_vector_t const& a_j, point_vector_t const& b_i ) const {
    // Warning! This operation can not be allowed to change the camera properties.

    // Loading equations
    boost::shared_ptr<asp::BaseEquation> posF = m_cameras[j]->position_func();
    boost::shared_ptr<asp::BaseEquation> poseF = m_cameras[j]->pose_func();

    // Applying new equation constants
    for (unsigned n = 0; n < posF->size(); ++n)
      (*posF)[n] = a_j[n];
    for (unsigned n = 0; n < poseF->size(); ++n)
      (*poseF)[n] = a_j[n + posF->size()];

    // Determine what time to use for the camera forward
    // projection. Sorry that this is a search, I don't have a better
    // idea. :/
    int m = 0;
    while ( (*m_network)[i][m].image_id() != int(j) )
      m++;
    if ( int(j) != (*m_network)[i][m].image_id() )
      vw_throw( LogicErr() << "ISIS Adjust: Failed to find measure matching camera id." );

    // Performing the forward projection. This is specific to the
    // IsisAdjustCameraModel. The first argument is really just
    // passing the time instance to load up a pinhole model for.
    //std::cout << "DBG: ephemeris time " << m_network[i][m].ephemeris_time() << std::endl;
    Vector2 forward_projection = m_cameras[j]->point_to_pixel( b_i );

    // Giving back the pixel measurement.
    return forward_projection;
  }

  void parse_camera_parameters(camera_vector_t a_j,
                               Vector3 &position_correction,
                               Vector3 &pose_correction) const {
    position_correction = subvector(a_j, 0, 3);
    pose_correction = subvector(a_j, 3, 3);
  }

  // Errors on the image plane
  std::string image_unit( void ) { return "px"; }
  void image_errors( std::vector<double>& px_errors ) {
    px_errors.clear();
    for (unsigned i = 0; i < m_network->size(); ++i )
      for (unsigned m = 0; m < (*m_network)[i].size(); ++m ) {
        int camera_idx = (*m_network)[i][m].image_id();
        Vector2 px_error = (*m_network)[i][m].position() - (*this)(i, camera_idx, a[camera_idx],b[i]);
        px_errors.push_back(norm_2(px_error));
      }
  }

  // Errors for camera position
  void camera_position_errors( std::vector<double>& camera_position_errors ) {
    camera_position_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j ) {
      // TODO: This needs to be compliant if the BA is using a
      // non-zero order equation
      Vector3 position_initial = subvector(a_initial[j],0,3);
      Vector3 position_now = subvector(a[j],0,3);
      camera_position_errors.push_back(norm_2(position_initial-position_now));
    }
  }

  // Errors for camera pose
  void camera_pose_errors( std::vector<double>& camera_pose_errors ) {
    camera_pose_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j ) {
      // TODO: This needs to be compliant if the BA is using a
      // non-zero order equation
      Vector3 pose_initial = subvector(a_initial[j],3,3);
      Vector3 pose_now = subvector(a[j],3,3);
      camera_pose_errors.push_back(norm_2(pose_initial-pose_now));
    }
  }

  // Errors for gcp errors
  void gcp_errors( std::vector<double>& gcp_errors ) {
    gcp_errors.clear();
    for (unsigned i=0; i < this->num_points(); ++i )
      if ((*m_network)[i].type() == ControlPoint::GroundControlPoint) {
        point_vector_t p1 = b_initial[i];
        point_vector_t p2 = b[i];
        gcp_errors.push_back(norm_2(subvector(p1,0,3) - subvector(p2,0,3)));
      }
  }

  // Give access to the control network
  boost::shared_ptr<ControlNetwork> control_network(void) {
    return m_network;
  }
};

}}
