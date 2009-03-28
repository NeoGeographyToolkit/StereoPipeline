// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file isis_adjust.cc
///    

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"
#include <boost/algorithm/string.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera/BundleAdjustReport.h>
#include <vw/Camera/ControlNetwork.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Core/Debugging.h>
using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::ip;

#include <stdlib.h>
#include <iostream>
#include <iomanip>

#include "Isis/DiskImageResourceIsis.h"
#include "Isis/IsisCameraModel.h"
#include "Isis/Equations.h"
#include "Isis/IsisAdjustCameraModel.h"
#include "BundleAdjustUtils.h"

// Global variables
float g_spacecraft_position_sigma;
float g_spacecraft_pose_sigma;
float g_gcp_scalar;
boost::shared_ptr<ControlNetwork> g_cnet( new ControlNetwork("IsisAdjust Control Network (in mm)"));
po::variables_map g_vm;
std::vector< boost::shared_ptr< IsisAdjustCameraModel > > g_camera_adjust_models;
std::vector<std::string> g_input_files;
double g_lambda;
int g_max_iterations;
int g_report_level;
bool g_kml_all;

// A useful snippet for working with files
static std::string prefix_from_filename( std::string const& filename ){
  std::string result = filename;
  int index = result.find(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

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

public:

  IsisBundleAdjustmentModel( std::vector< boost::shared_ptr< vw::camera::IsisAdjustCameraModel> > const& camera_models,
			     boost::shared_ptr<ControlNetwork> network,
			     std::vector< std::string > input_names ) :
    m_cameras( camera_models ), m_network(network), a( camera_models.size() ), 
    b( network->size()), a_initial( camera_models.size() ), 
    b_initial( network->size() ), m_files( input_names ){

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the A and B vectors, storing the initial values.
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      // I'm using what is already in the IsisAdjust camera file as
      // the orginal starting point for the problem. This way I can
      // nudge it with error and see what it is doing for debuging
      boost::shared_ptr<PositionZeroOrder> posF = boost::dynamic_pointer_cast<PositionZeroOrder>(m_cameras[j]->getPositionFuncPoint());
      boost::shared_ptr<PoseZeroOrder> poseF = boost::dynamic_pointer_cast<PoseZeroOrder>(m_cameras[j]->getPoseFuncPoint());

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
    if ( (*m_network)[0][0].description() != "millimeters" )
      std::cout << "WARNING: Control Network doesn't seem to be in the correct units for the Control Measure for this problem (millimeters)" << std::endl;

  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters( int j ) const { return a[j]; }
  point_vector_t B_parameters( int i ) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) { a[j] = a_j; }
  void set_B_parameters(int i, point_vector_t const& b_i) { b[i] = b_i; }

  // Approximate the jacobian for small variations in the a_j
  // parameters ( camera parameters ).
  virtual Matrix<double, 2, positionParam+poseParam> A_jacobian( unsigned i, unsigned j,
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
  virtual Matrix<double, 2, 3> B_jacobian ( unsigned i, unsigned j,
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
  inline Matrix<double, (positionParam+poseParam), (positionParam+poseParam)> A_inverse_covariance ( unsigned j ) {
    Matrix< double, (positionParam+poseParam), (positionParam+poseParam) > result;
    for ( unsigned i = 0; i <positionParam; ++i )
      result(i,i) = 1/pow(g_spacecraft_position_sigma,2);
    for ( unsigned i = positionParam; i < (positionParam+poseParam); ++i )
      result(i,i) = 1/pow(g_spacecraft_pose_sigma,2);
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double, 3, 3> B_inverse_covariance ( unsigned i ) {
    Matrix< double, 3, 3> result;
    Vector3 sigmas = (*m_network)[i].sigma();
    sigmas = g_gcp_scalar*sigmas;
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

    // The whole A parameter is just written in a line
    for ( unsigned i = 0; i < a[j].size(); ++i)
      ostr << std::setprecision(18) << a[j][i] << "\t";

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
    boost::shared_ptr<PositionZeroOrder> posF = boost::dynamic_pointer_cast<PositionZeroOrder>(m_cameras[j]->getPositionFuncPoint());
    boost::shared_ptr<PoseZeroOrder> poseF = boost::dynamic_pointer_cast<PoseZeroOrder>(m_cameras[j]->getPoseFuncPoint());

    // Setting new equations defined by a_j
    for (unsigned n = 0; n < posF->size(); ++n)
      posF->set_constant(n,a[j][n]);
    for (unsigned n = 0; n < poseF->size(); ++n)
      poseF->set_constant(n,a[j][n + posF->size()]);

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
    boost::shared_ptr<PositionZeroOrder> posF = boost::dynamic_pointer_cast<PositionZeroOrder>(m_cameras[j]->getPositionFuncPoint());
    boost::shared_ptr<PoseZeroOrder> poseF = boost::dynamic_pointer_cast<PoseZeroOrder>(m_cameras[j]->getPoseFuncPoint());

    // Applying new equation constants
    for (unsigned n = 0; n < posF->size(); ++n)
      posF->set_constant(n,a_j[n]);
    for (unsigned n = 0; n < poseF->size(); ++n)
      poseF->set_constant(n,a_j[n + posF->size()]);

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
    Vector3 forward_projection = m_cameras[j]->point_to_mm_time( Vector3( 0, 0, (*m_network)[i][m].ephemeris_time() ), b_i );

    // Giving back the millimeter measurement.
    return Vector2( forward_projection[0], forward_projection[1] );
  }

  void parse_camera_parameters(camera_vector_t a_j, 
                               Vector3 &position_correction,
                               Vector3 &pose_correction) const {
    position_correction = subvector(a_j, 0, 3);
    pose_correction = subvector(a_j, 3, 3);
  }

  // Errors on the image plane
  std::string image_unit( void ) { return "millimeters"; }
  void image_errors( std::vector<double>& mm_errors ) {
    mm_errors.clear();
    for (unsigned i = 0; i < m_network->size(); ++i )
      for (unsigned m = 0; m < (*m_network)[i].size(); ++m ) {
	int camera_idx = (*m_network)[i][m].image_id();
	Vector2 mm_error = (*m_network)[i][m].focalplane() - (*this)(i, camera_idx, a[camera_idx],b[i]);
	mm_errors.push_back(norm_2(mm_error));
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

// Perform bundle adjustment
template <class CostT>
void perform_bundleadjustment( CostT const& cost_function ) {
  // Building the Bundle Adjustment Model and applying the Bundle
  // Adjuster.
  IsisBundleAdjustmentModel< 3, 3> 
    ba_model( g_camera_adjust_models , g_cnet, g_input_files );
  BundleAdjustment< IsisBundleAdjustmentModel< 3, 3>, CostT > 
    bundle_adjuster( ba_model, cost_function );

  // Handling options to modify Bundle Adjuster
  if ( g_vm.count( "lambda" ) )
    bundle_adjuster.set_lambda( g_lambda );
  if ( cost_function.name_tag() != "L2Error" )
    bundle_adjuster.set_control( 1 ); // Shutting off fast Fletcher-style control
  if ( g_vm.count( "seed-with-previous" ) ) {
    std::cout << "Seeding with previous ISIS adjustment files.\n";
    std::cout << "\tLoading up previous ISIS adjustments\n";
    for (int j = 0; j < g_input_files.size(); ++j ) {
      std::string adjust_file = prefix_from_filename( g_input_files[j] ) +
	".isis_adjust";

      // Loading and forcing in the adjustment
      if ( fs::exists( adjust_file ) ) {
	std::cout << "\t\tFound: " << adjust_file << std::endl;
	std::ifstream input( adjust_file.c_str() );
	Vector<double> camera_vector = ba_model.A_parameters( j );
	for ( unsigned n = 0; n < camera_vector.size(); n++ )
	  input >> camera_vector[n];
	input.close();
	ba_model.set_A_parameters( j, camera_vector );

	// Store new A_vector into the ISIS Adjust Camera Models we
	// use.
	boost::shared_ptr<IsisAdjustCameraModel> temp = ba_model.adjusted_camera(j);
	// The above command indirectly stores the current A_vector
	// into the camera model

      }
    }
    
    // Retriangulating position of control points in control network
    {
      std::vector<boost::shared_ptr<CameraModel> > cameras = ba_model.adjusted_cameras();
      for ( ControlNetwork::iterator cp = g_cnet->begin();
	    cp != g_cnet->end(); ++cp ) {
	if ( cp->type() != ControlPoint::TiePoint )
	  continue; // We don't move GCPs, this seems correct.
	int count = 0;
	Vector3 estimate3d(0,0,0);
	// Running permutation of all measures
	for ( ControlPoint::const_iterator m1 = (*cp).begin()+1;
	      m1 != (*cp).end(); ++m1 ) {
	  for ( ControlPoint::const_iterator m2 = (*cp).begin();
		m2 != m1; ++m2 ) {
	    stereo::StereoModel sm( *cameras[m1->image_id()],
				    *cameras[m2->image_id()] );
	    double error;
	    Vector3 triangulation = sm( m1->position(),
					m2->position(),
					error );
	    if ( triangulation != Vector3() ) {
	      count++;
	      estimate3d += triangulation;
	      // Do I want to do anything with the error?
	    } else
	      vw_out(DebugMessage, "bundle_adjustment") << "Error: Failed a retriangulation.\n";
	  }
	}
	
	estimate3d /= count;
	cp->set_position( estimate3d );
      }
    }
      
    // Repushing the position in control network into BA model
    {
      std::cout << "\tPush new triangulation results back into BA model\n";
      
      for ( unsigned i = 0; i < g_cnet->size(); ++i )
	ba_model.set_B_parameters( i, (*g_cnet)[i].position() );
    }
  }

  // Clearing the monitoring text files
  if ( g_vm.count( "save-iteration-data" ) ) {
    std::ofstream ostr( "iterCameraParam.txt", std::ios::out );
    ostr << "";
    ostr.close();
    ostr.open( "iterPointsParam.txt", std::ios::out );
    ostr << "";
    ostr.close();

    // Now saving the initial starting data:
    // Recording Points Data
    std::ofstream ostr_points("iterPointsParam.txt", std::ios::app);
    for ( unsigned i = 0; i < ba_model.num_points(); ++i ) {
      Vector3 point = ba_model.B_parameters(i);
      ostr_points << std::setprecision(18) << i << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << std::endl;
    }

    // Recording Camera Data
    std::ofstream ostr_camera("iterCameraParam.txt", std::ios::app);
    for ( unsigned j = 0; j < ba_model.num_cameras(); ++j ) {
	  
      boost::shared_ptr<IsisAdjustCameraModel> camera = ba_model.adjusted_camera(j);

      // Saving points along the line of the camera
      for ( int i = 0; i < camera->getLines(); i+=(camera->getLines()/8) ) {
	Vector3 position = camera->camera_center( Vector2(0,i) ); // This calls legacy support
	ostr_camera << std::setprecision(18) << j << "\t" << position[0] << "\t" << position[1] << "\t" << position[2];
	Quaternion<double> pose = camera->camera_pose( Vector2(0,i) ); // Legacy as well
	pose = pose / norm_2(pose);
	Vector3 euler = rotation_matrix_to_euler_xyz( pose.rotation_matrix() );
	ostr_camera << std::setprecision(18) << "\t" << euler[0] << "\t" << euler[1] << "\t" << euler[2] << std::endl;
      }
    }
  }

  // Reporter
  BundleAdjustReport< IsisBundleAdjustmentModel<3,3>, BundleAdjustment< IsisBundleAdjustmentModel<3,3>, CostT > > reporter( "ISIS Adjust", ba_model, bundle_adjuster, g_report_level);

  // Option to write KML of control network
  if ( g_vm.count("write-kml") ) {
    std::cout << "Writing KML of Control Network.\n";
    reporter.write_control_network_kml( !g_kml_all );
  }

  // Performing the Bundle Adjustment
  double abs_tol = 1e10, rel_tol = 1e10;
  if (g_vm.count("nonsparse")) {    //What are you thinking? No!!
    // This is the non sparse implementation
    while ( bundle_adjuster.update_reference_impl( abs_tol, rel_tol ) ) {
      reporter.loop_tie_in();

      //Writing recording data for Bundlevis
      if ( g_vm.count("save-iteration-data") ) {
	
	// Recording Points Data
	std::ofstream ostr_points("iterPointsParam.txt", std::ios::app);
	for ( unsigned i = 0; i < ba_model.num_points(); ++i ) {
	  Vector3 point = ba_model.B_parameters(i);
	  ostr_points << std::setprecision(18) << i << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << std::endl;
	}

	// Recording Camera Data
	std::ofstream ostr_camera("iterCameraParam.txt", std::ios::app);
	for ( unsigned j = 0; j < ba_model.num_cameras(); ++j ) {
	  
	  boost::shared_ptr< IsisAdjustCameraModel > camera = ba_model.adjusted_camera(j);

	  // Saving points along the line of the camera
	  for ( int i = 0; i < camera->getLines(); i+=(camera->getLines()/8) ) {
	    Vector3 position = camera->camera_center( Vector2(0,i) ); // This calls legacy support
	    ostr_camera << std::setprecision(18) << j << "\t" << position[0] << "\t" << position[1] << "\t" << position[2];
	    Quaternion<double> pose = camera->camera_pose( Vector2(0,i) ); // Legacy as well
	    pose = pose / norm_2( pose );
	    Vector3 euler = rotation_matrix_to_euler_xyz( pose.rotation_matrix() );
	    ostr_camera << "\t" << euler[0] << "\t" << euler[1] << "\t" << euler[2] << std::endl;
	  }
	}
      } // end of saving data

      // Determing is it time to quit?
      if ( bundle_adjuster.iterations() > g_max_iterations || abs_tol < 0.01 || rel_tol < 1e-10)
	break;

    }
  } else {
    // This is the sparse implementation of the code
    while ( bundle_adjuster.update( abs_tol, rel_tol ) ) {
      reporter.loop_tie_in();

      //Writing recording data for Bundlevis
      if ( g_vm.count("save-iteration-data") ) {
	
	// Recording Points Data
	std::ofstream ostr_points("iterPointsParam.txt", std::ios::app);
	for ( unsigned i = 0; i < ba_model.num_points(); ++i ) {
	  Vector3 point = ba_model.B_parameters(i);
	  ostr_points << std::setprecision(18) << i << "\t" << point[0] << "\t" << point[1] << "\t" << point[2] << std::endl;
	}

	// Recording Camera Data
	std::ofstream ostr_camera("iterCameraParam.txt", std::ios::app);
	for ( unsigned j = 0; j < ba_model.num_cameras(); ++j ) {
	  
	  boost::shared_ptr< IsisAdjustCameraModel > camera = ba_model.adjusted_camera(j);

	  // Saving points along the line of the camera
	  for ( int i = 0; i < camera->getLines(); i+=(camera->getLines()/8) ) {
	    Vector3 position = camera->camera_center( Vector2(0,i) ); // This calls legacy support
	    ostr_camera << std::setprecision(18) << std::setprecision(18) << j << "\t" << position[0] << "\t" << position[1] << "\t" << position[2];
	    Quaternion<double> pose = camera->camera_pose( Vector2(0,i) ); // Legacy as well
	    pose = pose / norm_2(pose);
	    Vector3 euler = rotation_matrix_to_euler_xyz( pose.rotation_matrix() );
	    ostr_camera << std::setprecision(18) << "\t" << euler[0] << "\t" << euler[1] << "\t" << euler[2] << std::endl;
	  }
	}
      } // end of saving data

      // Determine if it is time to quit
      if ( bundle_adjuster.iterations() > g_max_iterations || abs_tol < 0.01 || rel_tol < 1e-10 )
	break;
    }
  }
  reporter.end_tie_in();

  for ( unsigned int i = 0; i < ba_model.num_cameras(); ++i ) {
    ba_model.write_adjustment( i, prefix_from_filename( g_input_files[i] ) + ".isis_adjust");
  }
}

// Main Executable
int main(int argc, char* argv[]) {

  std::string cnet_file;
  std::string robust_cost_function;
  int min_matches;
  double robust_outlier_threshold;

  // BOOST Program Options code
  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&g_lambda), "Set the intial value of the LM parameter g_lambda")
    ("position-sigma", po::value<float>(&g_spacecraft_position_sigma)->default_value(100.0), "Set the sigma (uncertainty) of the spacecraft position. (meters)")
    ("pose-sigma", po::value<float>(&g_spacecraft_pose_sigma)->default_value(1.0/10.0), "Set the sigma (uncertainty) of the spacecraft pose. (radians)")
    ("gcp-scalar", po::value<float>(&g_gcp_scalar)->default_value(1.0), "Sets a scalar to multiply to the sigmas (uncertainty) defined for the gcps. GCP sigmas are defined in the .gcp files.")
    ("cost-function", po::value<std::string>(&robust_cost_function)->default_value("L2"), "Choose a robust cost function from [PseudoHuber, Huber, L1, L2, Cauchy]")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("save-iteration-data,s", "Saves all camera/point/pixel information between iterations for later viewing in Bundlevis")
    ("min-matches", po::value<int>(&min_matches)->default_value(30), "Set the minimum number of matches between images that will be considered.")
    ("max-iterations", po::value<int>(&g_max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("report-level,r", po::value<int>(&g_report_level)->default_value(10), "Changes the detail of the Bundle Adjustment Report")
    ("nonsparse,n", "Run the non-sparse reference implementation of LM Bundle Adjustment. (For Debugging Purposes)")
    ("seed-with-previous", "Use previous isis_adjust files at starting point for this run.")
    ("write-isis-cnet-also", "Writes an ISIS style control network")
    ("write-kml", po::value<bool>(&g_kml_all), "Selecting this will cause a kml to be writting of the GCPs, send with a true and it will also write all the 3d estimates")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&g_input_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), g_vm );
  po::notify( g_vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <isis cube files> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if ( g_vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  }
  
  if ( g_vm.count("input-files") < 1 ) {
    std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }

  // Checking cost functio strings
  boost::to_lower( robust_cost_function );
  if ( !( robust_cost_function == "pseudohuber" ||
	  robust_cost_function == "huber" ||
	  robust_cost_function == "l1" ||
	  robust_cost_function == "l2" ||
	  robust_cost_function == "cauchy" ) ) {
    vw_out(0) << "Unkown robust cost function: " << robust_cost_function
	      << ". Options are : [ PseudoHuber, Huber, L1, L2, Cauchy]\n";
    exit(0);
  }

  // Loading the image data into the camera models. Also applying
  // blank equations to define the cameras
  std::vector< boost::shared_ptr<CameraModel> > camera_models( g_input_files.size() );
  for ( unsigned i = 0; i < g_input_files.size(); ++i ) {
    std::cout << "Loading: " << g_input_files[i] << std::endl;

    // Equations defining the delta
    boost::shared_ptr<PositionZeroOrder> posF( new PositionZeroOrder() );
    boost::shared_ptr<PoseZeroOrder> poseF( new PoseZeroOrder() );
    boost::shared_ptr<CameraModel> p ( new IsisAdjustCameraModel( g_input_files[i], posF, poseF ) );
    camera_models[i] = p;
  }

  // Checking to see if there is a cnet file to load up
  if ( g_vm.count("cnet") ){
    std::cout << "Loading control network from file: " << cnet_file << "\n";

    std::vector<std::string> tokens;
    boost::split( tokens, cnet_file, boost::is_any_of(".") );
    if ( tokens[tokens.size()-1] == "net" ) {
      // An ISIS style control network
      g_cnet->read_isis_pvl_control_network( cnet_file );
    } else if ( tokens[tokens.size()-1] == "cnet" ) {
      // A VW binary style
      g_cnet->read_binary_control_network( cnet_file );
    } else {
      vw_throw( IOErr() << "Unknown Control Network file extension, \""
		<< tokens[tokens.size()-1] << "\"." );
    }

    // Assigning camera id number for Control Measures
    std::vector<std::string> camera_serials;
    for ( unsigned i = 0; i < camera_models.size(); ++i ) {
      boost::shared_ptr< IsisAdjustCameraModel > cam =
	boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[i] );
      camera_serials.push_back( cam->serial_number() );
    }
    for ( unsigned i = 0; i < g_cnet->size(); ++i ) {
      for ( unsigned m = 0; m < (*g_cnet)[i].size(); ++m ) {
	bool found = false;
	// Determining which camera matches the serial string
	for ( unsigned s = 0; s < camera_serials.size(); ++s ) {
	  if ( (*g_cnet)[i][m].serial() == camera_serials[s] ) {
	    (*g_cnet)[i][m].set_image_id( s );
	    found = true;
	    break;
	  }
	}
	if (!found)
	  vw_throw( InputErr() << "ISIS Adjust doesn't seem to have a camera for serial, \"" << (*g_cnet)[i][m].serial() << "\", found in loaded Control Network" );
      }
    }

  } else {
    // Decided to build new Control Network. Now loading up the matches
    
    std::cout << "\nLoading Matches:\n";
    for (unsigned i = 0; i < g_input_files.size(); ++i) {
      for (unsigned j = i+1; j < g_input_files.size(); ++j){
	std::string match_filename = 
	  prefix_from_filename( g_input_files[i] ) + "__" +
	  prefix_from_filename( g_input_files[j] ) + ".match";
	
	if ( fs::exists(match_filename) ) {
	  // Locate all of the interest points between images that may
	  // overlap based on a rough approximation of their bounding box.

	  std::vector<InterestPoint> ip1, ip2;
	  read_binary_match_file(match_filename, ip1, ip2);
	  if ( int(ip1.size()) < min_matches ) {
	    std::cout << "\t" << match_filename << "    " << i << " <-> " 
		      << j << " : " << ip1.size() << " matches. [rejected]\n";
	  } else {
	    std::cout << "\t" << match_filename << "    " << i << " <-> " 
		      << j << " : " << ip1.size() << " matches.\n";
	    add_matched_points(*g_cnet, ip1, ip2, i, j, camera_models);
	  }
	}
      }
    }

    VW_ASSERT( g_cnet->size() != 0, vw::Aborted() << "Failed to load any points, Control Network Empty\n" );

    std::cout << "\nLoading Ground Control Points:\n";
    for (unsigned i = 0; i < g_input_files.size(); ++i){
      std::string gcp_filename = prefix_from_filename( g_input_files[i] ) + ".gcp";
      if ( fs::exists( gcp_filename ) ) {
	int numpoints = add_ground_control_points( *g_cnet, gcp_filename, i );
	std::cout << "\t" << gcp_filename << "    " << " : " << numpoints << " GCPs.\n";
      }
    }
  }

  // Double checking to make sure that the Control Network is set up
  // for ephemeris time. We continue to do this with loaded control
  // networks as a strict ISIS style control network will not record
  // Ephemeris Time.
  {
    std::cout << "\nCalculating focal plane measurements for Control Network:\n";
    for (unsigned i = 0; i < g_cnet->size(); ++i ) {
      for ( unsigned m = 0; m < (*g_cnet)[i].size(); ++m ) {
	if ( (*g_cnet)[i][m].ephemeris_time() == 0 ) {
	  // Loading camera used by measure
	  boost::shared_ptr< IsisAdjustCameraModel > cam =
	    boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[ (*g_cnet)[i][m].image_id() ] );

	  Vector3 mm_time = cam->pixel_to_mm_time( (*g_cnet)[i][m].position() );
	  (*g_cnet)[i][m].set_focalplane( mm_time[0], mm_time[1] );
	  (*g_cnet)[i][m].set_ephemeris_time( mm_time[2] );
	  (*g_cnet)[i][m].set_description( "millimeters" );
	  (*g_cnet)[i][m].set_serial( cam->serial_number() );
	  (*g_cnet)[i][m].set_pixels_dominant( false );
	}
      }
    }

    // Writing ISIS Control Network
    g_cnet->write_binary_control_network("isis_adjust");
  }

  VW_DEBUG_ASSERT( g_cnet->size() != 0, vw::MathErr() << "Control network conversion error to millimeter time" );

  // Option to write ISIS-style control network
  if ( g_vm.count("write-isis-cnet-also") ) {
    std::cout << "Writing ISIS-style Control Network.\n";
    g_cnet->write_isis_pvl_control_network("isis_adjust");
  }

  // Need to typecast all the models to feed to the Bundle Adjustment
  // model, kinda ugly.
  g_camera_adjust_models.resize( camera_models.size() );
  for ( unsigned j = 0; j < camera_models.size(); ++j )
    g_camera_adjust_models[j] = boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[j]);

  // Switching based on cost function
  if ( robust_cost_function == "pseudohuber" ) {
    perform_bundleadjustment<PseudoHuberError>( PseudoHuberError(robust_outlier_threshold) );
  } else if ( robust_cost_function == "huber" ) {
    perform_bundleadjustment<HuberError>( HuberError(robust_outlier_threshold) );
  } else if ( robust_cost_function == "l1" ) {
    perform_bundleadjustment<L1Error>( L1Error() );
  } else if ( robust_cost_function == "l2" ) {
    perform_bundleadjustment<L2Error>( L2Error() );
  } else if ( robust_cost_function == "cauchy" ) {
    perform_bundleadjustment<CauchyError>( CauchyError(robust_outlier_threshold) );
  }

  return 0;
}
