  
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
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera/ControlNetwork.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Math/LevenbergMarquardt.h>
using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::ip;

#include <stdlib.h>
#include <iostream>

#include "Isis/DiskImageResourceIsis.h"
#include "Isis/IsisCameraModel.h"
#include "Isis/Equations.h"
#include "Isis/IsisAdjustCameraModel.h"
#include "BundleAdjustUtils.h"

// Position and Pose Equations
//    For now the adjust equations are just constant offsets. This is
//    just to test to see if the code even works.
class PositionFunc : public VectorEquation{
private:
  std::vector<double> _constant;
public:
  PositionFunc ( double x, double y, double z ){
    _constant.push_back( x );
    _constant.push_back( y );
    _constant.push_back( z );
  }
  virtual Vector3 evaluate( double t ) const {
    // This evaluates the function at time t
    return Vector3( _constant[0], _constant[1], _constant[2] );
  }
  virtual unsigned size( void ) const {
    // This produces the number of variables allowed for changing
    return _constant.size();
  }
  virtual double& operator[]( unsigned n ) {
    // This produces the variable n's value;
    if ( n >= _constant.size() ){
      std::cout << "ERROR" << std::endl;
      return _constant[0];
    } else {
      return _constant[n];
    }
  }
};

class PoseFunc : public QuaternionEquation{
private:
  std::vector<double> _constant;
public:
  PoseFunc ( double x, double y, double z ) {
    _constant.push_back( x );
    _constant.push_back( y );
    _constant.push_back( z );
  }
  virtual Quaternion<double> evaluate( double t ) const {
    return euler_to_quaternion( _constant[0], _constant[1], _constant[2], "xyz" );
  }
  virtual unsigned size( void ) const {
    return _constant.size();
  }
  virtual double& operator[]( unsigned n ) {
    if ( n >= _constant.size() ){
      std::cout << "ERROR" << std::endl;
      return _constant[0];
    } else {
      return _constant[n];
    }
  }
};

// A useful snippet for working with files
static std::string prefix_from_filename( std::string const& filename ){
  std::string result = filename;
  int index = result.find(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

// This is the Bundle Adjustment model
template <unsigned positionParam, unsigned poseParam, class PositionFuncT, class PoseFuncT>
class IsisBundleAdjustmentModel : public camera::BundleAdjustmentModelBase< IsisBundleAdjustmentModel < positionParam,
													poseParam,
													PositionFuncT,
													PoseFuncT >,
									    (positionParam+poseParam), 3>{
  typedef Vector<double, positionParam+poseParam> camera_vector_t;
  typedef Vector<double, 3> point_vector_t;

  ControlNetwork m_network;

  std::vector< boost::shared_ptr<IsisAdjustCameraModel< PositionFuncT, PoseFuncT > > > m_cameras;

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_initial;
  std::vector<point_vector_t> b_initial;
  int m_num_pixel_observations;

public:

  IsisBundleAdjustmentModel( std::vector< boost::shared_ptr< vw::camera::IsisAdjustCameraModel< PositionFuncT,
			     PoseFuncT > > > const& camera_models,
			     ControlNetwork const& network ) :
    m_cameras( camera_models ), m_network( network ),
    a( camera_models.size() ), b( network.size() ),
    a_initial( camera_models.size() ), b_initial( network.size() ) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network.size(); ++i)
      m_num_pixel_observations += network[i].size();

    // Set up the A and B vectors, storing the initial values.
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      a[j] = camera_vector_t();
      a_initial[j] = a[j];
    }

    for (unsigned i = 0; i < network.size(); ++i) {
      b[i] = m_network[i].position();
      b_initial[i] = b[i];
    }

  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters( int j ) const { return a[j]; }
  point_vector_t B_parameters( int i ) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) {
    a[j] = a_j;
  }
  void set_B_parameters(int i, point_vector_t const& b_i) {
    b[i] = b_i;
  }

  // Approximate the jacobian for small variations in the a_j
  // parameters ( camera parameters ).
  virtual Matrix<double, 2, positionParam+poseParam> A_jacobian( unsigned i, unsigned j,
								 camera_vector_t const& a_j,
								 point_vector_t const& b_i ) {
    Matrix<double> partial_derivatives = camera::BundleAdjustmentModelBase< IsisBundleAdjustmentModel,
      positionParam+poseParam, 3>::A_jacobian(i, j, a_j, b_i);
    return partial_derivatives;
  }

  // Return Initial parameters. (Used by the bundle adjuster )
  camera_vector_t A_initial( int j ) const { return a_initial[j]; }
  point_vector_t B_initial( int i ) const { return b_intitial[i]; }

  // Return general sizes
  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return pixel observations -> supposedly used by Bundlevis eventually i think
  unsigned num_observations_of_point ( const int& i ) const { return m_network[i].size(); }
  unsigned corresponding_camera_for_measure( const int& i, const int& m ) {
    return m_network[i][m].image_id();
  }

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double, (positionParam+poseParam), (positionParam+poseParam)> A_inverse_covariance ( unsigned j ) {
    Matrix< double, (positionParam+poseParam), (positionParam+poseParam) > result;
    for ( unsigned i = 0; i < (positionParam+poseParam); ++i )
      result(i,i) = 1;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double, 3, 3> B_inverse_covariance ( unsigne i ) {
    Matrix< double, 3, 3> result;
    for ( unsigned i = 0; i < 3; ++i)
      result(i,i) = 1/1000.0;
    return result;
  }

  // This is for writing isis_adjust file for later
  void write_adjustment( int j, std::string const& filename ) {
    std::ofstream oster( filename.c_str() );

    for ( unsigned i = 0; i < positionParam; ++i )
      ostr << a[j][i] << "\t";
    ostr << std::endl;

    for ( unsigned i = 0; i < poseParam; ++i )
      ostr << a[j][positionParam + i] << "\t";
    ostr << std::endl;
  }

  std::vector< boost::shared_ptr< camera::CameraModel > > adjusted_cameras() {
    return m_cameras;
  }

  boost::shared_ptr< IsisAdjustCameraModel< PositionFuncT, PoseFuncT > > adjusted_camera( int j ) {
    return m_cameras[j];
  }
};


// Main Executable
int main(int argc, char* argv[]) {

  std::cout << "Hi, is this working?" << std::endl;

  std::vector<std::string> input_files;
  std::string cnet_file;
  ControlNetwork cnet("ISIS Control Network");
  double lambda;
  double robust_outlier_threshold;
  int min_matches;
  int max_iterations;

  // BOOST Program Options code
  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the intial value of the LM parameter lambda")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("help,h", "Display this help message")
    ("save-iteration-data,s", "Saves all camera/point/pixel information between iterations for later viewing in Bundlevis")
    ("run-match,m", "Run ipmatch to create .match files from overlapping images.")
    ("match-debug-images,d", "Create debug images when you run ipmatch.")
    ("min-matches", po::value<int>(&min_matches)->default_value(30), "Set the minimum number of matches between images that will be considered.")
    ("max-iterations", po::value<int>(&max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("nonsparse,n", "Run the non-sparse reference implementation of LM Bundle Adjustment.")
    ("verbose", "Verbose output");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <isis cube files> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if ( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  };
  
  if ( vm.count("input-files") < 1 ) {
    if ( vm.count("cnet") ) {
      std::cout << "Loading control network from file: " << cnet_file << "\n";
      cnet.read_control_network(cnet_file);
    } else {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }

  // Loading the image data
  std::vector< boost::shared_ptr<CameraModel> > camera_models( input_files.size() );
  // An ugly hack
  //std::vector< boost::shared_ptr<CameraModel> > camera_traditional_models( input_files.size() );
  for ( unsigned i = 0; i < input_files.size(); ++i ) {
    std::cout << "Loading: " << input_files[i] << std::endl;

    // Equations defining the delta
    PositionFunc posF( 0, 0, 0);
    std::cout << posF.evaluate(3) << std::endl;
    PoseFunc poseF( 0, 0, 0);
    boost::shared_ptr<CameraModel> p ( new IsisAdjustCameraModel<PositionFunc,PoseFunc>( input_files[i], posF, poseF ) );
    camera_models[i] = p;
    //camera_traditional_models[i] = p;
    //camera_models[i] = boost::shared_ptr< IsisAdjustCameraModel< PositionFuncT, PoseFuncT > >( new IsisAdjustCameraModel< PositionFuncT, PoseFuncT >( input_files[i], posF, poseF ) );
  }

  // Now I am loading up the matches between the two images
  if ( !vm.count("cnet") ) {
    
    std::cout << "\nLoading Matches:\n";
    for (unsigned i = 0; i < input_files.size(); ++i) {
      for (unsigned j = i+1; j < input_files.size(); ++j){
	std::string match_filename = 
	  prefix_from_filename( input_files[i] ) + "__" +
	  prefix_from_filename( input_files[j] ) + ".match";
	
	std::cout << "\tWhat I'm looking for: " << match_filename << std::endl;

	// I don't think this code works at all
	if ( vm.count("run-match")) {
	  std::ostringstream cmd;
	  cmd << "ipmatch "
	      << input_files[i] << " "
	      << input_files[j] << " ";
	  if (vm.count("match-debug-images"))
	    cmd << "-d";
	  system(cmd.str().c_str());
	}

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
	    add_matched_points(cnet, ip1, ip2, i, j, camera_models);
	  }
	}
      }
    }

    std::cout << "\nLoading Ground Control Points:\n";
    for (unsigned i = 0; i < input_files.size(); ++i){
      std::string gcp_filename = prefix_from_filename( input_files[i] ) + ".gcp";
      if ( fs::exists( gcp_filename ) ) {
	int numpoints = add_ground_control_points( cnet, gcp_filename, i );
	std::cout << "\t" << gcp_filename << "    " << " : " << numpoints << " GCPs.\n";
      }
    }
    
    cnet.write_control_network("control.cnet");
  }

  // This is for testing the camera models
  {
    std::cout << "\nPerforming Linescan Adjust Camera Model testing" << std::endl;
    for ( unsigned i = 0; i < cnet.size(); i+=100 ) {
      for ( unsigned j = 0; j < input_files.size(); ++j ) {
	// Converting the data to mm_time and testing..
	boost::shared_ptr< IsisAdjustCameraModel<PositionFunc,PoseFunc> > cam = boost::shared_dynamic_cast< IsisAdjustCameraModel<PositionFunc,PoseFunc> >( camera_models[j] );
	Vector3 mm_time = cam->pixel_to_mm_time( cnet[i][j].position() );

	std::cout << "Pixel Measure: " << cnet[i][j].position()  << std::endl;
	std::cout << "\t" << mm_time[0] << "\t" << mm_time[1] << "\t" << mm_time[2] << std::endl;

	Vector3 forward_projection = cam->point_to_mm_time( mm_time, cnet[i].position() );
	double distance = norm_2( cam->camera_center( forward_projection ) - cnet[i].position() );
	Vector3 orientation = cam->mm_time_to_vector( forward_projection );
	Vector3 reproject = cam->camera_center( forward_projection ) + orientation* distance;
	std::cout << "Feed Camera " << j << " point " << cnet[i].position() << "\n\treturned " << reproject << std::endl;
	std::cout << "\tdifference " << cnet[i].position() - reproject << std::endl;

	//Adding some change now
	PoseFunc* pose = cam->getPoseFuncPoint();
	(*pose)[0] = (*pose)[0] + 3.1415/180;
	Vector3 reproject2 = cam->camera_center( forward_projection ) + cam->mm_time_to_vector( forward_projection )*distance;
	std::cout << "Change in pose applied" << std::endl << "\tdiffernce " << reproject - reproject2 << std::endl;
	std::cout << "\t> Camera center: " << cam->camera_center( forward_projection ) << std::endl;
	std::cout << "\t\t> Point Before: " << reproject << std::endl;
	std::cout << "\t\t> Point After: " << reproject2 << std::endl;
	std::cout << "\t\t> Change in angle: " << acos( dot_prod( reproject - cam->camera_center( forward_projection ), reproject2 - cam->camera_center( forward_projection) )/( norm_2(reproject - cam->camera_center( forward_projection ))*norm_2(reproject2 - cam->camera_center( forward_projection)) ) ) * 180/3.1415 << std::endl;
	
	std::cout << std::endl;
      }
    }
  }
 
  return 0;
}
