// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file rmax_adjust.cc
///

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Math.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/BundleAdjustment.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

#include <stdlib.h>
#include <iostream>

#include <asp/Sessions/RMAX/RMAX.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

// This sifts out from a vector of strings, a listing of GCPs.  This
// should be useful for those programs who accept their data in a mass
// input vector.
std::vector<std::string>
sort_out_gcps( std::vector<std::string>& image_files ) {
  std::vector<std::string> gcp_files;
  std::vector<std::string>::iterator it = image_files.begin();
  while ( it != image_files.end() ) {
    if ( boost::iends_with(*it, ".gcp") ){
      gcp_files.push_back( *it );
      it = image_files.erase( it );
    } else
      it++;
  }

  return gcp_files;
}

class HelicopterBundleAdjustmentModel : public ba::ModelBase<HelicopterBundleAdjustmentModel, 6, 3> {

  typedef Vector<double,6> camera_vector_t;
  typedef Vector<double,3> point_vector_t;

  std::vector<ImageInfo> m_image_infos;

  boost::shared_ptr<ControlNetwork> m_network;

  std::vector<camera_vector_t> a, a_target;
  std::vector<point_vector_t> b, b_target;
  int m_num_pixel_observations;

public:

  HelicopterBundleAdjustmentModel(std::vector<ImageInfo> const& image_infos,
                                  boost::shared_ptr<ControlNetwork> network) :
    m_image_infos(image_infos), m_network(network),
    a(image_infos.size()), a_target(image_infos.size()),
    b(network->size()), b_target(network->size()),
    m_num_pixel_observations(0) {

    // Compute the number of observations from the bundle.
    std::vector<point_vector_t>::iterator bit = b.begin(),
      btit = b_target.begin();
    BOOST_FOREACH( ba::ControlPoint const& cp, *m_network ) {
      m_num_pixel_observations += cp.size();
      *bit++ = *btit++ = cp.position();
    }
  }

  // Return a reference to the camera and point parameters.
  camera_vector_t const& A_parameters(int j) const { return a[j]; }
  point_vector_t const& B_parameters(int i) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) {
    a[j] = a_j;
  }
  void set_B_parameters(int i, point_vector_t const& b_i) {
    b[i] = b_i;
  }

  // Return the initial parameters
  camera_vector_t const& A_target(int j) const { return a_target[j]; }
  point_vector_t const& B_target(int i) const { return b_target[i]; }

  // Return general sizes
  size_t num_cameras() const { return a.size(); }
  size_t num_points() const { return b.size(); }
  size_t num_pixel_observations() const { return m_num_pixel_observations; }

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n>
  A_inverse_covariance ( size_t /*j*/ ) {
    return math::identity_matrix<camera_params_n>();
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n>
  B_inverse_covariance ( size_t /*i*/ ) {
    return math::identity_matrix<point_params_n>();
  }

  void write_adjustment(int j, std::string const& filename) {
    std::ofstream ostr(filename.c_str());
    ostr << a[j][0] << " " << a[j][1] << " " << a[j][2] << "\n";
    ostr << a[j][3] << " " << a[j][4] << " " << a[j][5] << "\n";
  }

  void write_adjusted_camera(int j, std::string const& filename) {
    camera_vector_t a_j = a[j];
    camera::CAHVORModel cam =
      rmax_image_camera_model(m_image_infos[j],
                              subvector(a_j,0,3),
                              subvector(a_j,3,3) );
    cam.write(filename);
  }

  void write_adjusted_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    ostr.precision(15);
    for (size_t j=0; j < a.size();++j){
      camera_vector_t a_j = a[j];
      camera::CAHVORModel cam =
        rmax_image_camera_model( m_image_infos[j],
                                 subvector(a_j,0,3),
                                 subvector(a_j,3,3) );
      ostr << j << "\t" << cam.C(0) << "\t" << cam.C(1)
           << "\t" << cam.C(2) << " 1 0 0 0\n";
    }
  }

  std::vector< boost::shared_ptr< CameraModel > >
  adjusted_cameras() const {
    std::vector< boost::shared_ptr<CameraModel> > cameras( a.size() );
    for ( size_t j = 0; j < a.size(); j++ ) {
      CAHVORModel cam =
        rmax_image_camera_model( m_image_infos[j],
                                 subvector(a[j],0,3),
                                 subvector(a[j],3,3) );
      cameras[j].reset( new CAHVORModel( cam ) );
    }
    return cameras;
  }

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( size_t /*i*/, size_t j,
                       camera_vector_t const& a_j,
                       point_vector_t const& b_i ) const {
    return
      rmax_image_camera_model(m_image_infos[j],
                              subvector(a_j, 0, 3),
                              subvector(a_j, 3, 3) ).point_to_pixel( b_i );
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
    return 0;
  }
  inline double gcp_compare( point_vector_t const& meas,
                             point_vector_t const& obj ) {
    return norm_2(meas-obj);
  }

  // Give access to the control network
  boost::shared_ptr<ControlNetwork> control_network() {
    return m_network;
  }
};

struct Options : public asp::BaseOptions {
  Options() : lambda(-1), cnet( new ControlNetwork("") ) {}
  std::vector<std::string> image_files, gcp_files;
  std::string cnet_file;
  double lambda;
  boost::shared_ptr<ControlNetwork> cnet;
  int min_matches, max_iterations, report_level;

  bool save_bundlevis;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value(&opt.cnet_file), "Load a control network from a file")
    ("lambda,l", po::value(&opt.lambda), "Set the initial value of the LM parameter lambda")
    ("min-matches", po::value(&opt.min_matches)->default_value(5), "Set the mininmum number of matches between images that will be considered.")
    ("max-iterations", po::value(&opt.max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("save-iteration-data,s", po::bool_switch(&opt.save_bundlevis)->default_value(false), "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt. Warning: This is slow as pixel observations need to be calculated on each step.")
    ("report-level,r",po::value(&opt.report_level)->default_value(10),"Changes the detail of the Bundle Adjustment Report");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value(&opt.image_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  std::string usage("[options] <rmax image filenames> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             hidden_options, p, usage );

  if ( opt.image_files.size() < 2)
    vw_throw( ArgumentErr() << "Must specify at least two input files!\n\n" );
  opt.gcp_files = sort_out_gcps( opt.image_files );
}

void do_ba( Options& opt ) {
  // Read in the camera model and RMAX image info for the input
  // images.
  std::vector<std::string> camera_files(opt.image_files.size());
  std::vector<ImageInfo> image_infos(opt.image_files.size());
  std::vector<boost::shared_ptr<CameraModel> > camera_models(opt.image_files.size());
  for (size_t i = 0; i < opt.image_files.size(); ++i) {
    read_image_info( opt.image_files[i], image_infos[i] );
    CAHVORModel *cahvor = new CAHVORModel;
    *cahvor = rmax_image_camera_model(image_infos[i]);
    camera_models[i] = boost::shared_ptr<CameraModel>(cahvor);
  }

  { // Subtract off the first camera's position
    Vector3 origin = camera_models[0]->camera_center(Vector2());
    for ( size_t i = 0; i < camera_models.size(); i++ ) {
      CAHVORModel* cam =
        dynamic_cast<CAHVORModel*>( camera_models[i].get() );
      cam->C -= origin;
      image_infos[i].easting -= origin[0];
      image_infos[i].northing -= origin[1];
      image_infos[i].height -= origin[2];
    }
  }

  if ( !opt.cnet_file.empty() ) {
    // Loading a Control Network

    vw_out() << "Loading control network from file: " << opt.cnet_file << "\n";
    opt.cnet->read_binary(opt.cnet_file);

  } else {
    // Building a Control Network
    build_control_network( *opt.cnet, camera_models,
                           opt.image_files, opt.min_matches );
    add_ground_control_points( *opt.cnet, opt.image_files,
                               opt.gcp_files.begin(), opt.gcp_files.end() );

    opt.cnet->write_binary("rmax_adjust");
  }

  HelicopterBundleAdjustmentModel ba_model(image_infos, opt.cnet);
  AdjustRobustSparse<HelicopterBundleAdjustmentModel, L2Error> bundle_adjuster(ba_model, L2Error(),
                                                                               true, false );

  if ( opt.lambda > 0 )
    bundle_adjuster.set_lambda(opt.lambda);

  //Clearing the monitoring text files to be used for saving camera params
  if (opt.save_bundlevis){
    std::ofstream ostr("iterCameraParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
    ostr.open("iterPointsParam.txt",std::ios::out);
    ostr << "";
    ostr.close();

    //Now I'm going to save the initial starting position of the cameras
    ba_model.write_adjusted_cameras_append("iterCameraParam.txt");
    std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
    for (size_t i = 0; i < ba_model.num_points(); ++i){
      Vector<double,3> const& current_point = ba_model.B_parameters(i);
      ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
    }
  }

  // Reporter
  BundleAdjustReport<AdjustRobustSparse<HelicopterBundleAdjustmentModel, L2Error> >
    reporter( "RMAX Adjust", ba_model, bundle_adjuster, opt.report_level );

  // Performing Bundle Adjustment
  double abs_tol = 1e10, rel_tol=1e10, overall_delta;
  while(true) {
    reporter.loop_tie_in();

    overall_delta = bundle_adjuster.update(abs_tol, rel_tol);

    // Writing Current Camera Parameters to file for later reading in MATLAB
    if (opt.save_bundlevis) {

      //Writing this iterations camera data
      ba_model.write_adjusted_cameras_append("iterCameraParam.txt");

      //Writing this iterations point data, also saving the pixel param data
      std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
      ostr_points.precision(15);
      for (size_t i = 0; i < ba_model.num_points(); ++i){
        Vector<double,3> const& current_point = ba_model.B_parameters(i);
        ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
      }
    }

    if ( bundle_adjuster.iterations() > opt.max_iterations ) {
      reporter() << "Terminated on Max Iterations\n";
      break;
    }
    if ( abs_tol < 0.2 ) {
      reporter() << "Terminated on Absolute Tolerance\n";
      break;
    }
    if ( rel_tol < 1e-5 ) {
      reporter() << "Terminated on Relative Tolerance\n";
      break;
    }
  }
  reporter.end_tie_in();

  for (size_t i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, asp::prefix_from_filename(opt.image_files[i])+".rmax_adjust");
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    do_ba(opt);
  } ASP_STANDARD_CATCHES;

  return 0;
}
