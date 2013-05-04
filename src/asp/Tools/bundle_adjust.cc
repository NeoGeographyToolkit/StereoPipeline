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


/// \file bundle_adjust.cc
///

#include <asp/Core/Macros.h>
#include <asp/Tools/bundle_adjust.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

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

struct Options : public asp::BaseOptions {
  std::vector<std::string> image_files, gcp_files;
  std::string cnet_file, stereosession_type, ba_type;

  double lambda, robust_outlier_threshold;
  int report_level, min_matches, max_iterations;

  bool save_iteration;

  boost::shared_ptr<ControlNetwork> cnet;
  std::vector<boost::shared_ptr<CameraModel> > camera_models;
};

template <class AdjusterT>
void do_ba( typename  AdjusterT::cost_type const& cost_function,
            Options const& opt ) {
  BundleAdjustmentModel ba_model(opt.camera_models, opt.cnet);
  AdjusterT bundle_adjuster(ba_model, cost_function, false, false);

  if ( opt.lambda > 0 )
    bundle_adjuster.set_lambda( opt.lambda );

  //Clearing the monitoring text files to be used for saving camera params
  if (opt.save_iteration){
    fs::remove("iterCameraParam.txt");
    fs::remove("iterPointsParam.txt");

    // Write the starting locations
    ba_model.bundlevis_cameras_append("iterCameraParam.txt");
    ba_model.bundlevis_points_append("iterPointsParam.txt");
  }

  BundleAdjustReport<AdjusterT >
    reporter( "Bundle Adjust", ba_model, bundle_adjuster,
              opt.report_level );

  double abs_tol = 1e10, rel_tol=1e10;
  double overall_delta = 2;
  int no_improvement_count = 0;
  while ( true ) {
    // Determine if it is time to quit
    if ( bundle_adjuster.iterations() >= opt.max_iterations ) {
      reporter() << "Triggered 'Max Iterations'\n";
      break;
    } else if ( abs_tol < 0.01 ) {
      reporter() << "Triggered 'Abs Tol " << abs_tol << " < 0.01'\n";
      break;
    } else if ( rel_tol < 1e-6 ) {
      reporter() << "Triggered 'Rel Tol " << rel_tol << " < 1e-10'\n";
      break;
    } else if ( no_improvement_count > 4 ) {
      reporter() << "Triggered break, unable to improve after "
                 << no_improvement_count << " iterations\n";
      break;
    }

    overall_delta = bundle_adjuster.update( abs_tol, rel_tol );
    reporter.loop_tie_in();

    // Writing Current Camera Parameters to file for later reading
    if (opt.save_iteration) {
      ba_model.bundlevis_cameras_append("iterCameraParam.txt");
      ba_model.bundlevis_points_append("iterPointsParam.txt");
    }
    if ( overall_delta == 0 )
      no_improvement_count++;
    else
      no_improvement_count = 0;
  }
  reporter.end_tie_in();

  for (unsigned i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, fs::path(opt.image_files[i]).replace_extension("adjust").string() );
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("cnet,c", po::value(&opt.cnet_file),
     "Load a control network from a file")
    ("bundle-adjuster", po::value(&opt.ba_type)->default_value("RobustSparse"),
     "Choose a robust cost function from [PseudoHuber, Huber, L1, L2, Cauchy]")
    ("session-type,t", po::value(&opt.stereosession_type)->default_value("isis"),
     "Select the stereo session type to use for processing.")
    ("lambda,l", po::value(&opt.lambda)->default_value(-1),
     "Set the initial value of the LM parameter lambda")
    ("robust-threshold", po::value(&opt.robust_outlier_threshold)->default_value(10.0),
     "Set the threshold for robust cost functions.")
    ("min-matches", po::value(&opt.min_matches)->default_value(30),
     "Set the minimum  number of matches between images that will be considered.")
    ("max-iterations", po::value(&opt.max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("report-level,r",po::value(&opt.report_level)->default_value(10),
     "Changes the detail of the Bundle Adjustment Report")
    ("save-iteration-data,s", "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.image_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <image filenames> ...");
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage);

  if ( opt.image_files.empty() )
    vw_throw( ArgumentErr() << "Missing input cube files!\n"
              << usage << general_options );
  opt.gcp_files = sort_out_gcps( opt.image_files );
  opt.save_iteration = vm.count("save-iteration-data");
  boost::to_lower( opt.stereosession_type );
  boost::to_lower( opt.ba_type );
  if ( !( opt.ba_type == "ref" ||
          opt.ba_type == "sparse" ||
          opt.ba_type == "robustref" ||
          opt.ba_type == "robustsparse" ) )
    vw_throw( ArgumentErr() << "Unknown bundle adjustment version: " << opt.ba_type
              << ". Options are : [Ref, Sparse, RobustRef, RobustSparse]\n" );
}

int main(int argc, char* argv[]) {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Read in the camera model and image info for the input images.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSession::create(opt.stereosession_type));

    if (opt.stereosession_type == "pinhole")
      stereo_settings().keypoint_alignment = true;

    {
      TerminalProgressCallback progress("asp","Camera Models:");
      progress.report_progress(0);
      double tpc_inc = 1/double(opt.image_files.size());
      BOOST_FOREACH( std::string const& input, opt.image_files ) {
        progress.report_incremental_progress(tpc_inc);
        vw_out(DebugMessage,"asp") << "Loading: " << input << "\n";
        if (opt.stereosession_type == "pinhole")
          opt.camera_models.push_back(session->camera_model(input,input));
        else
          opt.camera_models.push_back(session->camera_model(input));
      }
      progress.report_finished();
    }

    opt.cnet.reset( new ControlNetwork("BundleAdjust") );
    if ( opt.cnet_file.empty() ) {
      build_control_network( (*opt.cnet), opt.camera_models,
                             opt.image_files,
                             opt.min_matches );
      add_ground_control_points( (*opt.cnet), opt.image_files,
                                 opt.gcp_files.begin(), opt.gcp_files.end() );

      opt.cnet->write_binary("control");
    } else  {
      std::cout << "Loading control network from file: "
                << opt.cnet_file << "\n";

      // Deciding which Control Network we have
      std::vector<std::string> tokens;
      boost::split( tokens, opt.cnet_file, boost::is_any_of(".") );
      if ( tokens.back() == "net" ) {
        // An ISIS style control network
        opt.cnet->read_isis( opt.cnet_file );
      } else if ( tokens.back() == "cnet" ) {
        // A VW binary style
        opt.cnet->read_binary( opt.cnet_file );
      } else {
        vw_throw( IOErr() << "Unknown Control Network file extension, \""
                  << tokens.back() << "\"." );
      }
    }

    // Switching based on cost function
    {
      typedef BundleAdjustmentModel ModelType;

      if ( opt.ba_type == "ref" ) {
        do_ba<AdjustRef< ModelType, L2Error > >( L2Error(), opt );
      } else if ( opt.ba_type == "sparse" ) {
        do_ba<AdjustSparse< ModelType, L2Error > >( L2Error(), opt );
      } else if ( opt.ba_type == "robustref" ) {
        do_ba<AdjustRobustRef< ModelType,L2Error> >( L2Error(), opt );
      } else if ( opt.ba_type == "robustsparse" ) {
        do_ba<AdjustRobustSparse< ModelType,L2Error> >( L2Error(), opt );
      }
    }

  } ASP_STANDARD_CATCHES;
}
