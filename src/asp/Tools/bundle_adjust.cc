// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file bundle_adjust.cc
///

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

int main(int argc, char* argv[]) {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif

  // Register all stereo session types
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  std::vector<std::string> image_files, gcp_files;
  std::string cnet_file, stereosession_type;
  boost::shared_ptr<ControlNetwork> cnet( new ControlNetwork("My first control network"));
  double lambda;
  double robust_outlier_threshold;
  int report_level;
  int min_matches;

  po::options_description general_options("Options");
  general_options.add_options()
    ("session-type,t", po::value<std::string>(&stereosession_type)->default_value("isis"), "Select the stereo session type to use for processing.")
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("save-iteration-data,s", "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("min-matches", po::value<int>(&min_matches)->default_value(30), "Set the minimum  number of matches between images that will be considered.")
    ("report-level,r",po::value<int>(&report_level)->default_value(10),"Changes the detail of the Bundle Adjustment Report")
    ("help", "Display this help message")
    ("verbose", "Verbose output");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&image_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( po::error &e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  }

  if ( image_files.empty() &&
       !vm.count("cnet") ) {
    std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }
  gcp_files = sort_out_gcps( image_files );

  // Read in the camera model and image info for the input images.
  StereoSession* session = StereoSession::create(stereosession_type);

  if (stereosession_type == "pinhole")
    stereo_settings().keypoint_alignment = true;

  std::vector<boost::shared_ptr<CameraModel> > camera_models;
  {
    TerminalProgressCallback progress("asp","Camera Models:");
    progress.report_progress(0);
    double tpc_inc = 1/double(image_files.size());
    BOOST_FOREACH( std::string const& input, image_files ) {
      progress.report_incremental_progress(tpc_inc);
      vw_out(DebugMessage,"asp") << "Loading: " << input << "\n";
      if (stereosession_type == "pinhole")
        camera_models.push_back(session->camera_model(input,input));
      else
        camera_models.push_back(session->camera_model(input));
    }
    progress.report_finished();
  }

  if (!vm.count("cnet") ) {
    build_control_network( (*cnet), camera_models,
                           image_files,
                           min_matches );
    add_ground_control_points( (*cnet),
                               image_files,
                               gcp_files );

    cnet->write_binary("control");
  } else  {
    std::cout << "Loading control network from file: " << cnet_file << "\n";

    // Deciding which Control Network we have
    std::vector<std::string> tokens;
    boost::split( tokens, cnet_file, boost::is_any_of(".") );
    if ( tokens.back() == "net" ) {
      // An ISIS style control network
      cnet->read_isis( cnet_file );
    } else if ( tokens.back() == "cnet" ) {
      // A VW binary style
      cnet->read_binary( cnet_file );
    } else {
      vw_throw( IOErr() << "Unknown Control Network file extension, \""
                << tokens.back() << "\"." );
    }
  }


  BundleAdjustmentModel ba_model(camera_models, cnet);
  AdjustRobustSparse<BundleAdjustmentModel, L2Error> bundle_adjuster(ba_model, L2Error(), false, false);

  if (vm.count("lambda"))
    bundle_adjuster.set_lambda(lambda);

  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("save-iteration-data")){
    fs::remove("iterCameraParam.txt");
    fs::remove("iterPointsParam.txt");

    // Write the starting locations
    ba_model.bundlevis_cameras_append("iterCameraParam.txt");
    ba_model.bundlevis_points_append("iterPointsParam.txt");
  }

  BundleAdjustReport<AdjustRobustSparse<BundleAdjustmentModel, L2Error> >
    reporter( "Bundle Adjust", ba_model, bundle_adjuster, report_level );

  double abs_tol = 1e10, rel_tol=1e10;
  double overall_delta = 2;
  int no_improvement_count = 0;
  while ( true ) {
    // Determine if it is time to quit
    if ( bundle_adjuster.iterations() >= 20 ) {
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
    if (vm.count("save-iteration-data")) {
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
    ba_model.write_adjustment(i, fs::path(image_files[i]).replace_extension("adjust").string() );
}
