// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file isis_adjust.cc
///
/// Utility for performing bundle adjustment of ISIS3 cube files. This
/// is a highly experimental program and reading of the bundle
/// adjustment chapter is required before use of this program.

#include <asp/Core/Macros.h>
#include <asp/Tools/isis_adjust.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::ba;

struct Options {
  Options() : lambda(-1), seed_previous(false) {}
  // Input
  std::string cnet_file, cost_function, ba_type, output_prefix;
  std::vector<std::string> input_names, gcp_names, gcp_cnet_names;
  double cam_position_sigma, cam_pose_sigma, gcp_scalar,
    lambda, robust_threshold;
  int max_iterations, report_level, min_matches, poly_order;

  // Boolean Settings
  bool seed_previous, disable_camera, disable_gcp,
    save_iteration, write_kml, all_kml, write_isis_cnet;

  // Common Variable
  std::vector<std::string> camera_serials;
  boost::shared_ptr<ControlNetwork> cnet;
  std::vector< boost::shared_ptr< IsisAdjustCameraModel > > camera_models;
};

// Helper Function to allow selection of different Bundle Adjustment Algorithms.
//------------------------------------------------------------------------------
template <class AdjusterT>
void do_ba( typename AdjusterT::cost_type const& cost_function,
                               Options const& opt ) {
  // Building the Bundle Adjustment Model and applying the Bundle
  // Adjuster.
  typename AdjusterT::model_type
    ba_model( opt.camera_models, opt.cnet, opt.input_names,
              opt.cam_position_sigma, opt.cam_pose_sigma,
              opt.gcp_scalar );
  AdjusterT bundle_adjuster( ba_model, cost_function,
                             !opt.disable_camera, !opt.disable_gcp);

  // Handling options to modify Bundle Adjuster
  if ( opt.lambda > 0 )
    bundle_adjuster.set_lambda( opt.lambda );
  if ( cost_function.name_tag() != "L2Error" )
    bundle_adjuster.set_control( 1 ); // Shutting off fast Fletcher-style control
  if ( opt.seed_previous ) {
    vw_out() << "Seeding with previous ISIS adjustment files.\n";
    for (unsigned j = 0; j < opt.input_names.size(); ++j ) {
      std::string adjust_file =
        fs::path( opt.input_names[j] ).replace_extension("isis_adjust").string();

      // Loading and forcing in the adjustment
      if ( fs::exists( adjust_file ) ) {
        // Reading in isis_adjust file
        std::ifstream input( adjust_file.c_str() );
        boost::shared_ptr<asp::BaseEquation> position_eq = asp::read_equation(input);
        boost::shared_ptr<asp::BaseEquation> pose_eq = asp::read_equation(input);
        input.close();
        Vector<double> camera_vector = ba_model.A_parameters( j );
        if ( camera_vector.size() != pose_eq->size()+position_eq->size() )
          vw_throw( IOErr() << "Isis Adjust files have incorrect number parameters for BA session." );

        // Apply said contents to BA's A state
        unsigned insert_i = 0;
        for ( unsigned i = 0; i < position_eq->size(); i++, insert_i++ )
          camera_vector[insert_i] = (*position_eq)[i];
        for ( unsigned i = 0; i < pose_eq->size(); i++, insert_i++ )
          camera_vector[insert_i] = (*pose_eq)[i];
        ba_model.set_A_parameters( j, camera_vector );
      }
    }

    // Retriangulating position of control points in control network
    std::vector<boost::shared_ptr<CameraModel> > camera_models = ba_model.adjusted_cameras();
    BOOST_FOREACH( ControlPoint & cp, *opt.cnet ) {
      triangulate_control_point( cp, camera_models,
                                 8.726646E-2 ); // require 5 degrees
    }

    // Repushing the position in control network into BA model
    vw_out() << "\tPush new triangulation results back into BA model\n";
    for ( unsigned i = 0; i < opt.cnet->size(); ++i )
      ba_model.set_B_parameters( i, (*opt.cnet)[i].position() );
  }

  // Clearing the monitoring text files
  if ( opt.save_iteration ) {
    fs::remove("iterPointsParam.txt");
    fs::remove("iterCameraParam.txt");

    // Write the starting locations
    ba_model.bundlevis_cameras_append("iterCameraParam.txt");
    ba_model.bundlevis_points_append("iterPointsParam.txt");
  }

  // Reporter
  BundleAdjustReport< AdjusterT > reporter( opt.output_prefix, ba_model,
                                            bundle_adjuster, opt.report_level);

  // Performing the Bundle Adjustment
  double abs_tol = 1e10, rel_tol = 1e10;
  // This is the sparse implementation of the code
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

    //Writing recording data for Bundlevis
    if ( opt.save_iteration ) {
      ba_model.bundlevis_cameras_append("iterCameraParam.txt");
      ba_model.bundlevis_points_append("iterPointsParam.txt");
    } // end of saving data

    if ( overall_delta == 0 )
      no_improvement_count++;
    else
      no_improvement_count = 0;
  }
  reporter.end_tie_in();

  // Option to write KML of control network
  if ( opt.write_kml ) {
    vw_out() << "Writing KML of Control Network.\n";
    reporter.write_control_network_kml( !opt.all_kml );
  }

  // Writing out results and applying solution back CNet
  for ( size_t i = 0; i < ba_model.num_cameras(); ++i )
    ba_model.write_adjustment( i, fs::path( opt.input_names[i] ).replace_extension("isis_adjust").string() );
}

// Main Executable
// -----------------------------------------------------------------------
void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("cnet,c", po::value(&opt.cnet_file), "Load a control network from a file")
    ("cost-function", po::value(&opt.cost_function)->default_value("L2"),
     "Choose a robust cost function from [PseudoHuber, Huber, L1, L2, Cauchy]")
    ("bundle-adjuster", po::value(&opt.ba_type)->default_value("Sparse"),
     "Choose a bundle adjustment version from [Ref, Sparse, RobustRef, RobustSparse, RobustSparseKGCP]")
    ("disable-camera-const", po::bool_switch(&opt.disable_camera)->default_value(false),
     "Disable camera constraint error")
    ("disable-gcp-const", po::bool_switch(&opt.disable_gcp)->default_value(false),
     "Disable GCP constraint error")
    ("gcp-scalar", po::value(&opt.gcp_scalar)->default_value(1.0),
     "Sets a scalar to multiply to the sigmas (uncertainty) defined for the gcps. GCP sigmas are defined in the .gcp files.")
    ("lambda,l", po::value(&opt.lambda), "Set the intial value of the LM parameter g_lambda")
    ("min-matches", po::value(&opt.min_matches)->default_value(5),
     "Set the minimum number of matches between images that will be considered.")
    ("max-iterations", po::value(&opt.max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("output-prefix,o", po::value(&opt.output_prefix)->default_value("isis_adjust"), "Output files use this prefix")
    ("poly-order", po::value(&opt.poly_order)->default_value(0),
     "Set the order of the polynomial used adjust the camera properties. If using a frame camera, leave at 0 (meaning scalar offsets). For line scan cameras try 2.")
    ("position-sigma", po::value(&opt.cam_position_sigma)->default_value(100.0),
     "Set the sigma (uncertainty) of the spacecraft position. (meters)")
    ("pose-sigma", po::value(&opt.cam_pose_sigma)->default_value(0.1),
     "Set the sigma (uncertainty) of the spacecraft pose. (radians)")
    ("report-level,r", po::value(&opt.report_level)->default_value(10),
     "Changes the detail of the Bundle Adjustment Report")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(10.0),
     "Set the threshold for robust cost functions.")
    ("save-iteration-data,s", po::bool_switch(&opt.save_iteration)->default_value(false),
     "Saves all camera/point/pixel information between iterations for later viewing in Bundlevis")
    ("seed-with-previous", po::bool_switch(&opt.seed_previous)->default_value(false),
     "Use previous isis_adjust files at starting point for this run.")
    ("write-isis-cnet-also", po::bool_switch(&opt.write_isis_cnet)->default_value(false),
     "Writes an ISIS style control network")
    ("write-kml", po::value(&opt.all_kml),
     "Selecting this will cause a kml to be writting of the GCPs, send with a true and it will also write all the 3d estimates")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.input_names));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e ) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <isis cube files> ...\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.input_names.empty() )
    vw_throw( ArgumentErr() << "Missing input cube files!\n"
              << usage.str() << general_options );
  sort_out_gcp( opt.input_names, opt.gcp_names );
  sort_out_gcpcnets( opt.input_names, opt.gcp_cnet_names );

  opt.write_kml = vm.count("write-kml");

  // double checking the string inputs
  boost::to_lower( opt.cost_function );
  if ( !( opt.cost_function == "pseudohuber" ||
          opt.cost_function == "huber" ||
          opt.cost_function == "l1" ||
          opt.cost_function == "l2" ||
          opt.cost_function == "cauchy" ) )
    vw_throw( ArgumentErr() << "Unknown robust cost function: " << opt.cost_function
              << ". Options are : [ PseudoHuber, Huber, L1, L2, Cauchy]\n" );
  boost::to_lower( opt.ba_type );
  if ( !( opt.ba_type == "ref" ||
          opt.ba_type == "sparse" ||
          opt.ba_type == "robustref" ||
          opt.ba_type == "robustsparse" ||
          opt.ba_type == "robustsparsekgcp" ) )
    vw_throw( ArgumentErr() << "Unknown bundle adjustment version: " << opt.ba_type
              << ". Options are : [Ref, Sparse, RobustRef, RobustSparse, RobustSparseKGCP]\n" );
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Loading the image data into the camera models. Also applying
    // blank equations to define the cameras
    std::vector< boost::shared_ptr<CameraModel> > camera_models;
    {
      vw_out() << "Loading Camera Models:\n";
      vw_out() << "----------------------\n";
      TerminalProgressCallback progress("asp","Camera Models:");
      progress.report_progress(0);
      double tpc_inc = 1/double(opt.input_names.size());
      BOOST_FOREACH( std::string const& input, opt.input_names ) {
        progress.report_incremental_progress( tpc_inc );
        vw_out(DebugMessage,"asp") << "Loading: " << input << "\n";

        std::string adjust_file =
          fs::path( input ).replace_extension("isis_adjust").string();

        boost::shared_ptr<asp::BaseEquation> posF( new asp::PolyEquation( opt.poly_order ) );
        boost::shared_ptr<asp::BaseEquation> poseF( new asp::PolyEquation( opt.poly_order ) );
        if ( fs::exists( adjust_file ) && opt.seed_previous ) {
          vw_out(DebugMessage,"asp") << "Loading already adjusted.\n";
          std::ifstream input( adjust_file.c_str() );
          boost::shared_ptr<asp::BaseEquation> posF = asp::read_equation(input);
          boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(input);
          input.close();
        }
        boost::shared_ptr<CameraModel> p ( new IsisAdjustCameraModel( input, posF, poseF ) );
        camera_models.push_back( p );
      }
      progress.report_finished();
    }

    // Need to typecast all the models to feed to the Bundle Adjustment
    // model, kinda ugly.
    opt.camera_models.resize( camera_models.size() );
    for ( unsigned j = 0; j < camera_models.size(); ++j )
      opt.camera_models[j] = boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[j]);

    // Extract serials
    opt.camera_serials.clear();
    BOOST_FOREACH( boost::shared_ptr<IsisAdjustCameraModel> ccam, opt.camera_models ) {
      opt.camera_serials.push_back( ccam->serial_number() );
    }

    // Loading/Building Control Network
    opt.cnet = boost::shared_ptr<ControlNetwork>( new ControlNetwork("IsisAdjust") );
    if ( !opt.cnet_file.empty() ) {
      vw_out() << "Loading control network from file: " << opt.cnet_file << "\n";

      std::vector<std::string> tokens;
      boost::split( tokens, opt.cnet_file, boost::is_any_of(".") );
      if ( tokens[tokens.size()-1] == "net" ) {
        // An ISIS style control network
        opt.cnet->read_isis( opt.cnet_file );
      } else if ( tokens[tokens.size()-1] == "cnet" ) {
        // A VW binary style
        opt.cnet->read_binary( opt.cnet_file );
      } else {
        vw_throw( IOErr() << "Unknown Control Network file extension, \""
                  << tokens[tokens.size()-1] << "\"." );
      }

      // Assigning camera id number for Control Measures
      BOOST_FOREACH( ControlPoint & cp, *opt.cnet ) {
        BOOST_FOREACH( ControlMeasure & cm, cp ) {
          bool found = false;
          for ( unsigned s = 0; s < opt.camera_serials.size(); ++s ) {
            if ( cm.serial() == opt.camera_serials[s] ) {
              cm.set_image_id( s );
              found = true;
              break;
            }
          }
          if (!found)
            vw_throw( InputErr() << "ISIS Adjust doesn't seem to have a camera for serial, \""
                      << cm.serial() << "\", found in loaded Control Network" );
        }

        // See if we need to triangulate the Control Point
        if ( cp.position() == Vector3() ) {
          triangulate_control_point( cp, camera_models,
                                     8.726646E-2 ); // require 5 degrees
        } // end cp check
      }
    } else {
      vw_out() << "Building Control Network:\n";
      vw_out() << "-------------------------\n";
      build_control_network( (*opt.cnet), camera_models,
                             opt.input_names,
                             opt.min_matches );
    }

    // Load up GCPs
    add_ground_control_points( (*opt.cnet), opt.input_names,
                               opt.gcp_names.begin(), opt.gcp_names.end() );
    add_ground_control_cnets( (*opt.cnet), opt.input_names,
                              opt.gcp_cnet_names.begin(),
                              opt.gcp_cnet_names.end() );

    {
      vw_out() << "Applying serial numbers:\n";
      vw_out() << "-------------------------------------\n";
      TerminalProgressCallback progress("asp","");
      progress.report_progress(0);
      double tpc_inc = 1.0/double(opt.cnet->size());

      BOOST_FOREACH( ControlPoint & cp, *opt.cnet ) {
        progress.report_incremental_progress( tpc_inc );
        BOOST_FOREACH( ControlMeasure & cm, cp ) {
          if ( cm.ephemeris_time() == 0 ) {
            cm.set_description( "px" );
            cm.set_serial( opt.camera_serials[cm.image_id()] );
            cm.set_pixels_dominant(true);
          }
        }
      }
      progress.report_finished();
    }

    VW_DEBUG_ASSERT( opt.cnet->size() != 0, vw::MathErr() << "Control network build error" );

    // Switching based on cost function
    {
      typedef IsisBundleAdjustmentModel<3,3> ModelType;

      if ( opt.ba_type == "ref" ) {
        if ( opt.cost_function == "pseudohuber" ) {
          do_ba<AdjustRef< ModelType, PseudoHuberError > >( PseudoHuberError(opt.robust_threshold), opt );
        } else if ( opt.cost_function == "huber" ) {
          do_ba<AdjustRef< ModelType, HuberError > >( HuberError(opt.robust_threshold), opt );
        } else if ( opt.cost_function == "l1" ) {
          do_ba<AdjustRef< ModelType, L1Error > >( L1Error(), opt );
        } else if ( opt.cost_function == "l2" ) {
          do_ba<AdjustRef< ModelType, L2Error > >( L2Error(), opt );
        } else if ( opt.cost_function == "cauchy" ) {
          do_ba<AdjustRef< ModelType, CauchyError > >( CauchyError(opt.robust_threshold), opt );
        }
      } else if ( opt.ba_type == "sparse" ) {
        if ( opt.cost_function == "pseudohuber" ) {
          do_ba<AdjustSparse< ModelType, PseudoHuberError > >( PseudoHuberError(opt.robust_threshold), opt );
        } else if ( opt.cost_function == "huber" ) {
          do_ba<AdjustSparse< ModelType, HuberError > >( HuberError(opt.robust_threshold), opt );
        } else if ( opt.cost_function == "l1" ) {
          do_ba<AdjustSparse< ModelType, L1Error > >( L1Error(), opt );
        } else if ( opt.cost_function == "l2" ) {
          do_ba<AdjustSparse< ModelType, L2Error > >( L2Error(), opt );
        } else if ( opt.cost_function == "cauchy" ) {
          do_ba<AdjustSparse< ModelType, CauchyError > >( CauchyError(opt.robust_threshold), opt );
        }
      } else if ( opt.ba_type == "robustref" ) {
        if ( opt.cost_function == "l2" ) {
          do_ba<AdjustRobustRef< ModelType,L2Error> >( L2Error(), opt );
        } else {
          vw_out() << "Robust Reference implementation doesn't allow the selection of different cost functions. Exiting!\n\n";
          exit(1);
        }
      } else if ( opt.ba_type == "robustsparse" ) {
        if ( opt.cost_function == "l2" ) {
          do_ba<AdjustRobustSparse< ModelType,L2Error> >( L2Error(), opt );
        } else {
          vw_out() << "Robust Sparse implementation doesn't allow the selection of different cost functions. Exiting!\n\n";
          exit(1);
        }
      } else if ( opt.ba_type == "robustsparsekgcp" ) {
        if ( opt.cost_function == "l2" ) {
          do_ba<AdjustRobustSparseKGCP< ModelType,L2Error> >( L2Error(), opt );
        } else {
          vw_out() << "Robust Sparse implementation doesn't allow the selection of different cost functions. Exiting!\n\n";
          exit(1);
        }
      }
    }

    // Writing ISIS Control Network
    opt.cnet->write_binary(opt.output_prefix);
    vw_out() << "\n";

    // Option to write ISIS-style control network
    if ( opt.write_isis_cnet ) {
      vw_out() << "Writing ISIS-style Control Network.\n";
      opt.cnet->write_isis(opt.output_prefix);
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
