// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file isis_adjust.cc
///
/// Utility for performing bundle adjustment of ISIS3 cube files. This
/// is a highly experimental program and reading of the bundle
/// adjustment chapter is required before use of this program.

#include <asp/Tools/isis_adjust.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::ip;

// Global variables
float g_spacecraft_position_sigma;
float g_spacecraft_pose_sigma;
float g_gcp_scalar;
boost::shared_ptr<ControlNetwork> g_cnet;
po::variables_map g_vm;
std::vector< boost::shared_ptr< IsisAdjustCameraModel > > g_camera_adjust_models;
std::vector<std::string> g_input_files, g_gcp_files;
double g_lambda;
int g_max_iterations;
int g_report_level;
bool g_kml_all;

// Helper Function to allow selection of different Bundle Adjustment Algorithms.
//------------------------------------------------------------------------------
template <class AdjusterT>
void perform_bundleadjustment( typename AdjusterT::cost_type const& cost_function ) {
  // Building the Bundle Adjustment Model and applying the Bundle
  // Adjuster.
  typename AdjusterT::model_type
    ba_model( g_camera_adjust_models , g_cnet, g_input_files,
              g_spacecraft_position_sigma, g_spacecraft_pose_sigma,
              g_gcp_scalar );
  AdjusterT bundle_adjuster( ba_model, cost_function,
                             !g_vm.count("disable-camera-const"),
                             !g_vm.count("disable-gcp-const"));

  // Handling options to modify Bundle Adjuster
  if ( g_vm.count( "lambda" ) )
    bundle_adjuster.set_lambda( g_lambda );
  if ( cost_function.name_tag() != "L2Error" )
    bundle_adjuster.set_control( 1 ); // Shutting off fast Fletcher-style control
  if ( g_vm.count( "seed-with-previous" ) ) {
    vw_out() << "Seeding with previous ISIS adjustment files.\n";
    vw_out() << "\tLoading up previous ISIS adjustments\n";
    for (unsigned j = 0; j < g_input_files.size(); ++j ) {
      std::string adjust_file = prefix_from_filename( g_input_files[j] ) +
        ".isis_adjust";

      // Loading and forcing in the adjustment
      if ( fs::exists( adjust_file ) ) {
        vw_out() << "\t\tFound: " << adjust_file << std::endl;
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
            stereo::StereoModel sm( cameras[m1->image_id()].get(),
                                    cameras[m2->image_id()].get() );
            double error;
            Vector3 triangulation = sm( m1->position(),
                                        m2->position(),
                                        error );
            if ( triangulation != Vector3() ) {
              count++;
              estimate3d += triangulation;
              // Do I want to do anything with the error?
            } else
              vw_out(DebugMessage, "bundle_adjustment") << "Error: Failed at retriangulation.\n";
          }
        }

        estimate3d /= count;
        cp->set_position( estimate3d );
      }
    }

    // Repushing the position in control network into BA model
    {
      vw_out() << "\tPush new triangulation results back into BA model\n";

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
      for ( int i = 0; i < camera->lines(); i+=(camera->lines()/8) ) {
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
  BundleAdjustReport< AdjusterT > reporter( "ISIS Adjust", ba_model, bundle_adjuster, g_report_level);

  // Performing the Bundle Adjustment
  double abs_tol = 1e10, rel_tol = 1e10;
  // This is the sparse implementation of the code
  double overall_delta = 2;
  int no_improvement_count = 0;
  while ( overall_delta ) {
    overall_delta = bundle_adjuster.update( abs_tol, rel_tol );
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
        for ( int i = 0; i < camera->lines(); i+=(camera->lines()/8) ) {
          Vector3 position = camera->camera_center( Vector2(0,i) ); // This calls legacy support
          ostr_camera << std::setprecision(18) << std::setprecision(18) << j << "\t" << position[0] << "\t" << position[1] << "\t" << position[2];
          Quaternion<double> pose = camera->camera_pose( Vector2(0,i) ); // Legacy as well
          pose = pose / norm_2(pose);
          Vector3 euler = rotation_matrix_to_euler_xyz( pose.rotation_matrix() );
          ostr_camera << std::setprecision(18) << "\t" << euler[0] << "\t" << euler[1] << "\t" << euler[2] << std::endl;
        }
      }
    } // end of saving data

    if ( overall_delta == ScalarTypeLimits<double>::highest() )
      no_improvement_count++;
    else
      no_improvement_count = 0;

    // Determine if it is time to quit
    if ( bundle_adjuster.iterations() > g_max_iterations || abs_tol < 0.01
         || rel_tol < 1e-10 || no_improvement_count > 5 )
      break;
  }
  reporter.end_tie_in();

  // Option to write KML of control network
  if ( g_vm.count("write-kml") ) {
    vw_out() << "Writing KML of Control Network.\n";
    reporter.write_control_network_kml( !g_kml_all );
  }

  for ( unsigned int i = 0; i < ba_model.num_cameras(); ++i )
    ba_model.write_adjustment( i, prefix_from_filename( g_input_files[i] ) + ".isis_adjust");

}

// Main Executable
// -----------------------------------------------------------------------
int main(int argc, char* argv[]) {

  std::string cnet_file;
  std::string robust_cost_function;
  std::string bundle_adjustment_type;
  int min_matches;
  double robust_outlier_threshold;
  int polynomial_order;

  // BOOST Program Options code
  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("cost-function", po::value<std::string>(&robust_cost_function)->default_value("L2"),
     "Choose a robust cost function from [PseudoHuber, Huber, L1, L2, Cauchy]")
    ("bundle-adjuster", po::value<std::string>(&bundle_adjustment_type)->default_value("Sparse"),
     "Choose a bundle adjustment version from [Ref, Sparse, RobustRef, RobustSparse]")
    ("disable-camera-const", "Disable camera constraint error")
    ("disable-gcp-const", "Disable GCP constraint error")
    ("gcp-scalar", po::value<float>(&g_gcp_scalar)->default_value(1.0),
     "Sets a scalar to multiply to the sigmas (uncertainty) defined for the gcps. GCP sigmas are defined in the .gcp files.")
    ("lambda,l", po::value<double>(&g_lambda), "Set the intial value of the LM parameter g_lambda")
    ("min-matches", po::value<int>(&min_matches)->default_value(30),
     "Set the minimum number of matches between images that will be considered.")
    ("max-iterations", po::value<int>(&g_max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("poly-order", po::value<int>(&polynomial_order)->default_value(0),
     "Set the order of the polynomial used adjust the camera properties. If using a frame camera, leave at 0 (meaning scalar offsets). For line scan cameras try 2.")
    ("position-sigma", po::value<float>(&g_spacecraft_position_sigma)->default_value(100.0),
     "Set the sigma (uncertainty) of the spacecraft position. (meters)")
    ("pose-sigma", po::value<float>(&g_spacecraft_pose_sigma)->default_value(0.1),
     "Set the sigma (uncertainty) of the spacecraft pose. (radians)")
    ("report-level,r", po::value<int>(&g_report_level)->default_value(10),
     "Changes the detail of the Bundle Adjustment Report")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0),
     "Set the threshold for robust cost functions.")
    ("save-iteration-data,s", "Saves all camera/point/pixel information between iterations for later viewing in Bundlevis")
    ("seed-with-previous", "Use previous isis_adjust files at starting point for this run.")
    ("write-isis-cnet-also", "Writes an ISIS style control network")
    ("write-kml", po::value<bool>(&g_kml_all),
     "Selecting this will cause a kml to be writting of the GCPs, send with a true and it will also write all the 3d estimates")
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
    vw_out() << usage.str() << std::endl;
    return 1;
  }

  if ( g_vm.count("input-files") < 1 ) {
    vw_out() << "Error: Must specify at least one input file!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }
  g_gcp_files = sort_out_gcps( g_input_files );

  // Checking cost function strings
  boost::to_lower( robust_cost_function );
  if ( !( robust_cost_function == "pseudohuber" ||
          robust_cost_function == "huber" ||
          robust_cost_function == "l1" ||
          robust_cost_function == "l2" ||
          robust_cost_function == "cauchy" ) ) {
    vw_out() << "Unknown robust cost function: " << robust_cost_function
              << ". Options are : [ PseudoHuber, Huber, L1, L2, Cauchy]\n";
    exit(1);
  }

  // Checking bundle adjustment type string
  boost::to_lower( bundle_adjustment_type );
  if ( !( bundle_adjustment_type == "ref" ||
          bundle_adjustment_type == "sparse" ||
          bundle_adjustment_type == "robustref" ||
          bundle_adjustment_type == "robustsparse" ) ) {
    vw_out() << "Unknown bundle adjustment version: " << bundle_adjustment_type
              << ". Options are : [Ref, Sparse, RobustRef, RobustSparse]\n";
    exit(1);
  }

  // Loading the image data into the camera models. Also applying
  // blank equations to define the cameras
  std::vector< boost::shared_ptr<CameraModel> > camera_models( g_input_files.size() );
  {
    vw_out() << "Loading Camera Models:\n";
    vw_out() << "----------------------\n";
    TerminalProgressCallback progress(vw::InfoMessage,"Camera Models:");
    progress.report_progress(0);
    for ( unsigned i = 0; i < g_input_files.size(); ++i ) {
      progress.report_progress(float(i)/float(g_input_files.size()));
      vw_out(DebugMessage,"asp") << "Loading: " << g_input_files[i] << std::endl;

      // Equations defining the delta
      boost::shared_ptr<BaseEquation> posF( new PolyEquation(polynomial_order) );
      boost::shared_ptr<BaseEquation> poseF( new PolyEquation(polynomial_order) );
      boost::shared_ptr<CameraModel> p ( new IsisAdjustCameraModel( g_input_files[i], posF, poseF ) );
      camera_models[i] = p;
    }
    progress.report_finished();
  }

  // Checking to see if there is a cnet file to load up
  g_cnet = boost::shared_ptr<ControlNetwork>( new ControlNetwork("IsisAdjust Control Network (in mm)"));
  if ( g_vm.count("cnet") ){
    vw_out() << "Loading control network from file: " << cnet_file << "\n";

    std::vector<std::string> tokens;
    boost::split( tokens, cnet_file, boost::is_any_of(".") );
    if ( tokens[tokens.size()-1] == "net" ) {
      // An ISIS style control network
      g_cnet->read_isis( cnet_file );
    } else if ( tokens[tokens.size()-1] == "cnet" ) {
      // A VW binary style
      g_cnet->read_binary( cnet_file );
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
          vw_throw( InputErr() << "ISIS Adjust doesn't seem to have a camera for serial, \""
                    << (*g_cnet)[i][m].serial() << "\", found in loaded Control Network" );
      }
    }

  } else {
    vw_out() << "Building Control Network:\n";
    vw_out() << "-------------------------\n";
    build_control_network( g_cnet,
                           camera_models,
                           g_input_files,
                           min_matches );
    add_ground_control_points( g_cnet,
                               g_input_files,
                               g_gcp_files );
  }

  // Need to typecast all the models to feed to the Bundle Adjustment
  // model, kinda ugly.
  g_camera_adjust_models.resize( camera_models.size() );
  for ( unsigned j = 0; j < camera_models.size(); ++j )
    g_camera_adjust_models[j] = boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[j]);

  // Double checking to make sure that the Control Network is set up
  // for ephemeris time. We continue to do this with loaded control
  // networks as a strict ISIS style control network will not record
  // Ephemeris Time.
  {
    vw_out() << "Calculating focal plane measurements:\n";
    vw_out() << "-------------------------------------\n";
    TerminalProgressCallback progress;
    progress.report_progress(0);

    // Applying millimeter conversion
    for (unsigned i = 0; i < g_cnet->size(); ++i ) {
      progress.report_progress(float(i)/float(g_cnet->size()));
      for ( ControlPoint::iterator cm = (*g_cnet)[i].begin();
            cm != (*g_cnet)[i].end(); cm++ ) {
        if ( cm->ephemeris_time() == 0 ) {
          // Loading camera used by measure
          //boost::shared_ptr< IsisAdjustCameraModel > cam =
          //  boost::shared_dynamic_cast< IsisAdjustCameraModel >( camera_models[ (*g_cnet)[i][m].image_id() ] );

          Vector3 mm_time = g_camera_adjust_models[cm->image_id()]->pixel_to_mm_time( cm->position() );
          cm->set_focalplane( mm_time[0], mm_time[1] );
          cm->set_ephemeris_time( mm_time[2] );
          cm->set_description( "millimeters" );
          cm->set_serial( g_camera_adjust_models[cm->image_id()]->serial_number() );
          cm->set_pixels_dominant( false );

          Vector2 mm_over_pixel = g_camera_adjust_models[cm->image_id()]->mm_over_pixel( cm->position() );

          Vector2 sigma = cm->sigma();
          sigma = elem_prod( sigma, mm_over_pixel );
          cm->set_sigma( sigma );
        }
      }
    }
    progress.report_finished();

    // Writing ISIS Control Network
    g_cnet->write_binary("isis_adjust");
    vw_out() << "\n";
  }

  VW_DEBUG_ASSERT( g_cnet->size() != 0, vw::MathErr() << "Control network conversion error to millimeter time" );

  // Option to write ISIS-style control network
  if ( g_vm.count("write-isis-cnet-also") ) {
    vw_out() << "Writing ISIS-style Control Network.\n";
    g_cnet->write_isis("isis_adjust");
  }

  // Switching based on cost function
  {
    typedef IsisBundleAdjustmentModel<3,3> ModelType;

    if ( bundle_adjustment_type == "ref" ) {
      if ( robust_cost_function == "pseudohuber" ) {
        perform_bundleadjustment<BundleAdjustmentRef< ModelType, PseudoHuberError > >( PseudoHuberError(robust_outlier_threshold) );
      } else if ( robust_cost_function == "huber" ) {
        perform_bundleadjustment<BundleAdjustmentRef< ModelType, HuberError > >( HuberError(robust_outlier_threshold) );
      } else if ( robust_cost_function == "l1" ) {
        perform_bundleadjustment<BundleAdjustmentRef< ModelType, L1Error > >( L1Error() );
      } else if ( robust_cost_function == "l2" ) {
        perform_bundleadjustment<BundleAdjustmentRef< ModelType, L2Error > >( L2Error() );
      } else if ( robust_cost_function == "cauchy" ) {
        perform_bundleadjustment<BundleAdjustmentRef< ModelType, CauchyError > >( CauchyError(robust_outlier_threshold) );
      }
    } else if ( bundle_adjustment_type == "sparse" ) {
      if ( robust_cost_function == "pseudohuber" ) {
        perform_bundleadjustment<BundleAdjustmentSparse< ModelType, PseudoHuberError > >( PseudoHuberError(robust_outlier_threshold) );
      } else if ( robust_cost_function == "huber" ) {
        perform_bundleadjustment<BundleAdjustmentSparse< ModelType, HuberError > >( HuberError(robust_outlier_threshold) );
      } else if ( robust_cost_function == "l1" ) {
        perform_bundleadjustment<BundleAdjustmentSparse< ModelType, L1Error > >( L1Error() );
      } else if ( robust_cost_function == "l2" ) {
        perform_bundleadjustment<BundleAdjustmentSparse< ModelType, L2Error > >( L2Error() );
      } else if ( robust_cost_function == "cauchy" ) {
        perform_bundleadjustment<BundleAdjustmentSparse< ModelType, CauchyError > >( CauchyError(robust_outlier_threshold) );
      }
    } else if ( bundle_adjustment_type == "robustref" ) {
      if ( robust_cost_function == "l2" ) {
        perform_bundleadjustment<BundleAdjustmentRobustRef< ModelType,L2Error> >( L2Error() );
      } else {
        vw_out() << "Robust Reference implementation doesn't allow the selection of different cost functions. Exiting!\n\n";
        exit(1);
      }
    } else if ( bundle_adjustment_type == "robustsparse" ) {
      if ( robust_cost_function == "l2" ) {
        perform_bundleadjustment<BundleAdjustmentRobustSparse< ModelType,L2Error> >( L2Error() );
      } else {
        vw_out() << "Robust Sparse implementation doesn't allow the selection of different cost functions. Exiting!\n\n";
        exit(1);
      }
    }
  }
  return 0;
}
