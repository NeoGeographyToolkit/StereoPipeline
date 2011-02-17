// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// This application finds the error statistics of all control points
/// in a control network. It then modifies the control network to
/// remove high error control points.

#include <vw/Core/ProgressCallback.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <asp/Core/Macros.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options {
  std::string cnet_file;
  std::vector<std::string> input_names;
  double std_dev_clip;
  bool dry_run, verify, clip_radius;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("dry-run", po::bool_switch(&opt.dry_run)->default_value(false),
     "Don't actually write the output control network")
    ("verify,v", po::bool_switch(&opt.verify)->default_value(false),
     "Verify that control points were actually clipped")
    ("clip-radius", po::bool_switch(&opt.clip_radius)->default_value(false),
     "If the points don't seem to be near the radius of the moon, clip'em.")
    ("s,std-dev-clip", po::value(&opt.std_dev_clip)->default_value(2),
     "How many std_dev away must a control point be to be clipped.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("cnet-file", po::value(&opt.cnet_file))
    ("input-files", po::value(&opt.input_names));

  po::positional_options_description positional_desc;
  positional_desc.add("cnet-file", 1);
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
  usage << "Usage: " << argv[0] << " [options] <cnet_file> <isis cube files> ...\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.input_names.empty() )
    vw_throw( ArgumentErr() << "Missing input cube files!\n"
              << usage.str() << general_options );
  if ( opt.cnet_file.empty() )
    vw_throw( ArgumentErr() << "Missing input control network!\n"
              << usage.str() << general_options );
}

int main( int argc, char* argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Loading the image data into the camera models. Also applying
    // blank equations to define the cameras
    std::vector< camera::IsisAdjustCameraModel > camera_models;
    std::map<std::string,size_t> serial_to_camera_model;
    std::map<size_t,std::string> id_to_filename;
    {
      bool start=true;
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

        typedef boost::shared_ptr<asp::BaseEquation> shared_eq;
        shared_eq posF, poseF;
        if ( fs::exists( adjust_file ) ) {
          if ( start ) {
            vw_out() << "Using adjust file\n";
            start = false;
          }
          std::ifstream input( adjust_file.c_str() );
          posF = asp::read_equation(input);
          poseF = asp::read_equation(input);
          input.close();
        } else {
          if ( start ) {
            vw_out() << "Not using adjust file\n";
            start = false;
          }
          posF = shared_eq( new asp::PolyEquation( 0 ) );
          poseF = shared_eq( new asp::PolyEquation( 0 ) );
        }
        camera::IsisAdjustCameraModel camera( input, posF, poseF );
        camera_models.push_back( camera );

        serial_to_camera_model[ camera_models.back().serial_number() ] =
          camera_models.size() - 1;
        id_to_filename[ camera_models.size() - 1 ] = input;
      }
      progress.report_finished();
    }

    ba::ControlNetwork cnet("");
    std::vector<std::string> tokens;
    boost::split( tokens, opt.cnet_file, boost::is_any_of(".") );
    if ( tokens[tokens.size()-1] == "net" ) {
      cnet.read_isis( opt.cnet_file );
    } else if ( tokens[tokens.size()-1] == "cnet" ) {
      cnet.read_binary( opt.cnet_file );
    } else {
      vw_throw( IOErr() << "Unknown Control Network file extension, \""
                << tokens[tokens.size()-1] << "\"." );
    }

    vw_out() << "Assigning Indexing:\n";
    BOOST_FOREACH( ba::ControlPoint& cp, cnet ) {
      BOOST_FOREACH( ba::ControlMeasure& cm, cp ) {
        std::map<std::string,size_t>::const_iterator id =
          serial_to_camera_model.find( cm.serial() );
        if ( id != serial_to_camera_model.end() )
          cm.set_image_id( id->second );
        else
          vw_throw( IOErr() << "Control Network has serial not associated with input cameras, \"" << cm.serial() << "\"." );
      }
    }

    vw_out() << "Iterating through control points.\n";
    std::vector<double>  cp_error( cnet.size() );
    std::fill( cp_error.begin(), cp_error.end(), 0 );
    {
      std::vector<double>::iterator error_it = cp_error.begin();
      ba::ControlNetwork::iterator cp_it = cnet.begin();
      while ( cp_it != cnet.end() ) {
        BOOST_FOREACH( ba::ControlMeasure& cm, *cp_it ) {
          Vector2 reprojection =
            camera_models[cm.image_id()].point_to_pixel( cp_it->position() );
          *error_it += norm_2(reprojection-cm.position());
        }
        *error_it /= cp_it->size();

        error_it++; cp_it++;
      }
    }

    // So getting mean and std dev
    MeanAccumulator<double> mean_acc;
    StdDevAccumulator<double> stddev_acc;
    BOOST_FOREACH( double& meas, cp_error ) {
      mean_acc( meas );
      stddev_acc( meas );
    }

    double mean = mean_acc.value();
    double stddev = stddev_acc.value();
    vw_out() << "Mean error   : " << mean << " px\n";
    vw_out() << "StdDev error : " << stddev << " px\n";

    std::vector<double>::reverse_iterator error_it = cp_error.rbegin();
    size_t error_index = cp_error.size()-1;
    size_t delete_count = 0;
    while ( error_it != cp_error.rend() ) {

      // Never delete a ground control point
      if ( cnet[error_index].type() ==
           ba::ControlPoint::GroundControlPoint )
        continue;

      if ( *error_it > opt.std_dev_clip*stddev+mean &&
           *error_it > 2.0 ) {
        cnet.delete_control_point( error_index );
        delete_count++;
      }
      if ( opt.clip_radius ) {
        double radius = norm_2( cnet[error_index].position() );
        if ( radius < 1737100*0.8 ||
             radius > 1737100*1.2 ) {
          cnet.delete_control_point( error_index );
          delete_count++;
        }
      }

      error_index--;
      error_it++;
    }

    vw_out() << "Removed " << delete_count << " of " << cp_error.size()
             << " control points.\n";

    size_t index = 0;
    if ( opt.verify ) {
      BOOST_FOREACH( ba::ControlPoint const& cp, cnet ) {
        double error = 0;
        BOOST_FOREACH( ba::ControlMeasure const& cm, cp ) {
          Vector2 reprojection =
            camera_models[cm.image_id()].point_to_pixel( cp.position() );
          error += norm_2(reprojection-cm.position());
        }
        error /= cp.size();
        if ( error - mean > opt.std_dev_clip*stddev &&
             error > 2.0 ) {
          vw_out() << "ERROR! Control point has error higher than expected.\n";
          vw_out() << "\t" << error << " > "
                   << mean+opt.std_dev_clip*stddev << "\n";
          vw_out() << "index = " << index << "\n";
        }
        index++;
      }
    }

    if ( !opt.dry_run )
      cnet.write_binary(fs::path(opt.cnet_file).stem()+"-clipped.cnet");

  } ASP_STANDARD_CATCHES;

  return 0;
}
