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


/// This application is for use of discovering which cameras contain
/// most of the error after a bundle adjustment. The information
/// produced by this application can help narrow down where better
/// measurements are required.

#include <vw/Core/ProgressCallback.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : public asp::BaseOptions {
  std::string cnet_file;
  std::vector<std::string> input_names;
  bool find_median;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("find-median", po::bool_switch(&opt.find_median)->default_value(false),
     "Find median when reporting high error cameras.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("cnet-file", po::value(&opt.cnet_file))
    ("input-files", po::value(&opt.input_names));

  po::positional_options_description positional_desc;
  positional_desc.add("cnet-file", 1);
  positional_desc.add("input-files", -1);

  std::string usage("<cnet_file> <isis cube files> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.input_names.empty() )
    vw_throw( ArgumentErr() << "Missing input cube files!\n"
              << usage << general_options );
  if ( opt.cnet_file.empty() )
    vw_throw( ArgumentErr() << "Missing input control network!\n"
              << usage << general_options );
}

int main( int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Loading the image data into the camera models. Also applying
    // blank equations to define the cameras
    std::vector< camera::IsisAdjustCameraModel > camera_models;
    std::map<std::string,size_t> serial_to_camera_model;
    std::map<size_t,std::string> id_to_filename;
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

        typedef boost::shared_ptr<asp::BaseEquation> shared_eq;
        shared_eq posF, poseF;
        if ( fs::exists( adjust_file ) ) {
          std::ifstream input( adjust_file.c_str() );
          posF = asp::read_equation(input);
          poseF = asp::read_equation(input);
          input.close();
        } else {
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

    vw_out() << "Iterating through cameras.\n";
    std::vector<double> average_error( camera_models.size() );
    std::vector<size_t> average_count( camera_models.size() );
    std::fill( average_error.begin(), average_error.end(), 0 );
    std::fill( average_count.begin(), average_count.end(), 0 );
    math::CDFAccumulator<double> cdf_accu;
    BOOST_FOREACH( ba::ControlPoint & cp, cnet ) {
      BOOST_FOREACH( ba::ControlMeasure & cm, cp ) {
        Vector2 reprojection =
          camera_models[cm.image_id()].point_to_pixel( cp.position() );
        average_count[cm.image_id()]++;
        double error = norm_2(reprojection-cm.position());
        average_error[cm.image_id()] += error;
        cdf_accu(error);
      }
    }
    cdf_accu.update();
    vw_out() << "CDF [" << cdf_accu.quantile(0.0) << " "
             << cdf_accu.quantile(0.25) << " " << cdf_accu.quantile(0.5)
             << " " << cdf_accu.quantile(0.75) << " "
             << cdf_accu.quantile(1.0) << "]\n";

    // Calculate average and accumulators
    MeanAccumulator<double> mean_acc;
    StdDevAccumulator<double> stddev_acc;
    double max_error = -1;
    size_t index_max_error = 0;
    std::list<size_t> cameras_not_connected;
    for ( size_t i = 0; i < average_error.size(); i++ ) {
      if ( average_count[i] == 0 ) {
        average_error[i] = 0;
        cameras_not_connected.push_back(i);
        continue;
      }
      average_error[i] /= double(average_count[i]);
      mean_acc( average_error[i] );
      stddev_acc( average_error[i] );
      if ( average_error[i] > max_error ) {
        max_error = average_error[i];
        index_max_error = i;
      }
    }

    double mean = mean_acc.value();
    double stddev = stddev_acc.value();
    std::cout << "Mean error  : " << mean << " px\n";
    std::cout << "StdDev error: " << stddev << " px\n";


    // Printing stats
    vw_out() << "\nCameras not connected:\n";
    vw_out() << "-------------------------------------\n";
    BOOST_FOREACH( size_t i, cameras_not_connected ) {
      vw_out() << id_to_filename[i] << "\n";
    }
    vw_out() << "\nCamera of largest error is:\n";
    vw_out() << "-------------------------------------\n";
    vw_out() << id_to_filename[index_max_error] << " ["
             << max_error << " px]\n";
    vw_out() << "\nCameras that are 1 std dev off:\n";
    vw_out() << "-------------------------------------\n";
    for ( size_t i = 0; i < average_error.size(); i++ ) {
      if ( average_error[i] - mean > stddev ) {
        if ( opt.find_median ) {
          MedianAccumulator<double> cmedian_acc;
          StdDevAccumulator<double> cstddev_acc;
          BOOST_FOREACH( ba::ControlPoint const& cp, cnet ) {
            BOOST_FOREACH( ba::ControlMeasure const& cm, cp ) {
              if ( cm.image_id() == i ) {
                Vector2 reprojection =
                  camera_models[cm.image_id()].point_to_pixel( cp.position() );
                double error = norm_2(reprojection-cm.position());
                cmedian_acc(error);
                cstddev_acc(error);
              }
            }
          }
          vw_out() << id_to_filename[i] << "[ m: " << average_error[i]
                   << " std: " << cstddev_acc.value()
                   << " md: " <<  cmedian_acc.value() << " px ]\n";
        } else {
          vw_out() << id_to_filename[i] << " ["
                   << average_error[i] << " px]\n";
        }
      }
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
