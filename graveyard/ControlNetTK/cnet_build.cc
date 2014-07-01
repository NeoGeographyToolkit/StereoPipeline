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


// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
using namespace vw;
using namespace vw::ba;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
using namespace vw::camera;

template<typename In, typename Out, typename Pred>
Out copy_if(In first, In last, Out res, Pred Pr)
{
  while (first != last)
  {
    if (Pr(*first))
      *res++ = *first;
    ++first;
  }
  return res;
}

// This sifts out from a vector of strings, a listing of GCPs.  This
// should be useful for those programs who accept their data in a mass
// input vector.
bool IsGCP( std::string const& name ) {
  return boost::iends_with(name,".gcp");
}

template <class IContainT, class OContainT>
void sort_out_gcp( IContainT& input, OContainT& output ) {
  copy_if( input.begin(), input.end(),
           std::back_inserter(output), IsGCP );
  typename IContainT::iterator new_end =
    std::remove_if(input.begin(), input.end(), IsGCP);
  input.erase(new_end,input.end());
}

// This sifts out from a vector of strings, a listing of input CNET
// GCPs. This should be useful for those programs who accept their data
// in a mass input vector.
bool IsGCPCnet( std::string const& name ) {
  return boost::iends_with(name,".net") || boost::iends_with(name,".cnet");
}

template <class IContainT, class OContainT>
void sort_out_gcpcnets( IContainT& input, OContainT& output ) {
  copy_if( input.begin(), input.end(),
           std::back_inserter(output), IsGCPCnet );
  typename IContainT::iterator new_end =
    std::remove_if(input.begin(), input.end(), IsGCPCnet );
  input.erase(new_end,input.end());
}

struct Options : public asp::BaseOptions {
  // Input
  std::vector<std::string> input_names, gcp_names,
    gcp_cnet_names, serial_names, directory_names;
  int min_matches;
  bool isis_adjust;

  // Output
  std::string cnet_output;
  std::string cnet_output_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("directory,d", po::value(&opt.directory_names),
     "Directory(-ies) to search for match files. Defaults with current directory.")
    ("o,output-cnet", po::value(&opt.cnet_output)->default_value("cnet_built"), "Output file for control network.")
    ("t,type-of-cnet", po::value(&opt.cnet_output_type)->default_value("binary"), "Types of cnets are [binary,isis]")
    ("isis-adjust", po::bool_switch(&opt.isis_adjust)->default_value(false),
     "Use isis_adjust camera models for triangulation")
    ("min-matches", po::value(&opt.min_matches)->default_value(5));
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.input_names));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <isis cube files> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.input_names.empty() )
    vw_throw( ArgumentErr() << "Missing input cube files!\n"
              << usage << general_options );
  sort_out_gcp( opt.input_names, opt.gcp_names );
  sort_out_gcpcnets( opt.input_names, opt.gcp_cnet_names );

  // double checking the string inputs
  boost::to_lower( opt.cnet_output_type );
  if ( opt.directory_names.empty() )
    opt.directory_names.push_back( std::string(".") );
}

int main( int argc, char* argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::vector< boost::shared_ptr<CameraModel> > camera_models;
    opt.serial_names.clear();
    vw_out() << "Loading Camera Models\n";
    TerminalProgressCallback tpc("cnet","");
    double inc_amt = 1.0/double(opt.input_names.size());
    BOOST_FOREACH( std::string const& name, opt.input_names ) {
      std::string adjust_file =
        fs::path( name ).replace_extension("isis_adjust").string();
      if ( opt.isis_adjust && fs::exists( adjust_file ) ) {
        std::ifstream input( adjust_file.c_str() );
        boost::shared_ptr<asp::BaseEquation> position_eq = asp::read_equation(input);
        boost::shared_ptr<asp::BaseEquation> pose_eq = asp::read_equation(input);
        input.close();

        camera_models.push_back( boost::shared_ptr<CameraModel>( new IsisAdjustCameraModel( name, position_eq, pose_eq ) ) );
        opt.serial_names.push_back( boost::dynamic_pointer_cast<IsisAdjustCameraModel>(camera_models.back())->serial_number() );
      } else {
        camera_models.push_back( boost::shared_ptr<CameraModel>( new IsisCameraModel(name) ));
        opt.serial_names.push_back( boost::dynamic_pointer_cast<IsisCameraModel>(camera_models.back())->serial_number() );
      }

      tpc.report_incremental_progress(inc_amt);
    }
    tpc.report_finished();

    vw_out() << "Building Control Network\n";
    ControlNetwork cnet( "ControlNetworkTK" );
    build_control_network( cnet, camera_models,
                           opt.input_names, opt.min_matches,
                           opt.directory_names);
    add_ground_control_points( cnet, opt.input_names,
                               opt.gcp_names.begin(), opt.gcp_names.end() );
    add_ground_control_cnets( cnet, opt.input_names,
                              opt.gcp_cnet_names.begin(),
                              opt.gcp_cnet_names.end() );

    vw_out() << "Applying Serial Numbers\n";
    tpc.report_progress(0);
    inc_amt = 1.0/double(cnet.size());
    // Applying serial numbers
    BOOST_FOREACH( ControlPoint & cp, cnet ) {
      tpc.report_incremental_progress(inc_amt);
      BOOST_FOREACH( ControlMeasure & cm, cp ) {
        if ( cm.ephemeris_time() == 0 ) {
          cm.set_description( "px" );
          cm.set_serial( opt.serial_names[cm.image_id()] );
          cm.set_pixels_dominant(true);
        }
      }
    }
    tpc.report_finished();

    vw_out() << "Saving Control Network\n";
    if ( opt.cnet_output_type == "isis" ) {
      cnet.write_isis(opt.cnet_output);
    } else {
      cnet.write_binary(opt.cnet_output);
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
