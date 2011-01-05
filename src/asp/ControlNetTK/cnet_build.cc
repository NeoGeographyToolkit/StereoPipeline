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

struct Options {
  // Input
  std::vector<std::string> input_names, gcp_names,
    gcp_cnet_names, serial_names;
  int min_matches;
  bool isis_adjust;

  // Output
  std::string cnet_output;
  std::string cnet_output_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("o,output-cnet", po::value(&opt.cnet_output)->default_value("cnet_built"), "Output file for control network.")
    ("t,type-of-cnet", po::value(&opt.cnet_output_type)->default_value("binary"), "Types of cnets are [binary,isis]")
    ("isis-adjust", po::bool_switch(&opt.isis_adjust)->default_value(false),
     "Use isis_adjust camera models for triangulation")
    ("min-matches", po::value(&opt.min_matches)->default_value(5))
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

  // double checking the string inputs
  boost::to_lower( opt.cnet_output_type );
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
        opt.serial_names.push_back( boost::shared_dynamic_cast<IsisAdjustCameraModel>(camera_models.back())->serial_number() );
      } else {
        camera_models.push_back( boost::shared_ptr<CameraModel>( new IsisCameraModel(name) ));
        opt.serial_names.push_back( boost::shared_dynamic_cast<IsisCameraModel>(camera_models.back())->serial_number() );
      }

      tpc.report_incremental_progress(inc_amt);
    }
    tpc.report_finished();

    vw_out() << "Building Control Network\n";
    ControlNetwork cnet( "ControlNetworkTK" );
    build_control_network( cnet, camera_models,
                           opt.input_names, opt.min_matches );
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
