// std header
#include <stdlib.h>
#include <iostream>
#include <vector>

// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
using namespace vw;
using namespace vw::ip;

// From this project
#include "equalization.h"

#include <asp/Core/Macros.h>

struct Options {
  std::vector<std::string> match_files;
  unsigned max_points, min_points;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("max-pts,m",po::value(&opt.max_points)->default_value(100),"Max points a pair can have. If it execeeds we trim.")
    ("min-pts,n",po::value(&opt.min_points)->default_value(10),"Minimum points a pair is required to have. Delete if fails this.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.match_files));

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
  usage << "Usage: " << argv[0] << " [options] <match-files> ...\n\n";
  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.match_files.empty() )
    vw_throw( ArgumentErr() << "Must specify at least one input file!\n\n" << usage.str() );
}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    BOOST_FOREACH( std::string const& matchfile, opt.match_files ) {
      vw_out() << "Loading: " << matchfile << "\n";

      std::vector<InterestPoint> ip1, ip2;
      read_binary_match_file( matchfile, ip1, ip2 );
      vw_out() << "\t> Found " << ip1.size() << " matches.\n";

      if ( ip1.size() < opt.min_points ) {
        vw_out() << "Match failed to have enough pairs.\n";
        fs::remove( matchfile );
        continue;
      }

      vw_out() << "Performing equalization\n";
      equalization( ip1, ip2, opt.max_points );

      // Finally write back out the reduced match file
      write_binary_match_file( matchfile, ip1, ip2 );
      vw_out() << "Wrote back: " << matchfile << "\n";
    }
  } ASP_STANDARD_CATCHES;

  return 0;
}
