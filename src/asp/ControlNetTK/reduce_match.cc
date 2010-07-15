// std header
#include <stdlib.h>
#include <iostream>
#include <vector>

// Boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
using namespace vw;
using namespace vw::ip;

// From this project
#include "equalization.h"

int main( int argc, char* argv[] ) {

  std::vector<std::string> match_files;
  int max_points, min_points;

  po::options_description general_options("Options");
  general_options.add_options()
    ("max-pts,m",po::value<int>(&max_points)->default_value(100),"Max points a pair can have. If it execeeds we trim.")
    ("min-pts,n",po::value<int>(&min_points)->default_value(10),"Minimum points a pair is required to have. Delete if fails this.")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&match_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <match-files> ...\n\n";
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
    vw_out() << usage.str();
    return 1;
  }

  if( match_files.size() < 1 ) {
    vw_out() << "Error: Must specify at least one input file!" << std::endl << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  for ( unsigned i = 0; i < match_files.size(); i++ ) {
    vw_out() << "Attempting to load: " << match_files[i] << "\n";

    std::vector<InterestPoint> ip1, ip2;
    read_binary_match_file( match_files[i], ip1, ip2 );

    vw_out() << "\t>Found " << ip1.size() << " matches.\n\n";

    if ( ip1.size() < min_points ) {
      std::cout << "Match failed to have enough pairs.\n";
      fs::remove( match_files[i] );
      continue;
    }

    vw_out() << "Performing equalization\n";
    equalization( ip1,
                  ip2, max_points );
    vw_out() << "\t> Reduced matches to " << ip1.size() << " pairs.\n";

    // Finally write back out the reduced match file
    vw_out() << "Writing back out\n";
    write_binary_match_file( match_files[i], ip1, ip2 );
  }

  vw_out() << "Finished!\n";
  return 0;
}
