// __BEGIN_LICENSE__
// Copyright (C) 2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// What's this file supposed to do ?
//
// (Pho)tometry (Init)ialize Phofile
//
// It's used to create project file and to solve for the starting ET.
//
// Solving for ET goes a like this. Ti =To*Ro/Ri. Where the R equals
// the average reflectance. To is the starting exposure time .. it
// needs to be provided by the user .. but I'm just going to provide 1.
//
// If we're not calculating R, all ET get defined as 1.

#include <vw/Image.h>
using namespace vw;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // For spawning multiple jobs
  int job_id, num_jobs;

  // Input .. questions about the project file
  int32 max_iterations;
  std::string reflectance_type;
  double rel_tol, abs_tol;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("job_id,j", po::value<int>(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value<int>(&opt.num_jobs)->default_value(1), "")
    ("max_iterations", po::value<int32>(&opt.max_iterations)->default_value(100), "")
    ("reflectance_type", po::value<std::string>(&opt.reflectance_type)->default_value("none"), "Reflectance options are [none, lambertian, gaskall, mcewen]")
    ("rel_tol", po::value<double>(&opt.rel_tol)->default_value(1e-2), "Minimium required amount of improvement between iterations")
    ("abs_tol", po::value<double>(&opt.abs_tol)->default_value(1e-2), "Error shutoff threshold.")
    ("help,h", "Display this help message");

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(general_options).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [programmer hasn't filled this out]\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
  } catch ( const ArgumentErr& e ) {
    vw_out() << e.what() << std::endl;
    return 1;
  } catch ( const Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
