// __BEGIN_LICENSE__
// Copyright (C) 2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// What's this file supposed to do ?
//
// (Pho)tometry (Init)ialize (E)xposure (T)ime
//
// This attempts to solve for the starting ephemeris time
// If reflectance equals "none", this excutable does nothing.

#include <vw/Image.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/Core/Macros.h>
using namespace vw;
using namespace asp::pho;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // For spawning multiple jobs
  int job_id, num_jobs;

  // Input
  std::string ptk_url;
};

void do_et_solve( Options const& opt ) {
  RemoteProjectFile remote_ptk( opt.ptk_url );

  // Pull project information
  ProjectMeta proj_meta;
  remote_ptk.OpenProjectMeta( proj_meta );

  if ( proj_meta.reflectance() == ProjectMeta::NONE ) {
    std::cout << "Ah! Nothing to be done!\n";
  } else {
    std::cout << "You've found unfinished code!\n\n";
    std::cout << "Sometime in the future this code should actually\n"
              << "calculate the reflectance. For now, this will just\n"
              << "print the remote information available.\n\n";
  }

  // Debug Testing
  proj_meta.PrintDebugString();
  std::cout << "-- CAMERAS -----------------\n";
  for ( int32 i = 0; i < proj_meta.num_cameras(); i++ ) {
    CameraMeta cam_meta;
    remote_ptk.ReadCameraMeta( i, cam_meta );
    cam_meta.PrintDebugString();
  }
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("job_id,j", po::value<int>(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value<int>(&opt.num_jobs)->default_value(1), "")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("ptkurl", po::value<std::string>(&opt.ptk_url), "Input PTK Url");

  po::positional_options_description positional_desc;
  positional_desc.add("ptkurl", 1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <ptk-url>\n";

  if ( opt.ptk_url.empty() )
    vw_throw( ArgumentErr() << "Missing ptk url.\n"
              << usage.str() << general_options );

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    do_et_solve( opt );
  } ASP_STANDARD_CATCHES;

  return 0;
}
