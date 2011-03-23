// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// What's this file supposed to do ?
//
// (Pho)tometry (It)eration Exposure (Time) Update
//
// With Reflectance
// .... see docs
//
// With out Relectance
//      T = T + sum((Ik-Tk*A)*A*Sk)/sum((A*Sk)^2)

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/PhotometryTK/TimeAccumulators.h>
#include <asp/Core/Macros.h>
using namespace vw;
using namespace vw::platefile;
using namespace asp::pho;

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // Input
  Url ptk_url;
  bool dry_run;

  // For spawning multiple jobs
  int job_id, num_jobs, level;
};

void update_exposure( Options& opt ) {

  // Open up project file
  RemoteProjectFile remote_ptk( opt.ptk_url );
  ProjectMeta project_info;
  remote_ptk.get_project( project_info );

  // Deciding what cameras
  int minidx, maxidx;
  minidx = float(project_info.num_cameras()*opt.job_id)/float(opt.num_jobs);
  maxidx = float(project_info.num_cameras()*(opt.job_id+1))/float(opt.num_jobs);

  // Load platefile
  boost::shared_ptr<PlateFile> drg_plate, albedo_plate, reflect_plate;
  remote_ptk.get_platefiles(drg_plate,albedo_plate,reflect_plate);

  for (int j = minidx; j < maxidx; j++ ) {
    // Pick up current time exposure
    CameraMeta cam_info;
    remote_ptk.get_camera(j, cam_info);
    std::cout << "Camera[" << j << "]         exposure time: "
              << cam_info.exposure_t() << "\n";

    // Deciding working area
    if ( opt.level < 0 )
      opt.level = drg_plate->num_levels() - 1;
    int32 full = 1 << opt.level;
    int32 quarter = full/4;
    BBox2i affected_tiles(0,quarter,full-1,quarter*2-1);
    std::list<TileHeader> drg_tiles =
      drg_plate->search_by_region(opt.level, affected_tiles,j+1,j+1,1);
    ImageView<PixelGrayA<uint8> > drg_temp, albedo_temp;
    std::cout << "Num drg tiles: " << drg_tiles.size() << "\n";

    if ( project_info.reflectance() == ProjectMeta::NONE ) {
      // Accumulating time exposure
      TimeDeltaNRAccumulator taccum(cam_info.exposure_t());

      std::cout << "About to print TimeDeltaNRAccumulator values\n";

      // Updating current time exposure
      BOOST_FOREACH( const TileHeader& drg_tile, drg_tiles ) {
        drg_plate->read( drg_temp, drg_tile.col(), drg_tile.row(),
                         opt.level, j+1, true );
        albedo_plate->read( albedo_temp, drg_tile.col(), drg_tile.row(),
                            opt.level, -1, false );
        for_each_pixel(drg_temp, albedo_temp, taccum);
	std::cout << "iter taccum val=[" << taccum.value() << "]\n";
      }
      cam_info.set_exposure_t(cam_info.exposure_t()+taccum.value());
    } else {
      vw_throw( NoImplErr() << "Sorry, reflectance code is incomplete.\n" );
      // Accumulating time exposure
      TimeDeltaAccumulator taccum(cam_info.exposure_t());

      // Updating current time exposure
    }
    if ( !opt.dry_run ) {
      remote_ptk.set_camera(j, cam_info);

      // Increment iterations
      if ( opt.job_id == 0 )
        remote_ptk.set_iteration(project_info.current_iteration()+1);
    }
    std::cout << "Camera[" << j << "] updated exposure time: "
              << cam_info.exposure_t() << "\n";

  }
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("level,l", po::value(&opt.level)->default_value(-1), "Default is to process lowest level.")
    ("dry-run", "Don't write results")
    ("job_id,j", po::value(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value(&opt.num_jobs)->default_value(1), "")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("ptk_url",  po::value(&opt.ptk_url),  "Input PTK Url");

  po::positional_options_description positional_desc;
  positional_desc.add("ptk_url", 1);

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
  opt.dry_run = vm.count("dry-run");

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.ptk_url.string().empty() )
    vw_throw( ArgumentErr() << "Missing project file url!\n"
              << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    update_exposure( opt );
  } ASP_STANDARD_CATCHES;

  return 0;
}
