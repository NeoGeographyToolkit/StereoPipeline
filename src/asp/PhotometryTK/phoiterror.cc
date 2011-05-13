// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// What's this file supposed to do ?
//
// (Pho)tometry (It)eration (Error) Update
//
// Equation:
//
// e = Sumk( Sumij( ((I - A*T*R)*S)^2 ))
//
// If there's no reflectance, don't multiply by it.

#include <vw/Image.h>
#include <asp/Core/Macros.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/PhotometryTK/ErrorAccumulators.h>
using namespace vw;
using namespace asp::pho;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  Url ptk_url;

  // For spawning multiple jobs
  int job_id, num_jobs, level;
};

void update_error( Options& opt ) {
  RemoteProjectFile remote_ptk( opt.ptk_url );
  ProjectMeta project_info;
  remote_ptk.get_project( project_info );

  // Load platefile
  boost::shared_ptr<PlateFile> drg_plate, albedo_plate, reflect_plate;

  // Deciding what cameras
  int minidx, maxidx;
  if (opt.num_jobs > 0) {
    minidx = float(project_info.num_cameras()*opt.job_id)/float(opt.num_jobs);
    maxidx = float(project_info.num_cameras()*(opt.job_id+1))/float(opt.num_jobs);
  
    remote_ptk.get_platefiles(drg_plate,albedo_plate,reflect_plate);

    if (opt.level < 0 )
      opt.level = drg_plate->num_levels() - 1;
  }
  else {
    minidx = 0;
    maxidx = 0;
  }

  for(int j = minidx; j < maxidx; j++) {
    CameraMeta cam_info;
    remote_ptk.get_camera(j, cam_info);

    std::cerr << "Beginning error calculation for cam[" << j << "]\n";

    ErrorNRAccumulatorFunc<double,Vector2i> funcProto(opt.level, j+1, cam_info.exposure_t(), drg_plate, albedo_plate);
    
    //std::cerr << "Camera[" << j << "]         exposure time: "
    //        << cam_info.exposure_t() << "\n";

    int32 full = 1 << opt.level;
    int32 quarter = full/4;
    BBox2i affected_tiles(0,quarter,full-1,quarter*2-1);

    RecursiveBBoxAccumulator<ErrorNRAccumulatorFunc<double,Vector2i> > accum(32, funcProto);

    ErrorNRAccumulatorFunc<double,Vector2i> funcResult = accum(affected_tiles);

    std::cerr << "cam[" << j << "] error=[" << funcResult.value() << "]\n";

    // Once I have the API, call funcResult.value() and 
    // add it to the existing value in the current camera
    if (project_info.current_iteration() == 0) {
      cam_info.set_init_error(funcResult.value());
    }
    else {
      cam_info.set_last_error(cam_info.curr_error());
      cam_info.set_curr_error(funcResult.value());
    }

    remote_ptk.set_camera(j, cam_info);
  }

  if (opt.num_jobs <= 0) {
    // Save last error before we overwrite it
    float32 initError = remote_ptk.get_init_error();
    float32 lastError = remote_ptk.get_last_error();
    float32 currError = remote_ptk.get_curr_error();

    // Compare init error and most recent error:
    std::cerr << "Init error=[" << initError << "]\n";
    std::cerr << "Last error=[" << lastError << "]\n";
    std::cerr << "Curr error=[" << currError << "]\n";
    
    std::cout << initError << " " << lastError << " " << currError << "\n";
  }
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("level,l", po::value(&opt.level)->default_value(-1), "Default is to process lowest level.")
    ("job_id,j", po::value(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value(&opt.num_jobs)->default_value(1), "If num_jobs is 0, don't process anything, just output the error values for the full plate")
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

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.ptk_url == Url() )
    vw_throw( ArgumentErr() << "Missing project file url!\n"
              << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    update_error( opt );
  } ASP_STANDARD_CATCHES;

  return 0;
}
