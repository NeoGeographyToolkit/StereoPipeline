// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// (Pho)tometry (It)eration Collection of min and max pixvals from multithreaded jobs
//
// Reads tmp files that contain min and max pixvals and find global min and max across these.
// 

#include <vw/Plate/PlateFile.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/PhotometryTK/ExtremePixvalAccumulator.h>
#include <asp/Core/Macros.h>
using namespace vw;
using namespace vw::platefile;
using namespace asp::pho;

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  Url ptk_url;
  int32 num_jobs;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
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

    RemoteProjectFile remote_ptk(opt.ptk_url);
    
    ProjectMeta project_info;
    remote_ptk.get_project( project_info );

    float32 minlist[opt.num_jobs];
    float32 maxlist[opt.num_jobs];

    for(int jobid = 0; jobid < opt.num_jobs; jobid++) {
      ostringstream ostr;
      ostr << JOB_PIXVAL_PATH_PREFIX << jobid << JOB_PIXVAL_PATH_SUFFIX;
      std::string path = ostr.str();
      ifstream tmpfile(path.c_str(), std::ios::in);
      float32 minPixval;
      float32 maxPixval;
      tmpfile.read((char*)&minPixval, sizeof(float32));
      tmpfile.read((char*)&maxPixval, sizeof(float32));
      minlist[jobid] = minPixval;
      maxlist[jobid] = maxPixval;
      std::cout << "minlist[" << jobid << "] = " << minPixval << "\n";
      std::cout << "maxlist[" << jobid << "] = " << maxPixval << "\n";
    }

    ExtremePixvalAccumulator<PixelGrayA<float32> > accum(project_info.min_pixval(), project_info.max_pixval());
    for_each(&minlist[0], &minlist[opt.num_jobs], accum);
    
    float32 globalMin, globalMax;
    accum.values(globalMin, globalMax);

    remote_ptk.set_pixvals(globalMin, globalMax);

  } ASP_STANDARD_CATCHES;
}
