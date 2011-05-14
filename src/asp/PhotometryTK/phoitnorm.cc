// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// (Pho)tometry (It)eration (Norm)alizing module
//
// Reads min and max pixel values from the Project File and
// uses these to normalize across the entire Albedo.plate
// to 0,1 for optimum viewing.

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/TileManipulation.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace vw;
using namespace vw::platefile;
using namespace asp::pho;

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options : asp::BaseOptions {
  // Input
  Url ptk_url;
  int32 level;
  
  // For spawning multiple jobs
  int32 job_id, num_jobs;
};

void handle_arguments( int argc, char* argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("level,l", po::value(&opt.level)->default_value(-1), "Default is to process at lowest level.")
    ("job_id,j", po::value(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value(&opt.num_jobs)->default_value(1), "");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("ptk_url",  po::value(&opt.ptk_url),  "Input PTK Url");

  po::positional_options_description positional_desc;
  positional_desc.add("ptk_url", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <ptk-url>\n";

  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage.str() );

  if ( opt.ptk_url == Url() )
    vw_throw( ArgumentErr() << "Missing project file url!\n"
              << usage.str() << general_options );  
}

void normalize_plate( Options const& opt, 
                      RemoteProjectFile& remote_ptk,
                      ProjectMeta const& ptk_meta,
                      boost::shared_ptr<PlateFile> drg_plate,
                      boost::shared_ptr<PlateFile> albedo_plate,
                      std::list<BBox2i> const& workunits ) {
  int32 max_tid = ptk_meta.max_iterations() * ptk_meta.num_cameras();
  std::ostringstream ostr;
  ostr << "Albedo Normalization [id=" << opt.job_id << "]";
  int32 transaction_id = albedo_plate->transaction_request(ostr.str(),-1);
  TerminalProgressCallback tpc("photometrytk", "AlbedoNorm");
  double tpc_inc = 1.0/float(workunits.size());
  albedo_plate->write_request();
  ImageView<PixelGrayA<float32> > image_temp;
  ImageView<PixelGray<float32> > image_noalpha;

  float32 minPixval = 0;
  float32 maxPixval = 0;
  remote_ptk.get_and_reset_pixvals(minPixval, maxPixval);

  vw_out() << "phoitnorm: normalizing with old pixvals min=[" << minPixval << "] max=[" << maxPixval << "]\n";

  BOOST_FOREACH(const BBox2i& workunit, workunits) {
    tpc.report_incremental_progress( tpc_inc );
    
    std::list<TileHeader> test_lower_res_tiles;
    test_lower_res_tiles = drg_plate->search_by_location(workunit.min().x()/8,
                                                         workunit.min().y()/8,
                                                         opt.level - 3, 0, 
                                                         max_tid, true );
    if ( test_lower_res_tiles.empty() ) {
      continue;
    }

    for( int32 ix = workunit.min().x(); ix < workunit.max().x(); ix++ ) {
      for( int32 iy = workunit.min().y(); iy < workunit.max().y(); iy++ ) {
        std::list<TileHeader> tiles =
          drg_plate->search_by_location( ix, iy, opt.level,
                                         0, max_tid, true );
        
        if ( tiles.empty() )
          continue;
        
        albedo_plate->read( image_temp, ix, iy,
                            opt.level, -1, true );

        image_temp = normalize( image_temp, minPixval, maxPixval, 0, 1 );

        albedo_plate->write_update(image_temp, ix, iy,
                                   opt.level, transaction_id);
      } // end for iy
    } // end for ix
  } // end foreach

  tpc.report_finished();
  albedo_plate->write_complete();
  albedo_plate->transaction_complete(transaction_id,true);
}

int main( int argc, char *argv[] ) {
  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    
    // Load remote project file
    RemoteProjectFile remote_ptk(opt.ptk_url);

    ProjectMeta project_info;
    remote_ptk.get_project( project_info );

    boost::shared_ptr<PlateFile> drg_plate, albedo_plate, reflect_plate;
    remote_ptk.get_platefiles(drg_plate,albedo_plate,reflect_plate);

    if (opt.level < 0 )
      opt.level = drg_plate->num_levels() - 1;
    if (opt.level >= drg_plate->num_levels() )
      vw_throw( ArgumentErr() << "Can't request level higher than available in DRG plate" );

    std::list<BBox2i> workunits;
    {
      int32 region_size = 1 << opt.level;
      BBox2i full_region(0,region_size/4,region_size,region_size/2);
      std::list<BBox2i> all_workunits = bbox_tiles(full_region,8,8);
      int32 count = 0;
      BOOST_FOREACH(const BBox2i& c, all_workunits ) {
        if ( count == opt.num_jobs )
          count = 0;
        if ( count == opt.job_id )
          workunits.push_back(c);
        count++;      
      }

      normalize_plate(opt, remote_ptk, project_info, drg_plate, albedo_plate, workunits );
    }
  } ASP_STANDARD_CATCHES;
}
