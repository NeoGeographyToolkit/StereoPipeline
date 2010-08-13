// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// What's this file supposed to do ?
//
// (Pho)tometry (It)eration (Albedo) Update
//
// With Reflectance
// .... see docs
//
// With out Relectance
//      A = A + sum((I^k-T^k*A)*T^k*S^k)/sum((T^k*S^k)^2)

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateCarreePlateManager.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/PhotometryTK/AlbedoAccumulators.h>
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
  std::string ptk_url;
  int level;

  bool perform_2band;

  // For spawning multiple jobs
  int job_id, num_jobs;
};

template <class AccumulatorT>
void initial_albedo( Options const& opt, ProjectMeta const& ptk_meta,
                     boost::shared_ptr<PlateFile> drg_plate,
                     boost::shared_ptr<PlateFile> albedo_plate,
                     boost::shared_ptr<PlateFile> /*reflect_plate*/,
                     std::list<BBox2i> const& workunits,
                     std::vector<double> const& exposure_ts ) {
  int32 max_tid = ptk_meta.max_iterations() *
    ptk_meta.num_cameras();
  int32 tile_size = albedo_plate->default_tile_size();
  std::ostringstream ostr;
  ostr << "Albedo Initialize [id=" << opt.job_id << "]";
  int transaction_id =
    albedo_plate->transaction_request(ostr.str(),-1);
  TerminalProgressCallback tpc("photometrytk", "Initial");
  double tpc_inc = 1.0/float(workunits.size());
  albedo_plate->write_request();
  ImageView<PixelGrayA<float> > image_temp;

  if ( ptk_meta.reflectance() == ProjectMeta::NONE ) {
    AccumulatorT accum( tile_size, tile_size );
    BOOST_FOREACH(const BBox2i& workunit, workunits) {
      tpc.report_incremental_progress( tpc_inc );

      // See if there's any tiles in this area to begin with
      std::list<TileHeader> h_tile_records;
      h_tile_records =
        drg_plate->search_by_location( workunit.min().x()/8,
                                       workunit.min().y()/8,
                                       opt.level - 3,
                                       0, max_tid, true );
      if ( h_tile_records.empty() )
        continue;

      for ( int ix = workunit.min().x(); ix < workunit.max().x(); ix++ ) {
        for ( int iy = workunit.min().y(); iy < workunit.max().y(); iy++ ) {
          // Polling for DRG Tiles
          std::list<TileHeader> tile_records =
            drg_plate->search_by_location( ix, iy, opt.level,
                                           0, max_tid, true );

          // No Tiles? No Problem!
          if ( tile_records.empty() )
            continue;

          // Feeding accumulator
          BOOST_FOREACH( const TileHeader& tile, tile_records ) {
            drg_plate->read( image_temp, ix, iy, opt.level,
                             tile.transaction_id(), true );

            accum(image_temp, exposure_ts[tile.transaction_id()-1]);
          }
          image_temp = accum.result();

          // Write result
          albedo_plate->write_update(image_temp, ix, iy,
                                     opt.level, transaction_id);
        } // end for iy
      }   // end for ix
    }     // end foreach
  } else {
    AlbedoInitAccumulator<PixelGrayA<float> > accum( tile_size, tile_size );
    vw_throw( NoImplErr() << "Sorry, reflectance code is incomplete.\n" );
  }

  tpc.report_finished();
  albedo_plate->write_complete();
  albedo_plate->transaction_complete(transaction_id,true);
}

void update_albedo( Options const& opt, ProjectMeta const& ptk_meta,
                    boost::shared_ptr<PlateFile> drg_plate,
                    boost::shared_ptr<PlateFile> albedo_plate,
                    boost::shared_ptr<PlateFile> /*reflect_plate*/,
                    std::list<BBox2i> const& workunits,
                    std::vector<double> const& exposure_ts ) {
  int32 max_tid = ptk_meta.max_iterations() *
    ptk_meta.num_cameras();
  int32 tile_size = albedo_plate->default_tile_size();
  std::ostringstream ostr;
  ostr << "Albedo Update [id=" << opt.job_id << "]";
  int transaction_id =
    albedo_plate->transaction_request(ostr.str(),-1);
  TerminalProgressCallback tpc("photometrytk", "Update");
  double tpc_inc = 1.0/float(workunits.size());
  albedo_plate->write_request();
  ImageView<PixelGrayA<float32> > image_temp, current_albedo;

  if ( ptk_meta.reflectance() == ProjectMeta::NONE ) {
    AlbedoDeltaNRAccumulator<PixelGrayA<float> > accum( tile_size, tile_size );
    BOOST_FOREACH(const BBox2i& workunit, workunits) {
      tpc.report_incremental_progress( tpc_inc );

      // See if there's any tiles in this area to begin with
      std::list<TileHeader> h_tile_records;
      h_tile_records =
        drg_plate->search_by_location( workunit.min().x()/8,
                                       workunit.min().y()/8,
                                       opt.level - 3, 0, max_tid, true );
      if ( h_tile_records.empty() )
        continue;

      for ( int ix = workunit.min().x(); ix < workunit.max().x(); ix++ ) {
        for ( int iy = workunit.min().y(); iy < workunit.max().y(); iy++ ) {

          // Polling for DRG Tiles
          std::list<TileHeader> tile_records =
            drg_plate->search_by_location( ix, iy, opt.level,
                                           0, max_tid, true );

          // No Tiles? No Problem!
          if ( tile_records.empty() )
            continue;

          // Polling for current albedo
          albedo_plate->read( current_albedo, ix, iy,
                              opt.level, -1, true );

          // Feeding accumulator
          BOOST_FOREACH( const TileHeader& tile, tile_records ) {
            drg_plate->read( image_temp, ix, iy, opt.level,
                             tile.transaction_id(), true );

            accum(image_temp, current_albedo,
                  exposure_ts[tile.transaction_id()-1]);
          }
          image_temp = accum.result();

          // Write result
          select_channel(current_albedo,0) += select_channel(image_temp,0);
          albedo_plate->write_update(current_albedo, ix, iy,
                                     opt.level, transaction_id);
        } // end for iy
      }   // end for ix
    }     // end foreach
  } else {
    AlbedoDeltaAccumulator<PixelGrayA<float> > accum( tile_size, tile_size );
    vw_throw( NoImplErr() << "Sorry, reflectance code is incomplete.\n" );
  }

  tpc.report_finished();
  albedo_plate->write_complete();
  albedo_plate->transaction_complete(transaction_id,true);
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("level,l", po::value<int>(&opt.level)->default_value(-1), "Default is to process at lowest level.")
    ("job_id,j", po::value<int>(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value<int>(&opt.num_jobs)->default_value(1), "")
    ("2band", "Perform 2 Band mosaic of albedo. This should be used as a last step.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("ptk_url",  po::value<std::string>(&opt.ptk_url),  "Input PTK Url");

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

  opt.perform_2band = vm.count("2band");

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.ptk_url.empty() )
    vw_throw( ArgumentErr() << "Missing project file url!\n"
              << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Load remote project file
    boost::shared_ptr<RemoteProjectFile> remote_ptk =
      boost::shared_ptr<RemoteProjectFile>( new RemoteProjectFile(opt.ptk_url) );
    ProjectMeta project_info;
    remote_ptk->OpenProjectMeta( project_info );

    // URL decomposition
    std::string hostname, exchange, file_name, plate_prefix, plate_postfix;
    int port;
    asp::pho::parse_url( opt.ptk_url, hostname, port, exchange, file_name );
    plate_prefix = "pf://"+hostname+":"+boost::lexical_cast<std::string>(port)+"/index/";
    plate_postfix = "?cache_size=1";

    // Loading standard plate files
    boost::shared_ptr<PlateFile> drg_plate, albedo_plate, reflect_plate;
    drg_plate =
      boost::shared_ptr<PlateFile>( new PlateFile(plate_prefix+"DRG.plate"+
                                                  plate_postfix,
                                                  "equi", "", 256, "tif",
                                                  VW_PIXEL_GRAYA,
                                                  VW_CHANNEL_UINT8 ) );
    albedo_plate =
      boost::shared_ptr<PlateFile>( new PlateFile(plate_prefix+"Albedo.plate"+
                                                  plate_postfix,
                                                  "equi", "", 256, "tif",
                                                  VW_PIXEL_GRAYA,
                                                  VW_CHANNEL_FLOAT32 ) );
    if ( project_info.reflectance() != ProjectMeta::NONE )
      boost::shared_ptr<PlateFile>( new PlateFile(plate_prefix+"Reflectance.plate"+
                                                  plate_postfix,
                                                  "equi", "", 256, "tif",
                                                  VW_PIXEL_GRAYA,
                                                  VW_CHANNEL_FLOAT32 ) );

    // Setting up working level
    if ( opt.level < 0 )
      opt.level = drg_plate->num_levels() - 1;
    if ( opt.level >= drg_plate->num_levels() )
      vw_throw( ArgumentErr() << "Can't request level higher than available in DRG plate" );

    // Diving up jobs and deciding work units
    std::list<BBox2i> workunits;
    {
      int region_size = pow(2.0,opt.level);
      BBox2i full_region(0,region_size/4,region_size,region_size/2);
      std::list<BBox2i> all_workunits = bbox_tiles(full_region,8,8);
      int count = 0;
      BOOST_FOREACH(const BBox2i& c, all_workunits ) {
        if ( count == opt.num_jobs )
          count = 0;
        if ( count == opt.job_id )
          workunits.push_back(c);
        count++;
      }
    }

    // Build up map with current exposure values
    std::vector<double> exposure_t( project_info.num_cameras() );
    for ( int32 i = 0; i < project_info.num_cameras(); i++ ) {
      CameraMeta current_cam;
      remote_ptk->ReadCameraMeta( i, current_cam );
      exposure_t[i] = current_cam.exposure_t();
    }

    // Double check for an iteration error
    if ( albedo_plate->num_levels() == 0 ||
         opt.level >= albedo_plate->num_levels() ) {
      project_info.set_current_iteration(0);
      remote_ptk->UpdateIteration(0);
    }

    // Determine if we're updating or initializing
    if ( opt.perform_2band ) {
      std::cout << "2 Band Albedo [ iteration "
                << project_info.current_iteration() << " ]\n";
      initial_albedo<Albedo2BandNRAccumulator<PixelGrayA<float> > >( opt, project_info, drg_plate, albedo_plate, reflect_plate, workunits, exposure_t );
    } else {
      if ( project_info.current_iteration() ) {
        std::cout << "Updating Albedo [ iteration "
                  << project_info.current_iteration() << " ]\n";
        update_albedo( opt, project_info, drg_plate,
                       albedo_plate, reflect_plate, workunits, exposure_t );
      } else {
        std::cout << "Initialize Albedo [ iteration "
                  << project_info.current_iteration() << " ]\n";
        initial_albedo<AlbedoInitNRAccumulator<PixelGrayA<float> > >( opt, project_info, drg_plate, albedo_plate, reflect_plate, workunits, exposure_t );
      }
    }
  } ASP_STANDARD_CATCHES;

  return 0;
}
