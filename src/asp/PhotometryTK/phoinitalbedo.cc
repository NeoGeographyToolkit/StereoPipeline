// __BEGIN_LICENSE__
// Copyright (C) 2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// What's this file supposed to do ?
//
// (Pho)tometry (Init)ialize (Albedo)
//
// This starts the albedo and creates it's platefile.
// Eq: A = sum( I^k/(R^k*T^k) )

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateCarreePlateManager.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/PhotometryTK/AlbedoAccumulators.h>
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

  // For spawning multiple jobs
  int job_id, num_jobs;
};

void init_albedo( Options const& opt ) {

  // Open up project file
  RemoteProjectFile remote_ptk( opt.ptk_url );

  // Divey up jobs and find work space
  boost::shared_ptr<PlateFile> drg_platefile =
    boost::shared_ptr<PlateFile>( new PlateFile("pf://index/DRG.plate") );
  std::list<BBox2i> workunits;
  {
    int region_size = pow(2.0,drg_platefile->num_levels()-1);
    BBox2i full_region(0,0,region_size,region_size);
    std::list<BBox2i> all_workunits = bbox_tiles(full_region,4,4);
    int count = 0;
    BOOST_FOREACH(const BBox2i& c, all_workunits ) {
      if ( count == opt.num_jobs )
        count = 0;
      if ( count == opt.job_id )
        workunits.push_back(c);
      count++;
    }
  }

  // Open output and find out current write transaction
  boost::shared_ptr<PlateFile> albedo_platefile =
    boost::shared_ptr<PlateFile>( new PlateFile("pf://index/Albedo.plate",
                                                "equi", "", 256, "tif",
                                                VW_PIXEL_GRAYA,
                                                VW_CHANNEL_UINT8 ) );
  std::ostringstream ostr;
  ostr << "Albedo Initialize [id=" << opt.job_id << "]";
  int transaction_id =
    albedo_platefile->transaction_request(ostr.str(),-1);

  // iterating and processing input
  AlbedoInitNRAccumulator< PixelGrayA<uint8> > albedo_accu( drg_platefile->default_tile_size(), drg_platefile->default_tile_size() );
  ImageView<PixelGrayA<uint8> > image_temp;

  albedo_platefile->write_request();

  TerminalProgressCallback tpc("photometrytk", "");
  double tpc_inc = 1.0/float(workunits.size());

  BOOST_FOREACH(const BBox2i& workunit, workunits) {
    tpc.report_incremental_progress( tpc_inc );

    // See if there's any tiles in this area to begin with
    std::list<TileHeader> h_tile_records;
    h_tile_records = drg_platefile->search_by_location( workunit.min().x()/4,
                                                        workunit.min().y()/4,
                                                        drg_platefile->num_levels()-3,
                                                        0, 1000, true );
    if ( h_tile_records.empty() )
      continue;

    for ( int ix = workunit.min().x(); ix < workunit.max().x(); ix++ ) {
      for ( int iy = workunit.min().y(); iy < workunit.max().y(); iy++ ) {

        // Polling for DRG Tiles
        std::list<TileHeader> tile_records;
        tile_records = drg_platefile->search_by_location( ix, iy,
                                                          drg_platefile->num_levels()-1,
                                                          0, 1000, true );

        // No Tiles? No Problem!
        if ( tile_records.size() == 0 )
          continue;

        // Feeding accumulator
        BOOST_FOREACH( const TileHeader& tile, tile_records ) {
          drg_platefile->read( image_temp, ix, iy,
                               drg_platefile->num_levels()-1,
                               tile.transaction_id(), true );

          // From transaction id figure out the exposure time
          albedo_accu(image_temp, 1);
        }
        image_temp = albedo_accu.result();

        // Write result
        albedo_platefile->write_update(image_temp,ix,iy,
                                       drg_platefile->num_levels()-1,
                                       transaction_id);

      }
    }
  }

  tpc.report_finished();
  albedo_platefile->write_complete();
  albedo_platefile->transaction_complete(transaction_id,true);
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("job_id,j", po::value<int>(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value<int>(&opt.num_jobs)->default_value(1), "")
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
  usage << "Usage: " << argv[0] << " <ptk url>\n";

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

    init_albedo( opt );
  } catch ( const ArgumentErr& e ) {
    vw_out() << e.what() << std::endl;
    return 1;
  } catch ( const Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
