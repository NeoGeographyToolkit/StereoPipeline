// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateView.h>
#include <asp/PhotometryTK/TimeAccumulators.h>
#include <asp/PhotometryTK/AlbedoAccumulators.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
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
  int tilex;
  int tiley;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("tile-x,x", po::value(&opt.tilex)->default_value(-1), "Tile X location.")
    ("tile-y,y", po::value(&opt.tiley)->default_value(-1), "Tile Y location.");

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
  usage << "Usage: " << argv[0] << " -x <tile x pos> -y <tile y pos> <ptk-url>\n"; 
  if (opt.ptk_url.string().empty())
    vw_throw( ArgumentErr() << "Missing project file url!\n"
              << usage.str() << general_options );

  if (opt.tilex < 0)
    vw_throw(ArgumentErr() << "X must be > 0\n" << usage.str() << general_options );
  
  if (opt.tiley < 0)
    vw_throw(ArgumentErr() << "Y must be > 0\n" << usage.str() << general_options );    
}

void dostuff(Options& opt)
{
  std::cout << "url=[" << opt.ptk_url << "] x=[" << opt.tilex << "] y=[" << opt.tiley << "]\n";

  RemoteProjectFile remote_ptk( opt.ptk_url );
  ProjectMeta project_info;
  remote_ptk.get_project( project_info );

  int maxcams = project_info.num_cameras();
  
  boost::shared_ptr<PlateFile> drg_plate, albedo_plate, reflect_plate;
  remote_ptk.get_platefiles(drg_plate,albedo_plate,reflect_plate);
  
  int level = 10;
  int maxIters = 100;
  int pixmapsize = 10; // 10x10 grid of pixels to examine centered on tile

  //PlateView<PixelGrayA<uint8> > croppedDRG(drg_plate);
  //croppedDRG.set_level(level);

  int size = drg_plate->default_tile_size();
  int x = size * opt.tilex;
  int y = size * opt.tiley;

  int pixmapmincol = size/2 + x - pixmapsize/2;
  int pixmapmaxcol = pixmapmincol + pixmapsize;
  int pixmapminrow = size/2 + y - pixmapsize/2;
  int pixmapmaxrow = pixmapminrow + pixmapsize;

  std::cout << "Default tile size=[" << size << "]\n";
  std::cout << "ul tile corner=(" << x << ", " << y << ")\n";
  std::cout << "pixmap ul corner=(" << pixmapmincol << ", " << pixmapminrow << ")\n";
  std::cout << "pixmap lr corner=(" << pixmapmaxcol << ", " << pixmapmaxrow << ")\n";

  BBox2i bbox(opt.tilex, opt.tiley, size, size);
  std::list<TileHeader> all_tiles = drg_plate->search_by_region(level, bbox,0, -1, 0);
  //std::cout << "DRG TILES IN REGION=[" << tiles.size() << "]\n";

  //croppedDRG.search_for_tiles(bbox);
  BOOST_FOREACH(const TileHeader& tileIndex, all_tiles) {
    std::cout << "txid=[" << tileIndex.transaction_id() << "] loc=(" << tileIndex.col() << ", " << tileIndex.row() << ")\n";
  }

  //Index//croppedDRG.search_for_tiles(bbox);

  ImageView<PixelGrayA<uint8> > croppedDRGImg;

  ImageView<PixelGrayA<float32> > croppedAlbedoImg;
  ImageView<PixelGrayA<float32> > image_tmp;
  
  // Sanity check: output bounded tile location as a single image so we can verify we started with a reasonable tile-set.
  
  AlbedoInitNRAccumulator<PixelGrayA<float> > initAccum(size, size);
  AlbedoDeltaNRAccumulator<PixelGrayA<float> > deltaAccum(size, size);

  std::vector<double> exposures;
  exposures.reserve(maxcams);
  
  int iters = 0;
  while(iters < maxIters) {
    vw::fill(croppedDRGImg, 0);
    vw::fill(image_tmp, 0);

    for(int j = 0; j < maxcams; j++) {
      CameraMeta cam_info;
      remote_ptk.get_camera(j, cam_info);

      exposures[j] = cam_info.exposure_t();
    }
      
    // Update Albedo
    BOOST_FOREACH( const TileHeader& drg_tile, all_tiles ) {
      int32 tid = drg_tile.transaction_id();

      // Bad tid
      if (tid < 0 || tid >= maxcams) continue;

      drg_plate->read( croppedDRGImg, drg_tile.col(), drg_tile.row(),
		       level, tid, true );
      
      std::cout << "croppedDRGImg dims=(" << croppedDRGImg.rows() << ", " << croppedDRGImg.cols() << ", " << croppedDRGImg.planes() << ")\n";
      
      if (0 == iters)
	initAccum(image_tmp, exposures[tid-1]);
      else
	deltaAccum(image_tmp, croppedAlbedoImg, exposures[tid-1]);
    }
    if (0 == iters)
      croppedAlbedoImg = initAccum.result();
    else {
      image_tmp = deltaAccum.result();
      select_channel(croppedAlbedoImg, 0) += select_channel(image_tmp, 0);
    }
    
    /****** PRINT 10x10 image region of croppedAlbedoImg HERE *****/

    // Update exposures
    for(int j = 0; j < maxcams; j++) {
      CameraMeta cam_info;
      remote_ptk.get_camera(j, cam_info);
      
      TimeDeltaNRAccumulator taccum(cam_info.exposure_t());
      
      std::list<TileHeader> tiles = drg_plate->search_by_region(level, bbox, j+1, j+1, 0);
      
      BOOST_FOREACH( const TileHeader& drg_tile, tiles) {
	drg_plate->read ( croppedDRGImg, drg_tile.col(), drg_tile.row(),
			  level, j+1, true );
	
      }
      cam_info.set_exposure_t(cam_info.exposure_t()+taccum.value());

      remote_ptk.set_camera(j, cam_info);
    }
  }
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    dostuff(opt);
  } ASP_STANDARD_CATCHES;

  return 0;
}
