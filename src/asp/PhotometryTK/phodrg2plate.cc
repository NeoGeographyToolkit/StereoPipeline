// __BEGIN_LICENSE__
// Copyright (C) 2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// What's this file supposed to do ?
//
// (Pho)tometry DRG 2 Platefiles
//
// It's supposed to insert and help create 2 things, DRG
// and Reflectance.
//
// Right now .. we support only PlateCarree

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateCarreePlateManager.h>

using namespace vw;
using namespace vw::cartography;
using namespace vw::platefile;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // For spawning multiple jobs
  int job_id, num_jobs;

  // Input
  std::string drg_file, cam_file;

  // Output
  int transaction_id;
};

void do_creation( Options const& opt ) {

  // Load up DRG
  DiskImageView<PixelMask<PixelGray<uint8> > > drg_image( opt.drg_file );
  GeoReference georef;
  read_georeference( georef, opt.drg_file );

  { // Insert DRG
    boost::shared_ptr<PlateFile> drg_plate =
      boost::shared_ptr<PlateFile>( new PlateFile( "pf://index/DRG.plate",
                                                   "equi", "", 256, "tif",
                                                   VW_PIXEL_GRAYA,
                                                   VW_CHANNEL_UINT8 ) );
    PlateCarreePlateManager< PixelGrayA<uint8> > drg_manager( drg_plate );
    drg_manager.insert( mask_to_alpha( drg_image ), opt.drg_file,
                        opt.transaction_id, georef, false,
                        TerminalProgressCallback( "photometrytk",
                                                  "\tProcessing" ) );
  }

  { // Create Reflectance ( at the moment it's just white )
    boost::shared_ptr<PlateFile> ref_plate =
      boost::shared_ptr<PlateFile>( new PlateFile( "pf://index/Reflectance.plate",
                                                   "equi", "", 256, "tif",
                                                   VW_PIXEL_GRAYA,
                                                   VW_CHANNEL_FLOAT32 ) );
    PlateCarreePlateManager< PixelGrayA<float32> > ref_manager( ref_plate );
    ref_manager.insert( mask_to_alpha(copy_mask(ConstantView<PixelGray<float32> >(1.0, drg_image.cols(), drg_image.rows() ), drg_image)),
                        opt.drg_file, opt.transaction_id, georef, false,
                        TerminalProgressCallback( "photometrytk",
                                                  "\tProcessing" ) );
  }


}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("job_id,j", po::value<int>(&opt.job_id)->default_value(0), "")
    ("num_jobs,n", po::value<int>(&opt.num_jobs)->default_value(1), "")
    ("transaction_id,t", po::value<int>(&opt.transaction_id)->default_value(-1), "")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("drg_file", po::value<std::string>(&opt.drg_file), "Input DRG file")
    ("cam_file", po::value<std::string>(&opt.cam_file), "Input Camera file");

  po::positional_options_description positional_desc;
  positional_desc.add("drg_file", 1);
  positional_desc.add("cam_file", 1);

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
  usage << "Usage: " << argv[0] << " [programmer hasn't filled this out]\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.drg_file.empty() )
    vw_throw( ArgumentErr() << "Missing input DRG!\n"
              << usage.str() << general_options );
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

  do_creation( opt );

  return 0;
}
