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
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/Core/Macros.h>

using namespace vw;
using namespace vw::cartography;
using namespace vw::platefile;
using namespace asp::pho;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // Input
  std::string ptk_url, drg_file, cam_file;

  // Output
  std::string output_dir;
};

void do_creation( Options const& opt ) {
  // Load up camera information
  RemoteProjectFile remote_ptk( opt.ptk_url );

  // Load up DRG
  DiskImageView<PixelGrayA<uint8> > drg_image( opt.drg_file );
  GeoReference georef;
  read_georeference( georef, opt.drg_file );

  // Request creation of new camera meta
  CameraMeta cam_meta;
  cam_meta.set_name(opt.drg_file);
  cam_meta.set_exposure_t( 1.0 );
  int32 cam_id = remote_ptk.CreateCameraMeta( cam_meta );
  std::cout << "Assigned Camera ID: " << cam_id << "\n";

  { // Insert DRG
    boost::shared_ptr<PlateFile> drg_plate =
      boost::shared_ptr<PlateFile>( new PlateFile( "pf://index/DRG.plate",
                                                   "equi", "", 256, "tif",
                                                   VW_PIXEL_GRAYA,
                                                   VW_CHANNEL_UINT8 ) );
    PlateCarreePlateManager< PixelGrayA<uint8> > drg_manager( drg_plate );
    drg_manager.insert( drg_image, opt.drg_file,
                        cam_id+1, georef, false, false,
                        TerminalProgressCallback( "photometrytk",
                                                  "\tProcessing" ) );
  }

  /*
  { // Create Reflectance ( at the moment it's just white )
    boost::shared_ptr<PlateFile> ref_plate =
      boost::shared_ptr<PlateFile>( new PlateFile( "pf://index/Reflectance.plate",
                                                   "equi", "", 256, "tif",
                                                   VW_PIXEL_GRAYA,
                                                   VW_CHANNEL_FLOAT32 ) );
    PlateCarreePlateManager< PixelGrayA<float32> > ref_manager( ref_plate );
    ref_manager.insert( mask_to_alpha(copy_mask(ConstantView<PixelGray<float32> >(1.0, drg_image.cols(), drg_image.rows() ), alpha_to_mask(drg_image))),
                        opt.drg_file, cam_id+1, georef, false,
                        TerminalProgressCallback( "photometrytk",
                                                  "\tProcessing" ) );
  }
  */

}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-dir,o", po::value<std::string>(&opt.output_dir)->default_value(""))
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("ptk_url",  po::value<std::string>(&opt.ptk_url),  "Input PTK Url")
    ("drg_file", po::value<std::string>(&opt.drg_file), "Input DRG file")
    ("cam_file", po::value<std::string>(&opt.cam_file), "Input Camera file");

  po::positional_options_description positional_desc;
  positional_desc.add("ptk_url", 1);
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
  usage << "Usage: " << argv[0] << " <ptk-url> <drg-file> <cam-file>\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.drg_file.empty() || opt.ptk_url.empty() )
    vw_throw( ArgumentErr() << "Missing input DRG or URL!\n"
              << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    do_creation( opt );
  } ASP_STANDARD_CATCHES;

  return 0;
}
