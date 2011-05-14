// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
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
#include <vw/Plate/PlateManager.h>
#include <asp/PhotometryTK/RemoteProjectFile.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

using namespace vw;
using namespace vw::cartography;
using namespace vw::platefile;
using namespace asp::pho;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options : asp::BaseOptions {
  Options() : nodata_value(std::numeric_limits<double>::max()) {}
  // Input
  Url ptk_url;
  std::string drg_file, cam_file;

  // Settings
  double nodata_value;

  // Output
  std::string output_dir;
};

void do_creation( Options& opt ) {
  // Load up camera information
  RemoteProjectFile remote_ptk( opt.ptk_url );
  ProjectMeta prj_meta;
  remote_ptk.get_project( prj_meta );

  // Load up GeoReference
  GeoReference georef;
  read_georeference( georef, opt.drg_file );

  // Request creation of new camera meta
  CameraMeta cam_meta;
  cam_meta.set_name(opt.drg_file);
  cam_meta.set_exposure_t( 1.0 );
  cam_meta.set_init_error( 0.0 );
  cam_meta.set_last_error( 0.0 );
  cam_meta.set_curr_error( 0.0 );
  int32 cam_id = remote_ptk.add_camera( cam_meta );

  // Find relative urls for platefiles
  boost::shared_ptr<PlateFile> drg, albedo, reflectance;
  remote_ptk.get_platefiles(drg,albedo,reflectance);

  // Determine if we have a nodata value
  if ( opt.nodata_value == std::numeric_limits<double>::max() ) {
    boost::shared_ptr<DiskImageResource> rsrc( DiskImageResource::open(opt.drg_file) );
    if ( rsrc->has_nodata_read() )
      opt.nodata_value = rsrc->nodata_read();
  }

  {
    PlateManager<PixelGrayA<float32> >* pm =
      PlateManager<PixelGrayA<float32> >::make(prj_meta.plate_manager(),drg);
    // Insert DRG
    if ( opt.nodata_value != std::numeric_limits<double>::max() ) {
      pm->insert( mask_to_alpha(create_mask(DiskImageView<PixelGray<float32> >(opt.drg_file),opt.nodata_value)),
                  opt.drg_file, cam_id+1, georef, false, false,
                  TerminalProgressCallback( "photometrytk", "\tProcessing" ) );
    } else {
      pm->insert( DiskImageView<PixelGrayA<float32> >( opt.drg_file ),
                  opt.drg_file, cam_id+1, georef, false, false,
                  TerminalProgressCallback( "photometrytk", "\tProcessing" ) );
    }
    delete pm;
  }

  /*
  { // Create Reflectance ( at the moment it's just white )
    PlateCarreePlateManager< PixelGrayA<float32> > ref_manager( reflectance );
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
    ("output-dir,o", po::value(&opt.output_dir)->default_value(""))
    ("nodata-value", po::value(&opt.nodata_value), "Explicitly set the value to treat as no data in the input file.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("ptk_url",  po::value(&opt.ptk_url),  "Input PTK Url")
    ("drg_file", po::value(&opt.drg_file), "Input DRG file")
    ("cam_file", po::value(&opt.cam_file), "Input Camera file");

  po::positional_options_description positional_desc;
  positional_desc.add("ptk_url", 1);
  positional_desc.add("drg_file", 1);
  positional_desc.add("cam_file", 1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <ptk-url> <drg-file> <cam-file>\n";

  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage.str() );

  if ( opt.drg_file.empty() || opt.ptk_url == Url() )
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
