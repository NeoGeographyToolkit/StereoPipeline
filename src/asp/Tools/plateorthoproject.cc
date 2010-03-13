// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/Image.h>
#include <vw/Camera.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/TileManipulation.h>
#include <vw/Plate/PlateCarreePlateManager.h>
#include <vw/Plate/ToastPlateManager.h>
using namespace vw;
using namespace vw::platefile;

#include <asp/Sessions.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/convenience.hpp>
namespace fs = boost::filesystem;

using namespace std;

// --- Standard Terminal Args ---

struct Options {
  // Input
  string input_url;
  string camera_image; // For ISIS, these are the same
  string camera_model;
  int dem_id;
  int level;

  // Settings
  string datum;
  string session;
  string output_mode;
  int output_scale;

  // Output
  string output_url;
  int output_id; // transaction id

  Options() : output_id(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("Orthoprojects imagery into a plate file");
  general_options.add_options()
    ("dem-id", po::value<int>(&opt.dem_id)->default_value(1), "DEM's transaction ID in input platefile")
    ("level,l", po::value<int>(&opt.level)->default_value(-1), "Level at which to find input DEM.")
    ("output-id", po::value<int>(&opt.output_id), "Output transaction ID to use. If not provided, one will be solved for.")
    ("output-scale", po::value<int>(&opt.output_scale)->default_value(1), "Output scale, a 2 will double the out resolution of the orthoprojection.")
    ("mode,m", po::value<string>(&opt.output_mode)->default_value("equi"), "Output mode [equi]")
    ("datum,d", po::value<string>(&opt.datum)->default_value("earth"), "Datum to use [earth, moon, mars].")
    ("session-type,t", po::value<string>(&opt.session), "Select the stereo session type to use for processing. [Options are pinhole, isis, ...]")
    ("help", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-url", po::value<string>(&opt.input_url), "")
    ("camera-image", po::value<string>(&opt.camera_image), "")
    ("camera-model", po::value<string>(&opt.camera_model), "")
    ("output-url", po::value<string>(&opt.output_url), "");

  po::options_description options("");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-url",1);
  p.add("camera-image",1);
  p.add("camera-model",1);
  p.add("output-url",1);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <input-plate> <camera-image> <camera-model> <output-plate> [options]\n";

  if ( vm.count("help") || opt.input_url.empty() ||
       opt.camera_image.empty() || opt.camera_model.empty() )
    vw_throw( ArgumentErr() << usage.str() << options );

  // Detecting session type
  boost::to_lower(opt.session);
  if ( opt.session.empty() ) {
    if ( boost::iends_with(opt.camera_model,".cahvor") ||
         boost::iends_with(opt.camera_model,".cmod") ||
         boost::iends_with(opt.camera_model,".cahv") ||
         boost::iends_with(opt.camera_model,".pin") ||
         boost::iends_with(opt.camera_model,".tsai") ) {
      vw_out() << "\t--> Detected pinhole camera files.  Executing pinhole stereo pipeline.\n";
      opt.session = "pinhole";
    }

    else if ( boost::iends_with(opt.camera_image, ".cub") ) {
      vw_out() << "\t--> Detected ISIS cube files.  Executing ISIS stereo pipeline.\n";
      opt.session = "isis";
    }

    else {
      vw_throw( ArgumentErr() << "\n\n******************************************************************\n"
                << "Could not determine stereo session type.   Please set it explicitly\n"
                << "using the -t switch.  Options include: [pinhole isis].\n"
                << "******************************************************************\n\n" );
    }
  }

  // Handling ISIS's model within image
  if ( opt.output_url.empty() ) {
    if ( opt.session == "isis" ) {
      opt.output_url = opt.camera_model;
      opt.camera_model = "";
    } else {
      vw_throw( ArgumentErr() << usage.str() << options );
    }
  }

  // Final clean up and saving user from themselves
  boost::to_lower(opt.datum);
  if ( opt.output_scale < 1 )
    vw_throw( ArgumentErr() << "Output scaling can't be less than one.\n" );
}

// --- Internal Operations ---

template <class PixelT>
void do_projection(boost::shared_ptr<PlateFile> input_plate,
                   boost::shared_ptr<PlateFile> output_plate,
                   boost::shared_ptr<camera::CameraModel> camera_model,
                   Options& opt) {

  PlateCarreePlateManager<PixelGrayA<float > > input_pm(input_plate);
  cartography::GeoReference input_georef = input_pm.georeference(opt.level);

  // Finding and setting our Datum
  if ( opt.datum == "earth" )
    input_georef.set_well_known_geogcs("WGS84");
  else if ( opt.datum == "mars" )
    input_georef.set_well_known_geogcs("D_MARS");
  else if ( opt.datum == "moon" )
    input_georef.set_well_known_geogcs("D_MOON");
  else
    vw_throw( ArgumentErr() << "Unknown datum, \"" << opt.datum << "\".\n" );

  unsigned region_size = pow(2.0,opt.level);

  // Loading up input image
  DiskImageView<PixelT> texture_disk_image(opt.camera_image);
  ImageViewRef<PixelT> texture_image = texture_disk_image;

  // Camera Center's
  Vector3 camera_center = camera_model->camera_center(Vector2());
  Vector3 camera_lla = cartography::xyz_to_lon_lat_radius( camera_center );
  std::cout << "Camera's location = " << camera_lla << "\n";

  // Normalize between 0->1 since ISIS is crazy
  if ( opt.session == "isis" ) {
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    channel_type lo, hi;
    min_max_channel_values(texture_disk_image, lo, hi);
    std::cout << " Low: " << lo << " Hi: " << hi << "\n";
    texture_image = normalize(remove_isis_special_pixels(texture_disk_image),lo,hi,ChannelRange<channel_type>::min(),ChannelRange<channel_type>::max());
    std::cout << " CMin: " << ChannelRange<channel_type>::min() << " CMax: " << ChannelRange<channel_type>::max() << "\n";
  }

  // Performing rough approximation of which tiles this camera touches
  BBox2 image_alt = camera_bbox( input_georef, camera_model,
                                 texture_image.cols(), texture_image.rows() );
  Vector2 start_tile = input_georef.lonlat_to_pixel(image_alt.min())/float(input_plate->default_tile_size());
  Vector2 end_tile = input_georef.lonlat_to_pixel(image_alt.max())/float(input_plate->default_tile_size());
  Vector2i start_tile_i;
  start_tile_i[0] = floor(start_tile[0]);
  start_tile_i[1] = floor(start_tile[1]);
  Vector2i end_tile_i;
  end_tile_i[0] = floor(end_tile[0]);
  end_tile_i[1] = floor(end_tile[1]);
  if ( end_tile_i[0] < start_tile_i[0] )
    std::swap(end_tile_i[0],start_tile_i[0]);
  if ( end_tile_i[1] < start_tile_i[1] )
    std::swap(end_tile_i[1],start_tile_i[1]);
  end_tile_i += Vector2i(1,1);

  // Attempting to project over platefile
  for ( int32 i = start_tile_i[0]; i < end_tile_i[0]; i++ ) {
    for ( int32 j = start_tile_i[1]; j < end_tile_i[1]; j++ ) {

      // Polling for DEM tile
      ImageView<PixelGrayA<float32> > dem_tile;
      try {
        TileHeader th = input_plate->read(dem_tile,i,j,opt.level,
                                          opt.dem_id, true);
      } catch (TileNotFoundErr &e) {
        // Not found, continue on
        continue;
      }

      // Creating this tile's georeference
      cartography::GeoReference dem_georef = input_georef;
      Vector2 top_left_ll = input_georef.pixel_to_lonlat(Vector2(i,j)*input_plate->default_tile_size());
      Matrix3x3 T = dem_georef.transform();
      T(0,2) = top_left_ll[0];
      T(1,2) = top_left_ll[1];
      dem_georef.set_transform(T);

      std::cout << "\t--> Orthoprojecting onto tile (" << i << "," << j << ")\n"
                << "\t    with transform " << dem_georef << "\n";

      cartography::GeoReference drg_georef = dem_georef;
      Matrix3x3 drg_trans = drg_georef.transform();
      drg_trans(0,0) /= opt.output_scale;
      drg_trans(1,1) /= opt.output_scale;
      drg_georef.set_transform(drg_trans);

      cartography::GeoTransform trans(dem_georef, drg_georef);
      ImageViewRef<PixelGrayA<float32> > output_dem = transform(dem_tile, trans, ZeroEdgeExtension(), BicubicInterpolation());

      ImageView<PixelT> projection = orthoproject(output_dem, drg_georef,
                                                  texture_image, camera_model,
                                                  BicubicInterpolation(), ZeroEdgeExtension());

      // Push to output plate file
      if ( opt.output_mode == "equi" ) {
        boost::shared_ptr<PlateCarreePlateManager<PixelT> > pm(
          new PlateCarreePlateManager<PixelT> (output_plate) );

        pm->insert(projection, opt.camera_image,
                   opt.output_id, drg_georef, false,
                   TerminalProgressCallback( "asp.plateorthoproject", "\t    Processing") );
      } else if ( opt.output_mode == "toast" ) {
        boost::shared_ptr<ToastPlateManager<PixelT> > pm(
          new ToastPlateManager<PixelT> (output_plate) );

        pm->insert(projection, opt.camera_image,
                   opt.output_id, drg_georef, false,
                   TerminalProgressCallback( "asp.plateorthoproject", "\t    Processing") );
      }

    } // for j loop
  }   // for i loop
}

void do_run( Options& opt ) {

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  // Extracting camera model from stereo session
  StereoSession* session = StereoSession::create(opt.session);
  session->initialize(opt.camera_image, opt.camera_image,
                      opt.camera_model, opt.camera_model,
                      "", "", "", "", "" );
  boost::shared_ptr<camera::CameraModel> camera_model;
  camera_model = session->camera_model(opt.camera_image,
                                       opt.camera_model);

  // Opening input and output plate files
  if ( opt.output_url.empty() )
    opt.output_url = fs::change_extension( opt.camera_image, "_DRG.plate" ).string();
  PixelFormatEnum pixel_format;
  ChannelTypeEnum channel_type;
  try {
    DiskImageResource *rsrc = DiskImageResource::open(opt.camera_image);
    pixel_format = rsrc->pixel_format();
    channel_type = rsrc->channel_type();

    // Plate files should always have an alpha channel.
    if (pixel_format == VW_PIXEL_GRAY)
      pixel_format = VW_PIXEL_GRAYA;
    if (pixel_format == VW_PIXEL_RGB)
      pixel_format = VW_PIXEL_RGBA;
    delete rsrc;
  } catch (vw::Exception &e) {
    vw_out(ErrorMessage) << "An error occured: " << e.what() << "\n";
    exit(1);
  }
  string tile_filetype;
  if (channel_type == VW_CHANNEL_FLOAT32)
    tile_filetype = "tif";
  else
    tile_filetype = "png";
  boost::shared_ptr<PlateFile> input_plate =
    boost::shared_ptr<PlateFile>( new PlateFile(opt.input_url) );
  boost::shared_ptr<PlateFile> output_plate =
    boost::shared_ptr<PlateFile>( new PlateFile(opt.output_url,
                                                opt.output_mode, "",
                                                256, tile_filetype,
                                                pixel_format, channel_type) );
  switch( pixel_format ) {
  case VW_PIXEL_GRAYA:
    switch( channel_type ) {
    case VW_CHANNEL_UINT8:
    case VW_CHANNEL_UINT16:
      do_projection<PixelGrayA<uint8> >( input_plate, output_plate,
                                         camera_model, opt );
      break;
    case VW_CHANNEL_INT16:
      do_projection<PixelGrayA<int16> >( input_plate, output_plate,
                                         camera_model, opt );
      break;
    case VW_CHANNEL_FLOAT32:
      do_projection<PixelGrayA<float32> >( input_plate, output_plate,
                                           camera_model, opt );
      break;
    default:
      vw_throw(InputErr() << "Unsupported channel type.\n");
    }
    break;
  case VW_PIXEL_RGBA:
    switch( channel_type ) {
    case VW_CHANNEL_UINT8:
      do_projection<PixelRGBA<uint8> >( input_plate, output_plate,
                                        camera_model, opt );
      break;
    default:
      vw_throw(InputErr() << "Unsupported channel type.\n");
    }
  default:
    vw_throw(InputErr() << "Unsupported pixel type.\n");
    break;
  }
}

// --- Interface to Terminal ----

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Farm out task from here
    do_run( opt );

  } catch ( const ArgumentErr& e ) {
    vw_out() << e.what() << std::endl;
    return 1;
  } catch ( const Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
