// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#include <vw/Image.h>
#include <vw/Camera.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/TileManipulation.h>
#include <vw/Plate/PlateCarreePlateManager.h>
#include <vw/Plate/ToastPlateManager.h>
#include <vw/Plate/PlateView.h>
using namespace vw;
using namespace vw::platefile;

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/convenience.hpp>
namespace fs = boost::filesystem;

using namespace std;

// --- Standard Terminal Args ---
struct Options : public asp::BaseOptions {
  Options() : output_id(-1) {}

  // Input
  string input_url;
  string camera_image, camera_model; // For ISIS, these are the same
  int32 dem_id, level;

  // Settings
  string datum, session, output_mode, mask_image;

  // Output
  string output_url;
  int output_id; // transaction id
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("Orthoprojects imagery into a plate file");
  general_options.add_options()
    ("dem-id", po::value(&opt.dem_id)->default_value(-1), "DEM's transaction ID in input platefile")
    ("level,l", po::value(&opt.level)->default_value(-1), "Level at which to find input DEM.")
    ("output-id", po::value(&opt.output_id),
     "Output transaction ID to use. If not provided, one will be solved for.")
    ("mode,m", po::value(&opt.output_mode)->default_value("equi"), "Output mode [toast, equi, polar]")
    ("mask", po::value(&opt.mask_image), "Input single channel image with same aspect ratio as camera image whose pixel with maximum value define valid pixels from projection.")
    ("datum,d", po::value(&opt.datum)->default_value("Earth"), "Datum to use [Earth, Moon, Mars].")
    ("session-type,t", po::value(&opt.session),
     "Select the stereo session type to use for processing. [Options are pinhole, isis, ...]");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-url", po::value(&opt.input_url), "")
    ("camera-image", po::value(&opt.camera_image), "")
    ("camera-model", po::value(&opt.camera_model), "")
    ("output-url", po::value(&opt.output_url), "");

  po::positional_options_description p;
  p.add("input-url",1);
  p.add("camera-image",1);
  p.add("camera-model",1);
  p.add("output-url",1);

  std::string usage("<input-plate> <camera-image> <camera-model> <output-plate> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             hidden_options, p, usage,
                             allow_unregistered, unregistered );

  if ( opt.input_url.empty() ||
       opt.camera_image.empty() || opt.camera_model.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n\n" << usage << general_options);

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
      vw_throw( ArgumentErr() << usage << general_options );
    }
  }

  // Final clean up and saving user from themselves
  boost::to_lower(opt.datum);
}

// --- Internal Operations ---
template <class PixelT>
void do_projection(boost::shared_ptr<PlateFile> input_plate,
                   boost::shared_ptr<PlateFile> output_plate,
                   boost::shared_ptr<camera::CameraModel> camera_model,
                   Options& opt) {
  typedef PixelGrayA<float> InputT;

  boost::scoped_ptr<PlateManager<InputT> > input_pm( PlateManager<InputT >::make(opt.output_mode, input_plate ) );
  cartography::GeoReference input_georef = input_pm->georeference(opt.level);

  // Finding and setting our Datum
  if ( opt.datum == "earth" )
    input_georef.set_well_known_geogcs("WGS84");
  else if ( opt.datum == "mars" )
    input_georef.set_well_known_geogcs("D_MARS");
  else if ( opt.datum == "moon" )
    input_georef.set_well_known_geogcs("D_MOON");
  else
    vw_throw( ArgumentErr() << "Unknown datum, \"" << opt.datum << "\".\n" );

  // Loading up input image
  DiskImageView<PixelT> texture_disk_image(opt.camera_image);
  ImageViewRef<PixelT> texture_image = texture_disk_image;

  // Camera Center's
  Vector3 camera_center = camera_model->camera_center(Vector2());
  Vector3 camera_lla = cartography::xyz_to_lon_lat_radius( camera_center );
  vw_out() << "\tLoaded Camera Location: " << camera_lla << " [deg deg meters]\n";

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Normalize between 0->1.
  if ( opt.session == "isis" ) {
    DiskImageResourceIsis isis_rsrc( opt.camera_image );
    texture_image =
      normalize_retain_alpha(asp::remove_isis_special_pixels(texture_disk_image),
                             isis_rsrc.valid_minimum(),isis_rsrc.valid_maximum(),
                             ChannelRange<PixelT>::min(), ChannelRange<PixelT>::max() );
  }
#endif

  if ( !opt.mask_image.empty() ) {
    DiskImageView<uint8> mask(opt.mask_image);
    if ( bounding_box(mask) != bounding_box(texture_image ) ) {
      vw_out(WarningMessage) << "Mask image does not have same size as input. Attempting to scale to correct size.\n";
      VW_ASSERT( fabs( float(mask.cols())/float(mask.rows()) - float(texture_image.cols())/float(texture_image.rows()) ) < 1e-2,
                 ArgumentErr() << "Mask image does not have same aspect ratio as camera image. Unable to use as mask!" );
      texture_image = mask_to_alpha(copy_mask(alpha_to_mask(texture_image),
                                              create_mask(resize(DiskImageView<uint8>(mask),
                                                                 texture_image.cols(), texture_image.rows(),
                                                                 ZeroEdgeExtension(), NearestPixelInterpolation()),0)));
    } else {
      texture_image = mask_to_alpha(copy_mask(alpha_to_mask(texture_image),create_mask(DiskImageView<uint8>(mask),0)));
    }
  }

  // Performing rough approximation of which tiles this camera touches
  BBox2 image_alt = camera_bbox( input_georef, camera_model,
                                 texture_image.cols(), texture_image.rows() );

  BBox2i dem_square; // Is in pixels
  dem_square.grow( input_georef.lonlat_to_pixel(image_alt.min()) );
  dem_square.grow( input_georef.lonlat_to_pixel(image_alt.max()) );
  dem_square.max() += Vector2i(1,1);

  // Expand rough draft DEM square to the 256 boundaries
  dem_square.min() = floor(Vector2f(dem_square.min())/256.)*256;
  dem_square.max() = ceil(Vector2f(dem_square.max())/256.)*256;

  // Render a low res version of the DEM and use that to get a better approximation
  PlateView<InputT > input_view( opt.input_url );
  input_view.set_level( opt.level > 4 ? opt.level - 3 : opt.level );
  image_alt = camera_bbox(copy(crop(input_view, opt.level > 4 ? dem_square / 8 : dem_square)),
                          crop(resample(input_georef, opt.level > 4 ? 1./8. : 1),
                               opt.level > 4 ? dem_square / 8 : dem_square),
                          camera_model, texture_image.cols(), texture_image.rows());

  dem_square = BBox2i();
  dem_square.grow( input_georef.lonlat_to_pixel(image_alt.min()) );
  dem_square.grow( input_georef.lonlat_to_pixel(image_alt.max()) );
  dem_square.max() += Vector2i(1,1);

  // Expand rough draft DEM square to the 256 boundaries
  dem_square.min() = floor(Vector2f(dem_square.min())/256.)*256;
  dem_square.max() = ceil(Vector2f(dem_square.max())/256.)*256;

  // Building plateview to extract a single DEM
  input_view.set_level( opt.level );
  ImageViewRef<InputT > input_view_ref =
    crop(edge_extend(input_view, PeriodicEdgeExtension()),dem_square);

  cartography::GeoReference dem_georef = crop(input_georef, dem_square);

  vw_out() << "\t--> Orthoprojecting into DEM square with transform:\n"
           << "\t    " << dem_georef << "\n";
  ImageViewRef<PixelT> projection =
    orthoproject(input_view_ref, dem_georef,
                 texture_image, camera_model,
                 BicubicInterpolation(),
                 ZeroEdgeExtension());

  // Push to output plate file
  boost::scoped_ptr<PlateManager<PixelT> > output_pm( PlateManager<PixelT>::make( opt.output_mode, output_plate ) );
  output_pm->insert(projection, opt.camera_image,
                    opt.output_id, dem_georef, false, false,
                    TerminalProgressCallback( "asp.plateorthoproject",
                                              "\t   Processing") );
}

void do_run( Options& opt ) {

  // Extracting camera model from stereo session
  typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
  SessionPtr session( asp::StereoSession::create(opt.session) );
  session->initialize(opt, opt.camera_image, opt.camera_image,
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
  } catch (vw::Exception const& e) {
    vw_throw( ArgumentErr() << "An error occured while finding pixel type: " << e.what() << "\n" );
  }

  string tile_filetype;
  if (channel_type == VW_CHANNEL_FLOAT32 ||
      channel_type == VW_CHANNEL_INT16 )
    tile_filetype = "tif";
  else
    tile_filetype = "png";

  typedef boost::shared_ptr<PlateFile> PlatePtr;
  PlatePtr input_plate( new PlateFile(opt.input_url) );
  PlatePtr output_plate( new PlateFile(opt.output_url,
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

  } ASP_STANDARD_CATCHES;

  return 0;
}
