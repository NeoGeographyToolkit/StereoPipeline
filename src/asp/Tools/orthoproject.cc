// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file othoproject.cc
///
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/Functors.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

#include <asp/Core/Macros.h>
#include <asp/Sessions.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <boost/tokenizer.hpp>

template <class ViewT, class DemViewT>
GeoReference compute_geotransform_from_camera(ImageViewBase<ViewT> const& view,
                                              boost::shared_ptr<CameraModel> camera,
                                              ImageViewBase<DemViewT> const& dem,
                                              GeoReference const& dem_georef,
                                              double scale,
                                              double &output_width, double& output_height) {

  // Adopt the datum & projection of the parent DEM.
  GeoReference geo = dem_georef;

  // Compute the longitude/latitude bounding box and the lonlat pixel scale
  float lonlat_scale;
  BBox2 image_bbox = camera_bbox(dem_georef, camera, view.impl().cols(), view.impl().rows(), lonlat_scale);
  BBox2 dem_bbox = dem_georef.lonlat_bounding_box(dem);

  // Use a bbox where both the image and the DEM have valid data.
  // Throw an error if there turns out to be no overlap.
  BBox2 bbox = image_bbox;
  bbox.crop(dem_bbox);

  if (bbox.width() == 0 || bbox.height() == 0) {
    vw_out() << "Image bounding box (" << image_bbox << ") and DEM bounding box (" << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
    return 0;
  }

  double diff1 = bbox.max().x()-bbox.min().x();

  // Convert the lon/lat bounding box into the map projection.
  bbox.min() = geo.lonlat_to_point(bbox.min());
  bbox.max() = geo.lonlat_to_point(bbox.max());
  double diff2 = bbox.max().x()-bbox.min().x();

  // Convert the scale into the projected space
  double projected_space_scale = lonlat_scale * diff2 / diff1;

  // If the user has supplied a scale, we override our computed value
  // here.
  if (scale != 0)
    projected_space_scale = scale;

  // Compute the affine transform for this image.
  Matrix3x3 trans = geo.transform();

  trans(0,0) = projected_space_scale;
  trans(1,1) = -projected_space_scale;
  trans(0,2) = bbox.min().x();
  trans(1,2) = bbox.max().y();
  geo.set_transform(trans);

  // Compute the width and height of the output image so that it
  // contains the entirety of the computed bounding box.

  //***** FIXME: This seems backwards to me, which probably means that
  //***** lonlat_bounding_box() is backwards, but I need to get this
  //***** to work right now so I'll investigate later.
  output_height = (bbox.max().x() - bbox.min().x()) / projected_space_scale;
  output_width = (bbox.max().y() - bbox.min().y()) / projected_space_scale;

  return geo;
}

struct Options {
  Options() : lo(0), hi(0), do_color(false) {
    nodata_value = mpp = ppd = std::numeric_limits<double>::quiet_NaN();
  }

  // Input
  std::string dem_file, image_file, camera_model_file;

  // Settings
  std::string stereo_session, output_file;
  double mpp, ppd, nodata_value;
  float lo, hi;
  bool do_color;
  PixelRGB<uint8> color;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  std::string color_text;
  po::options_description general_options("");
  general_options.add_options()
    ("mpp", po::value(&opt.mpp), "Specify the output resolution of the orthoimage in meters per pixel.")
    ("ppd", po::value(&opt.ppd), "Specify the output resolution of the orthoimage in pixels per degree.")
    ("nodata-value", po::value(&opt.nodata_value), "Specify the nodata pixel value in input DEM. Will automatically find if available.")
    ("match-dem", "Match the georeferencing parameters and dimensions of the input DEM.")
    ("min", po::value(&opt.lo), "Explicitly specify the range of the normalization (for ISIS images only)")
    ("max", po::value(&opt.hi), "Explicitly specify the range of the normalization (for ISIS images only)")
    ("session-type,t", po::value(&opt.stereo_session), "Select the stereo session type to use for processing. [default: pinhole]")
    ("use-solid-color", po::value(&color_text), "Use a solid color instead of camera image. Example: 255,0,128")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("dem", po::value(&opt.dem_file))
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_model_file))
    ("output-file", po::value(&opt.output_file));

  po::positional_options_description positional_desc;
  positional_desc.add("dem", 1);
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-file",1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n"
              << e.what() << "\n" << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <dem> <camera-image> <camera-model> <output>\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.dem_file.empty() || opt.image_file.empty() ||
       opt.camera_model_file.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n"
              << usage.str() << general_options );
  if ( !color_text.empty() ) {
    opt.do_color=true;
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(",:@");
    tokenizer tokens(color_text, sep);
    tokenizer::iterator tok_iter = tokens.begin();

    if (tok_iter == tokens.end()) vw_throw(ArgumentErr() << "Error parsing color input.");
    opt.color[0] = boost::lexical_cast<uint16>(*tok_iter);
    ++tok_iter;
    if (tok_iter == tokens.end()) vw_throw(ArgumentErr() << "Error parsing color input.");
    opt.color[1] = boost::lexical_cast<uint16>(*tok_iter);
    ++tok_iter;
    if (tok_iter == tokens.end()) vw_throw(ArgumentErr() << "Error parsing color input.");
    opt.color[2] = boost::lexical_cast<uint16>(*tok_iter);
  } else {
    opt.do_color=false;
  }
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Create a fresh stereo session and query it for the camera models.
    StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if ( opt.stereo_session.empty() ) {
      std::string ext = fs::path(opt.camera_model_file).extension();
      if ( ext == ".cahvor" || ext == ".cmod" ||
           ext == ".cahv" || ext ==  ".pin" || ext == ".tsai" ) {
        vw_out() << "\t--> Detected pinhole camera files.  Executing pinhole stereo pipeline.\n";
        opt.stereo_session = "pinhole";
      } else if ( fs::path(opt.image_file).extension() == ".cub" ) {
        vw_out() << "\t--> Detected ISIS cube files.  Executing ISIS stereo pipeline.\n";
        opt.stereo_session = "isis";
      } else {
        vw_out() << "\n\n******************************************************************\n";
        vw_out() << "Could not determine stereo session type.   Please set it explicitly\n";
        vw_out() << "using the -t switch.  Options include: [pinhole isis].\n";
        vw_out() << "******************************************************************\n\n";
        return 1;
      }
    }

    // Isis Session command handling
    if (opt.stereo_session == "isis") {
      // User didn't provide an output file, meaning camera_model was not an isis_adjust file.
      if ( opt.output_file.empty() )
        opt.output_file = opt.camera_model_file;
    } else {
      if ( opt.output_file.empty() )
        vw_throw( ArgumentErr() << "Missing output filename.\n" );
    }

    // Okay, here's a total hack.  We create a stereo session where both
    // of the imagers and images are the same, because we want to take
    // advantage of the stereo pipeline's ability to generate camera
    // models for various missions.  Hence, we create two identical
    // camera models, but only one is used.  The last four empty strings
    // are dummy arguments.
    boost::shared_ptr<StereoSession> session( StereoSession::create(opt.stereo_session) );
    session->initialize(opt.image_file, opt.image_file,
                        opt.camera_model_file, opt.camera_model_file,
                        opt.output_file, "", "","","" );
    boost::shared_ptr<camera::CameraModel> camera_model;
    camera_model = session->camera_model(opt.image_file, opt.camera_model_file);

    GeoReference dem_georef;
    ImageViewRef<PixelMask<PixelGray<float> > > dem;

    if ( fs::path(opt.dem_file).extension() == "" ) {
      // This option allows the user to just project directly unto to
      // the datum. This seems like a stop gap. This should be
      // addressed after a rewrite of orthoproject.
      //
      // Valid values are well known Datums like D_MOON D_MARS WGS84 and so on.
      Vector3 llr_camera_loc =
        cartography::XYZtoLonLatRadFunctor::apply( camera_model->camera_center(Vector2()) );
      if ( llr_camera_loc[0] < 0 ) llr_camera_loc[0] += 360;

      dem_georef =
        GeoReference(Datum(opt.dem_file),
                     Matrix3x3(1, 0, (llr_camera_loc[0] < 90 ||
                                      llr_camera_loc[0] > 270) ? -180 : 0,
                               0, -1, 90, 0, 0, 1) );
      dem = constant_view(PixelMask<PixelGray<float> >(0), 360, 180 );
      vw_out() << "\t--> Using flat datum \"" << opt.dem_file << "\" as elevation model.\n";
    } else {
      // Open the DEM
      cartography::read_georeference(dem_georef, opt.dem_file);

      DiskImageView<PixelGray<float> > dem_disk_image(opt.dem_file);
      dem = pixel_cast<PixelMask<PixelGray<float> > >(dem_disk_image);

      // Attempting to extract automatic DEM nodata value.
      if ( std::isnan(opt.nodata_value) ) {
        SrcImageResource *rsrc = DiskImageResource::open(opt.dem_file);
        if ( rsrc->has_nodata_read() )
          opt.nodata_value = rsrc->nodata_read();
        delete rsrc;
      }
      // If we have a nodata value, create a mask.
      if ( !std::isnan(opt.nodata_value) ) {
        vw_out() << "\t--> Using " << opt.nodata_value << " as the nodata value for the DEM.\n";
        dem = create_mask(dem_disk_image, opt.nodata_value);
      }
    }

    // This is a bad idea. We should switch like image2qtree.
    DiskImageView<PixelGrayA<uint8> > texture_disk_image(opt.image_file);
    ImageViewRef<PixelGrayA<uint8> > texture_image = texture_disk_image;
    DiskImageView<float > float_texture_disk_image(opt.image_file);

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    // ISIS cubes need to be normalized because their pixels are often
    // photometrically calibrated.
    if ( opt.stereo_session == "isis" ) {
      DiskImageResourceIsis isis_rsrc( opt.image_file);
      if (opt.lo == 0 && opt.hi == 0) {
        vw_out(InfoMessage) << "\t--> Computing statistics for ISIS image\n";
        int32 left_stat_scale =
          int32(ceil(sqrt(float(texture_image.cols())*
                          float(texture_image.rows()) / 1000000)));
        ChannelAccumulator<vw::math::CDFAccumulator<float> > accumulator;
        for_each_pixel(
                       subsample(create_mask( float_texture_disk_image,
                                              float(isis_rsrc.valid_minimum()),
                                              float(isis_rsrc.valid_maximum())),
                                 left_stat_scale ),
                       accumulator );
        opt.lo = accumulator.approximate_mean() -
          2*accumulator.approximate_stddev();
        opt.hi = accumulator.approximate_mean() +
          2*accumulator.approximate_stddev();
        vw_out() << "\t--> Normalizing ISIS pixel range range: ["
                 << opt.lo << "  " << opt.hi << "]\n";
      } else {
        vw_out() << "\t--> Using user-specified normalization range: ["
                 << opt.lo << "  " << opt.hi << "]\n";
      }
      texture_image =
        pixel_cast_rescale<PixelGrayA<uint8> >(normalize(remove_isis_special_pixels(float_texture_disk_image, opt.lo), opt.lo, opt.hi, 0, 1));
    }
#endif

    // Parse the scale of the output image from the command line
    // arguments.  This value can be set either in meters per pixel or
    // pixels per degree.  At the moment, we haven't done the math to
    // convert ppd to meters in a projected coordinate system and mpp to
    // degrees in an unprojected coordinate system.  We throw an error
    // in these cases for now.
    double scale = 0;
    if ( !std::isnan(opt.ppd) ) {
      if (dem_georef.is_projected()) {
        vw_out() << "Input DEM is in a map projection.  Cannot specify resolution in pixels per degree.  Use meters per pixel (-mpp) instead.";
        return 1;
      } else {
        scale = 1/opt.ppd;
      }
    } else if ( !std::isnan(opt.mpp) ) {
      if (dem_georef.is_projected()) {
        scale = opt.mpp;
      } else {
        vw_out() << "Input DEM is in a simple cylindrical map projection (Plate Carree).  Cannot specify resolution in meters per pixel.  Use pixels per degree (-ppd) instead.";
        return 1;
      }
    }

    GeoReference drg_georef = dem_georef;
    // If the user has supplied a scale, we use it.  Otherwise, we
    // compute the optimal scale of image based on the camera model.
    BBox2 projection_bbox, dem_bbox;
    {
      vw_out() << "\t--> Extracting camera bbox ... " << std::flush;
      float mpp_auto_scale;
      projection_bbox =
        camera_bbox( dem, dem_georef, camera_model, texture_image.cols(),
                     texture_image.rows(), mpp_auto_scale );
      dem_bbox = dem_georef.bounding_box(dem);
      // if ( projection_bbox.min()[0] < 0 ) {
      //   projection_bbox.min() += Vector2(360,0);
      //   projection_bbox.max() += Vector2(360,0);
      //   if ( projection_bbox.max()[0] > 360 ) {
      //     projection_bbox.min()[0] = 0;
      //     projection_bbox.max()[0] = 360;
      //   }
      // }
      vw_out() << "done\n";

      // Use a bbox where both the image and the DEM have valid data.
      // Throw an error if there turns out to be no overlap.
      projection_bbox.crop(dem_bbox);

      if ( projection_bbox.empty() ) {
        vw_out() << "Image bounding box (" << projection_bbox << ") and DEM bounding box ("
                 << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
        return 1;
      }

      if ( scale == 0 )
        scale = mpp_auto_scale;
    }

    double output_width = projection_bbox.width() / scale;
    double output_height = projection_bbox.height() / scale;

    Matrix3x3 drg_trans = drg_georef.transform();
    // This weird polarity checking is to make sure the output has been
    // transposed after going through reprojection. Normally this is the
    // case. Yet with grid data from GMT, it is not.     -ZMM
    if ( drg_trans(0,0) < 0 )
      drg_trans(0,2) = projection_bbox.max().x();
    else
      drg_trans(0,2) = projection_bbox.min().x();
    drg_trans(0,0) = scale;
    drg_trans(1,1) = -scale;
    drg_trans(1,2) = projection_bbox.max().y();
    drg_georef.set_transform(drg_trans);

    GeoTransform trans(dem_georef, drg_georef);
    ImageViewRef<PixelMask<PixelGray<float> > > output_dem =
      crop(transform(dem, trans,
                     ZeroEdgeExtension(),
                     BicubicInterpolation()),
           BBox2i(0,0,int32(output_width),int32(output_height)));

    if ( opt.do_color ) {
      vw_out() << "\t--> Orthoprojecting solid color image.\n";
      ImageViewRef<PixelGrayA<uint8> > final_result =
        orthoproject(output_dem, drg_georef,
                     constant_view(PixelGrayA<uint8>(255,255),texture_image.cols(),texture_image.rows()),
                     camera_model,
                     BicubicInterpolation(), ZeroEdgeExtension());
      DiskImageResourceGDAL rsrc(opt.output_file, final_result.format(), Vector2i(vw_settings().default_tile_size(),vw_settings().default_tile_size()) );
      write_georeference(rsrc, drg_georef);
      write_image(rsrc, final_result, TerminalProgressCallback("asp",""));
    } else {
      vw_out() << "\t--> Orthoprojecting texture.\n";
      ImageViewRef<PixelGrayA<uint8> > final_result =
        orthoproject(output_dem, drg_georef,
                     texture_image, camera_model,
                     BicubicInterpolation(), ZeroEdgeExtension());

      DiskImageResourceGDAL rsrc(opt.output_file, final_result.format(), Vector2i(vw_settings().default_tile_size(),vw_settings().default_tile_size()) );
      write_georeference(rsrc, drg_georef);
      write_image(rsrc, final_result, TerminalProgressCallback("asp",""));
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}


