// Wholesale copy from orthoproject.cc. Longer term, this needs to be dealt with properly.
// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file reconstruct_aux.cc
///
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Math/Functors.h>
#include <vw/Photometry.h>
using namespace std;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::math;
using namespace vw::photometry;
using namespace vw;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <boost/tokenizer.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
struct Options : asp::BaseOptions {
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
    ("use-solid-color", po::value(&color_text), "Use a solid color instead of camera image. Example: 255,0,128");
  general_options.add( asp::BaseOptionsDescription(opt) );

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

  std::string usage("[options] <dem> <camera-image> <camera-model> <output>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage );

  if ( opt.dem_file.empty() || opt.image_file.empty() ||
       opt.camera_model_file.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n"
              << usage << general_options );
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

template <class PixelT>
void do_projection( Options& opt,
                    ImageViewRef<PixelMask<float> > const& dem,
                    GeoReference const& drg_georef,
                    boost::shared_ptr<camera::CameraModel> camera_model ) {

  DiskImageView<PixelT > texture_disk_image(opt.image_file);
  ImageViewRef<PixelT > texture_image = texture_disk_image;
  DiskImageView<float > float_texture_disk_image(opt.image_file);

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // ISIS cubes need to be normalized because their pixels are often
  // photometrically calibrated.
  if ( opt.stereo_session == "isis" && !opt.do_color ) {
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
      pixel_cast_rescale<PixelT >(normalize(asp::remove_isis_special_pixels(float_texture_disk_image, opt.lo), opt.lo, opt.hi, 0, 1));
  }
#endif


  vw_out() << "\t--> Orthoprojecting texture.\n";
  ImageViewRef<PixelT > final_result =
    orthoproject_markNoProcessedData(dem, drg_georef,
                                     texture_image, camera_model,
                                     BicubicInterpolation(), ZeroEdgeExtension());

  // Note that we convert the output from uint16 to uint8. This is a bit of a hack.
  std::cout << "Writing to " << opt.output_file << std::endl;
  asp::write_gdal_georeferenced_image(opt.output_file,
                                      pixel_cast_rescale< PixelGrayA<uint8> >(clamp(final_result, 0, 32767)),
                                      drg_georef, opt, TerminalProgressCallback("asp","")
                                      );
}

int extractDRGFromCube(bool useDEMTiles, double cubeToDrgScaleFactor, std::string DEMTilesDir, std::string DEMFile,
                      std::string cubeFile, std::string isis_adjust_file, std::string outputDrgFile,
                      Vector3 & sunPosition, Vector3 & spacecraftPosition
                      ){

  // Orthorproject a cube onto a given DEM. If the parameter
  // useDEMTiles is true, use for the DEM the union of all DEM tiles
  // which intersect the cube.

  Options opt;
  try {
    //handle_arguments( argc, argv, opt );

    if ( !fs::exists(isis_adjust_file) ){
      std::cout << "WARNING: The ISIS adjust file " << isis_adjust_file
                << " is missing, will extract the unadjusted spacecraft position from the cube file "
                << cubeFile<< std::endl;
      isis_adjust_file = cubeFile;
    }

    opt.dem_file           = DEMFile;
    opt.image_file         = cubeFile;
    opt.camera_model_file  = isis_adjust_file;
    opt.output_file        = outputDrgFile;
    
    // Create a fresh stereo session and query it for the camera models.
    asp::StereoSession::register_session_type( "rmax", &asp::StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    asp::StereoSession::register_session_type( "isis", &asp::StereoSessionIsis::construct);
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
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSession::create(opt.stereo_session) );
    session->initialize(opt, opt.image_file, opt.image_file,
                        opt.camera_model_file, opt.camera_model_file,
                        opt.output_file, "","","","" );
    boost::shared_ptr<camera::CameraModel> camera_model;
    camera_model = session->camera_model(opt.image_file, opt.camera_model_file);
    
    if (opt.stereo_session == "isis") {
      vw::camera::IsisCameraModel M(opt.image_file);
      sunPosition = M.sun_position();
    }

    spacecraftPosition = camera_model->camera_center(Vector2());
    
    GeoReference dem_georef;
    ImageViewRef<PixelMask<float > > dem;

    if ( fs::path(opt.dem_file).extension() == "" && (!useDEMTiles) ) {
      std::cerr << "Must never come here, in " << __FILE__ << std::endl;
      exit(1);
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
      dem = constant_view(PixelMask<float>(0), 360, 180 );
      vw_out() << "\t--> Using flat datum \"" << opt.dem_file << "\" as elevation model.\n";
    } else {

      // If we use DEM tiles, need to find the bounding box of the
      // cube, and load all the dems intersecting that bounding
      // box. Else, just load the DEM specified in opt.dem_file.
      if (useDEMTiles){
        
        std::vector<std::string> tifsInDir;
        listTifsInDir(DEMTilesDir, tifsInDir);
        if (tifsInDir.size() == 0){
          cerr << "No DEM tiles are available" << endl;
          exit(1);
        }
        GeoReference someGeoRef;
        std::string someDEM = tifsInDir[0];
        cartography::read_georeference(someGeoRef, someDEM);
        BBox2 projection_bbox;
        SrcImageResource *texture_rsrc = DiskImageResource::open(opt.image_file);
        float tmp_scale;
        BBox2 cubeBox = camera_bbox(someGeoRef, camera_model, texture_rsrc->cols(),
                     texture_rsrc->rows(), tmp_scale );
        delete texture_rsrc;
        
        // Attempting to extract automatic DEM nodata value.
        if ( std::isnan(opt.nodata_value) ) {
          SrcImageResource *rsrc = DiskImageResource::open(someDEM);
          if ( rsrc->has_nodata_read() )
            opt.nodata_value = rsrc->nodata_read();
          delete rsrc;
        }

        Vector2 boxNW = Vector2(cubeBox.min().x(), cubeBox.max().y());
        Vector2 boxSE = Vector2(cubeBox.max().x(), cubeBox.min().y());

        ImageView<PixelGray<float> > dem_disk_image;
        vw::photometry::readDEMTilesIntersectingBox(opt.nodata_value, boxNW, boxSE, tifsInDir, // inputs
                                                    dem_disk_image, dem_georef                 // outputs
                                                    );
        
        // If we have a nodata value, create a mask.
        if ( !std::isnan(opt.nodata_value) ) {
          vw_out() << "\t--> Using " << opt.nodata_value << " as the nodata value for the DEM.\n";
          dem = create_mask(pixel_cast<float>(dem_disk_image), opt.nodata_value);
        }

      }else{
        // Open the DEM
        cartography::read_georeference(dem_georef, opt.dem_file);
        
        DiskImageView<float> dem_disk_image(opt.dem_file);
        dem = pixel_cast<PixelMask<float> >(dem_disk_image);

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
    }
    
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
      SrcImageResource *texture_rsrc =
        DiskImageResource::open(opt.image_file);
      float mpp_auto_scale;
      projection_bbox =
        camera_bbox( dem, dem_georef, camera_model, texture_rsrc->cols(),
                     texture_rsrc->rows(), mpp_auto_scale );
      delete texture_rsrc;

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

      if ( scale == 0 ){
        scale = mpp_auto_scale;
        // We got the scale from the cube, rather from the user. Then, the user may want to adjust
        // the scale by given factor (factor > 1 will result in lower-res output image).
        scale *= cubeToDrgScaleFactor; 
      }
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
    ImageViewRef<PixelMask<float> > output_dem =
      crop(transform(dem, trans,
                     ZeroEdgeExtension(),
                     BicubicInterpolation()),
           BBox2i(0,0,int32(output_width),int32(output_height)));

    SrcImageResource *texture_rsrc =
      DiskImageResource::open(opt.image_file);
    ImageFormat fmt = texture_rsrc->format();
    delete texture_rsrc;

    if ( opt.do_color ) {
      vw_out() << "\t--> Orthoprojecting solid color image.\n";
      ImageViewRef<PixelRGB<uint8> > final_result =
        orthoproject_markNoProcessedData(output_dem, drg_georef,
                                         constant_view(PixelRGB<uint8>(opt.color[0],opt.color[1],opt.color[2]),
                                                       fmt.cols,fmt.rows),
                                         camera_model,
                                         BicubicInterpolation(), ZeroEdgeExtension());
      DiskImageResourceGDAL rsrc(opt.output_file, final_result.format() );
      write_georeference(rsrc, drg_georef);
      write_image(rsrc, final_result, TerminalProgressCallback("asp",""));
    } else {
      switch(fmt.pixel_format) {
      case VW_PIXEL_GRAY:
      case VW_PIXEL_GRAYA:
      default:
        switch(fmt.channel_type) {
        case VW_CHANNEL_UINT8: do_projection<PixelGrayA<uint8> >( opt, output_dem,
                                                                  drg_georef,
                                                                  camera_model ); break;
        case VW_CHANNEL_INT16: do_projection<PixelGrayA<int16> >( opt, output_dem,
                                                                  drg_georef,
                                                                  camera_model ); break;
        case VW_CHANNEL_UINT16: do_projection<PixelGrayA<uint16> >( opt, output_dem,
                                                                    drg_georef,
                                                                    camera_model ); break;
        default: do_projection<PixelGrayA<float32> >( opt, output_dem,
                                                      drg_georef,
                                                      camera_model ); break;
        }
        break;
      case VW_PIXEL_RGB:
      case VW_PIXEL_RGBA:
        switch(fmt.channel_type) {
        case VW_CHANNEL_UINT8: do_projection<PixelRGBA<uint8> >( opt, output_dem,
                                                                 drg_georef,
                                                                 camera_model ); break;
        case VW_CHANNEL_INT16: do_projection<PixelRGBA<int16> >( opt, output_dem,
                                                                 drg_georef,
                                                                 camera_model ); break;
        case VW_CHANNEL_UINT16: do_projection<PixelRGBA<uint16> >( opt, output_dem,
                                                                   drg_georef,
                                                                   camera_model ); break;
        default: do_projection<PixelRGBA<float32> >( opt, output_dem,
                                                     drg_georef,
                                                     camera_model ); break;
        }
        break;
      }
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}


