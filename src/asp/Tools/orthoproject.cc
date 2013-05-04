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


/// \file othoproject.cc
///
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
#include <asp/Core/Common.h>
#include <asp/Sessions.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <boost/tokenizer.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : asp::BaseOptions {
  Options() : lo(0), hi(0), do_color(false), mark_no_processed_data(false) {
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
  bool mark_no_processed_data;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  std::string color_text;
  po::options_description general_options("");
  general_options.add_options()
    ("mpp", po::value(&opt.mpp), "Specify the output resolution of the orthoimage in meters per pixel.")
    ("ppd", po::value(&opt.ppd), "Specify the output resolution of the orthoimage in pixels per degree.")
    ("nodata-value", po::value(&opt.nodata_value),
     "Specify the nodata pixel value in input DEM. Will automatically find if available.")
    ("match-dem", "Match the georeferencing parameters and dimensions of the input DEM.")
    ("min", po::value(&opt.lo), "Explicitly specify the range of the normalization (for ISIS images only)")
    ("max", po::value(&opt.hi), "Explicitly specify the range of the normalization (for ISIS images only)")
    ("session-type,t", po::value(&opt.stereo_session),
     "Select the stereo session type to use for processing. [default: pinhole]")
    ("use-solid-color", po::value(&color_text),
     "Use a solid color instead of camera image. Example: 255,0,128")
    ("mark-no-processed-data", po::bool_switch(&opt.mark_no_processed_data)->default_value(false),
     "If to set no-data pixels in the DEM which project onto the camera to black.");

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
    asp::check_command_line( argc, argv, opt, general_options, general_options,
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
  if (!opt.mark_no_processed_data){
    // Default
    asp::write_gdal_georeferenced_image(
      opt.output_file,
      orthoproject(dem, drg_georef,
                   texture_image, camera_model,
                   BicubicInterpolation(),
                   ZeroEdgeExtension()),
      drg_georef, opt,
      TerminalProgressCallback("asp","") );
  }else{
    // Save as uint8, need this for albedo
    asp::write_gdal_georeferenced_image(
      opt.output_file,
      pixel_cast_rescale< PixelGrayA<uint8> >(clamp(
        orthoproject_markNoProcessedData(dem, drg_georef,
                                         texture_image, camera_model,
                                         BicubicInterpolation(), ZeroEdgeExtension()), 0, 32767)),
      drg_georef, opt, TerminalProgressCallback("asp","") );
  }

}

int main(int argc, char* argv[]) {

  // Orthorpoject a camera image not a DEM.
  // Note: This process is not multi-threaded because it uses ISIS which is not thread safe.
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if ( opt.stereo_session.empty() ) {
      std::string ext = fs::path(opt.camera_model_file).extension().string();
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
    SessionPtr session( asp::StereoSession::create(opt.stereo_session, opt,
                                                   opt.image_file, opt.image_file,
                                                   opt.camera_model_file,
                                                   opt.camera_model_file,
                                                   opt.output_file) );
    boost::shared_ptr<camera::CameraModel> camera_model =
      session->camera_model(opt.image_file, opt.camera_model_file);

    GeoReference dem_georef;
    ImageViewRef<PixelMask<float > > dem;

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
      dem = constant_view(PixelMask<float>(0), 360, 180 );
      vw_out() << "\t--> Using flat datum \"" << opt.dem_file << "\" as elevation model.\n";
    } else {
      // Open the DEM
      bool has_georef = cartography::read_georeference(dem_georef, opt.dem_file);
      if (!has_georef)
        vw_throw( ArgumentErr() << "There is no georeference information in: " << opt.dem_file << ".\n" );

      DiskImageView<float> dem_disk_image(opt.dem_file);
      dem = pixel_cast<PixelMask<float> >(dem_disk_image);

      // Attempting to extract automatic DEM nodata value.
      if ( std::isnan(opt.nodata_value) ) {
        boost::scoped_ptr<SrcImageResource> rsrc( DiskImageResource::open(opt.dem_file) );
        if ( rsrc->has_nodata_read() )
          opt.nodata_value = rsrc->nodata_read();
      }
      // If we have a nodata value, create a mask.
      if ( !std::isnan(opt.nodata_value) ) {
        vw_out() << "\t--> Using " << opt.nodata_value << " as the nodata value for the DEM.\n";
        dem = create_mask(dem_disk_image, opt.nodata_value);
      }
    }

    // Do the math to convert pixel-per-degree to meter-per-pixel and vice-versa
    double radius = dem_georef.datum().semi_major_axis();
    if ( !std::isnan(opt.mpp) && !std::isnan(opt.ppd) ) {
      vw_throw( ArgumentErr() << "Must specify either --mpp or --ppd option, never both.\n" );
      return 1;
    }else if ( !std::isnan(opt.mpp) ){
      opt.ppd = 2.0*M_PI*radius/(360.0*opt.mpp);
    }else if ( !std::isnan(opt.ppd) ){
      opt.mpp = 2.0*M_PI*radius/(360.0*opt.ppd);
    }

    // Use the output resolution specified by the user if provided
    double scale = 0;
    if ( !std::isnan(opt.ppd) ) {
      if (dem_georef.is_projected()) {
        scale = opt.mpp;
      } else {
        scale = 1/opt.ppd;
      }
    }

    ImageFormat texture_fmt;
    {
      boost::scoped_ptr<SrcImageResource> texture_rsrc( DiskImageResource::open(opt.image_file) );
      texture_fmt = texture_rsrc->format();
    }

    GeoReference drg_georef = dem_georef;
    // If the user has supplied a scale, we use it.  Otherwise, we
    // compute the optimal scale of image based on the camera model.
    BBox2 projection_bbox;
    {
      BBox2 cam_bbox, dem_bbox;
      vw_out() << "\t--> Extracting camera bbox ... " << std::flush;
      float mpp_auto_scale;
      cam_bbox =
        camera_bbox( dem, dem_georef, camera_model, texture_fmt.cols,
                     texture_fmt.rows, mpp_auto_scale );
      dem_bbox = dem_georef.bounding_box(dem);
      vw_out() << "done" << std::endl;

      // Use a bbox where both the image and the DEM have valid data.
      // Throw an error if there turns out to be no overlap.
      projection_bbox = cam_bbox; projection_bbox.crop(dem_bbox);
      
      if ( projection_bbox.empty() && !dem_georef.is_projected()) {

        // If the boxes do not intersect, it may be that one box is
        // shifted in respect to the other by 360 degrees.
        // Attempt 1
        projection_bbox = cam_bbox - Vector2(360.0, 0.0); projection_bbox.crop(dem_bbox);
        if ( projection_bbox.empty() ) {
          // Attempt 2
          projection_bbox = cam_bbox + Vector2(360.0, 0.0); projection_bbox.crop(dem_bbox);

          // If still no overlap, throw an error.
          if ( projection_bbox.empty() ) {
            vw_out() << "Image bounding box (" << cam_bbox << ") and DEM bounding box ("
                     << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
            return 1;
          }
        }
      }
      
      if ( scale == 0 )
        scale = mpp_auto_scale;
    }

    int min_x         = (int)round(projection_bbox.min().x() / scale);
    int min_y         = (int)round(projection_bbox.min().y() / scale);
    int output_width  = (int)round(projection_bbox.width()   / scale);
    int output_height = (int)round(projection_bbox.height()  / scale);

    // We must adjust the projection box given that we performed snapping to integer above.
    projection_bbox = scale*BBox2(min_x, min_y, output_width, output_height);
    
    vw_out( DebugMessage, "asp" ) << "Output size : "
                                  << output_width << " " << output_height << " px\n"
                                  << scale << " pt/px\n";

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
                     ConstantEdgeExtension(),
                     BicubicInterpolation()),
           0,0,output_width,output_height);
    
    vw_out( DebugMessage, "asp" ) << "Output GeoReference: "
                                  << drg_georef << "\n";

    if ( opt.do_color ) {
      vw_out() << "\t--> Orthoprojecting solid color image.\n";
      if (!opt.mark_no_processed_data){
        // Default
        asp::write_gdal_georeferenced_image(
          opt.output_file,
          orthoproject(output_dem, drg_georef,
                       constant_view(opt.color, texture_fmt.cols,
                                     texture_fmt.rows),
                       camera_model,
                       BicubicInterpolation(), ZeroEdgeExtension()),
          drg_georef, opt,
          TerminalProgressCallback("asp","") );
      } else {
        asp::write_gdal_georeferenced_image(
          opt.output_file,
          orthoproject_markNoProcessedData(output_dem, drg_georef,
                                           constant_view(opt.color, texture_fmt.cols,
                                                         texture_fmt.rows),
                                           camera_model,
                                           BicubicInterpolation(), ZeroEdgeExtension()),
          drg_georef, opt,
          TerminalProgressCallback("asp","") );
      }
    } else {
      switch(texture_fmt.pixel_format) {
      case VW_PIXEL_GRAY:
      case VW_PIXEL_GRAYA:
      default:
        switch(texture_fmt.channel_type) {
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
        switch(texture_fmt.channel_type) {
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


