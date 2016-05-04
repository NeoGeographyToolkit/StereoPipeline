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

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <boost/tokenizer.hpp>

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  Options() : lo(0), hi(0), mark_no_processed_data(false) {
    nodata_value = mpp = ppd = std::numeric_limits<double>::quiet_NaN();
  }

  // Input
  std::string dem_file, image_file, camera_model_file;

  // Settings
  std::string stereo_session, output_file;
  double mpp, ppd, nodata_value;
  float lo, hi;
  bool mark_no_processed_data;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  std::string color_text;
  po::options_description general_options("");
  general_options.add_options()
    ("mpp", po::value(&opt.mpp), "Specify the output resolution of the orthoimage in meters per pixel.")
    ("ppd", po::value(&opt.ppd), "Specify the output resolution of the orthoimage in pixels per degree.")
    ("nodata-value", po::value(&opt.nodata_value),
     "The nodata pixel value to use for the input DEM, overriding the value specified in the DEM.")
    ("session-type,t", po::value(&opt.stereo_session),
     "Select the stereo session type to use for processing. [default: pinhole]")
    ("mark-no-processed-data", po::bool_switch(&opt.mark_no_processed_data)->default_value(false),
     "If to set no-data pixels in the DEM which project onto the camera to black.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

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
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered  );

  if ( opt.dem_file.empty() || opt.image_file.empty() ||
       opt.camera_model_file.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n"
              << usage << general_options );
}

template <class ImageT>
void write_parallel_cond( std::string const& filename,
                          ImageViewBase<ImageT> const& image,
                          cartography::GeoReference const& georef,
                          bool use_nodata, double nodata_val,
                          Options const& opt,
                          TerminalProgressCallback const& tpc ) {
  // ISIS is not thread safe so we must switch out base on what the
  // session is.
  vw_out() << "Writing: " << filename << "\n";
  bool has_georef = true;
  if ( opt.stereo_session == "isis" )
    vw::cartography::write_gdal_image(filename, image.impl(), has_georef, georef,
                                      use_nodata, nodata_val, opt, tpc);
  else
    vw::cartography::block_write_gdal_image(filename, image.impl(), has_georef, georef,
                                use_nodata, nodata_val, opt, tpc);
}

// Convert PixelMask<PixelT> to PixelGrayA<PixelT> taking particular care
// with no-data values.
template <class ImageT, class OutPixelT>
class MaskToGrayA : public ImageViewBase<MaskToGrayA<ImageT, OutPixelT> > {
  ImageT m_masked_image;
  OutPixelT m_left_mask;
  double m_nodata_val;

public:
  MaskToGrayA( ImageViewBase<ImageT> const& masked_image,
               OutPixelT const& left_mask,
               double nodata_val) :
    m_masked_image(masked_image.impl()),
    m_left_mask(left_mask),
    m_nodata_val(nodata_val){}

  typedef OutPixelT pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<MaskToGrayA> pixel_accessor;

  inline int32 cols() const { return m_masked_image.cols(); }
  inline int32 rows() const { return m_masked_image.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline result_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "MaskToGrayA::operator()(...) is not implemented");
    return result_type();
  }

  typedef CropView<ImageView<result_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    ImageView<result_type> tileImg(bbox.width(), bbox.height());
    for (int col = 0; col < bbox.width(); col++){
      for (int row = 0; row < bbox.height(); row++){
        typename ImageT::result_type pix = m_masked_image(col + bbox.min().x(),
                                                          row + bbox.min().y());
        if (is_valid(pix))
          tileImg(col, row) = result_type(pix.child());     // valid and non-transparent
        else
          tileImg(col, row) = result_type(m_nodata_val, 0); // invalid and transparent
      }
    }

    return prerasterize_type(tileImg, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class ImageT, class OutPixelT>
MaskToGrayA<ImageT, OutPixelT>
mask_to_graya( ImageViewBase<ImageT> const& left,
               OutPixelT const& out_pixel,
               double nodata_val) {
  typedef MaskToGrayA<ImageT, OutPixelT> return_type;
  return return_type(left.impl(), out_pixel, nodata_val );
}

template <class PixelT>
void do_projection( Options& opt,
                    ImageViewRef<PixelMask<float> > const& dem,
                    GeoReference const& drg_georef,
                    boost::shared_ptr<camera::CameraModel> camera_model ) {

  DiskImageView<PixelT> texture_disk_image(opt.image_file);

  bool use_nodata = false;
  double nodata_val = std::numeric_limits<double>::quiet_NaN();

  if (!opt.mark_no_processed_data){
    // Default
    write_parallel_cond( opt.output_file,
                         orthoproject(dem, drg_georef,
                                      texture_disk_image, camera_model.get(),
                                      BicubicInterpolation(), ZeroEdgeExtension()),
                         drg_georef, use_nodata, nodata_val, opt,
                         TerminalProgressCallback("asp","") );
  } else {
    // Save as uint8, need this for albedo
    write_parallel_cond( opt.output_file,
                         pixel_cast_rescale< PixelGrayA<uint8> >
                         (clamp(orthoproject_markNoProcessedData
                                (dem, drg_georef,
                                 texture_disk_image, camera_model.get(),
                                 BicubicInterpolation(), ZeroEdgeExtension()),
                                0, 32767)),
                         drg_georef, use_nodata, nodata_val, opt,
                         TerminalProgressCallback("asp","") );
  }

}

template <class PixelT>
void do_projection_scalar(Options& opt,
                          ImageViewRef<PixelMask<float> > const& dem,
                          GeoReference const& drg_georef,
                          boost::shared_ptr<camera::CameraModel> camera_model ) {

  // Orthoprojection of scalar images, when there can be no-data values
  // which need to be handled.

  // Find the smallest valid value for this type.
  PixelT smallest_val = std::min(std::numeric_limits<PixelT>::min(),
                                 PixelT(-std::numeric_limits<PixelT>::max())
                                 );

  // We will always use a nodata value on output, even when the input
  // has none. Orthoprojecting will one way or another create invalid
  // pixels, we must map them to something. Use the smallest valid
  // pixel value as no-data value if it was not present in the input.
  double nodata_val = smallest_val;
  bool use_nodata = true;

  // Mask the input nodata values.
  DiskImageView<PixelT> texture_disk_image(opt.image_file);
  ImageViewRef< PixelMask<PixelT> > masked_texture;
  masked_texture = pixel_cast< PixelMask<PixelT> >(texture_disk_image);
  boost::scoped_ptr<SrcImageResource> rsrc( DiskImageResource::open(opt.image_file) );
  if ( rsrc->has_nodata_read() ){
    nodata_val = rsrc->nodata_read();
    masked_texture = create_mask(texture_disk_image, nodata_val);
  }

  // This is needed purely to force mask_to_graya() output pixels of this type.
  PixelGrayA<PixelT> out_pixel;

  if (!opt.mark_no_processed_data){
    // Default
    write_parallel_cond( opt.output_file,
                         mask_to_graya
                         (orthoproject
                          (dem, drg_georef,
                           masked_texture,
                           camera_model.get(),
                           BicubicInterpolation(),
                           ZeroEdgeExtension()),
                          out_pixel, nodata_val),
                         drg_georef, use_nodata, nodata_val, opt,
                         TerminalProgressCallback("asp","") );
  } else {
    // Save as uint8, need this for albedo
    write_parallel_cond( opt.output_file,
                         pixel_cast_rescale< PixelGrayA<uint8> >
                         (clamp
                          (mask_to_graya
                           (orthoproject_markNoProcessedData
                            (dem, drg_georef,
                             masked_texture,
                             camera_model.get(),
                             BicubicInterpolation(),
                             ZeroEdgeExtension()),
                            out_pixel, nodata_val), 0, 32767)),
                         drg_georef, use_nodata, nodata_val, opt,
                         TerminalProgressCallback("asp","") );
  }

}

int main(int argc, char* argv[]) {

  vw_out(WarningMessage) << "orthoproject is obsolete and will be dropped in future releases. The tool named mapproject is its replacement.\n";

  // Orthorpoject a camera image onto a DEM.
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // We create a stereo session where both of the cameras and images
    // are the same, because we want to take advantage of the stereo
    // pipeline's ability to generate camera models for various
    // missions.  Hence, we create two identical camera models, but
    // only one is used.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSessionFactory::create(opt.stereo_session, // in-out
                                                           opt,
                                                           opt.image_file, opt.image_file,
                                                           opt.camera_model_file,
                                                           opt.camera_model_file,
                                                           opt.output_file) );

    if (session->name() == "isis" && opt.output_file.empty() ){
      // The user did not provide an output file. Then the camera
      // information is contained within the image file and what is in
      // the camera file is actually the output file.
      opt.output_file = opt.camera_model_file;
      opt.camera_model_file.clear();
    }
    if ( opt.output_file.empty() )
      vw_throw( ArgumentErr() << "Missing output filename.\n" );


    boost::shared_ptr<camera::CameraModel> camera_model =
      session->camera_model(opt.image_file, opt.camera_model_file);

    if ( opt.stereo_session != "isis" && opt.stereo_session != "pinhole" ) {
      vw_throw( ArgumentErr() << "Supported sessions: [pinhole isis].\n" );
    }

    // Load the DEM
    GeoReference dem_georef;
    ImageViewRef<PixelMask<float > > dem;

    if ( fs::path(opt.dem_file).extension() == "" ) {
      // This option allows the user to just project directly unto to
      // the datum. This seems like a stop gap. This should be
      // addressed after a rewrite of orthoproject.
      //
      // Valid values are well known Datums like D_MOON D_MARS WGS84 and so on.
      Vector3 llr_camera_loc =
        cartography::XYZtoLonLatRadEstimateFunctor::apply( camera_model->camera_center(Vector2()) );
      if ( llr_camera_loc[0] < 0 ) llr_camera_loc[0] += 360;

      dem_georef =
        GeoReference(Datum(opt.dem_file),
                     Matrix3x3(1, 0, (llr_camera_loc[0] < 90 ||
                                      llr_camera_loc[0] > 270) ? -180 : 0,
                               0, -1, 90, 0, 0, 1) );
      dem = constant_view(PixelMask<float>(0), 360, 180 );
      vw_out() << "\t--> Using flat datum \"" << opt.dem_file << "\" as elevation model.\n";
    } else {
      bool has_georef = cartography::read_georeference(dem_georef, opt.dem_file);
      if (!has_georef)
        vw_throw( ArgumentErr() << "There is no georeference information in: "
                  << opt.dem_file << ".\n" );

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
      vw_throw( ArgumentErr()
                << "Must specify either --mpp or --ppd option, but not both.\n" );
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
      boost::scoped_ptr<SrcImageResource>
        texture_rsrc( DiskImageResource::open(opt.image_file) );
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
        projection_bbox = cam_bbox - Vector2(360.0, 0.0);
        projection_bbox.crop(dem_bbox);
        if ( projection_bbox.empty() ) {
          // Attempt 2
          projection_bbox = cam_bbox + Vector2(360.0, 0.0);
          projection_bbox.crop(dem_bbox);

          // If still no overlap, throw an error.
          if ( projection_bbox.empty() ) {
            vw_out() << "Image bounding box (" << cam_bbox
                     << ") and DEM bounding box (" << dem_bbox
                     << ") have no overlap.\n";
            return 1;
          }
        }
      }

      if ( scale == 0 ) scale = mpp_auto_scale;
    }


    // In principle the corners of the projection box can be
    // arbitrary.  However, we will force them to be at integer
    // multiples of pixel dimensions. This is needed if we want to do
    // tiling, that is break the DEM into tiles, orthoproject on
    // individual tiles, and then combine the tiles nicely without
    // seams into a single orthoprojected image. The tiling solution
    // provides a nice speedup when dealing with ISIS images, when
    // orthoproject runs only with one thread.
    int min_x         = (int)round(projection_bbox.min().x() / scale);
    int min_y         = (int)round(projection_bbox.min().y() / scale);
    int output_width  = (int)round(projection_bbox.width()   / scale);
    int output_height = (int)round(projection_bbox.height()  / scale);
    projection_bbox = scale*BBox2(min_x, min_y, output_width, output_height);

    vw_out( DebugMessage, "asp" )
      << "Output size: " << output_width << " x " << output_height << " px\n"
      << scale << " pt/px\n";

    Matrix3x3 drg_trans = drg_georef.transform();
    // This polarity checking is to make sure the output has been
    // transposed after going through reprojection. Normally this is
    // the case. Yet with grid data from GMT, it is not.
    if ( drg_trans(0,0) < 0 )
      drg_trans(0,2) = projection_bbox.max().x();
    else
      drg_trans(0,2) = projection_bbox.min().x();
    drg_trans(0,0) = scale;
    drg_trans(1,1) = -scale;
    drg_trans(1,2) = projection_bbox.max().y();
    if ( drg_georef.pixel_interpretation() ==
         cartography::GeoReference::PixelAsArea ) {
      drg_trans(0,2) -= 0.5 * scale;
      drg_trans(1,2) += 0.5 * scale;
    }
    drg_georef.set_transform(drg_trans);

    GeoTransform trans(dem_georef, drg_georef);
    ImageViewRef<PixelMask<float> > output_dem =
      crop(transform(dem, trans,
                     ConstantEdgeExtension(),
                     BicubicInterpolation()),
           0,0,output_width,output_height);

    vw_out( DebugMessage, "asp" ) << "Output georeference: "
                                  << drg_georef << "\n";

    // Create and write the orthoprojected image.
    vw::create_out_dir(opt.output_file);
    switch(texture_fmt.pixel_format) {

      // 1.0. RGB, with our without alpha channel
    case VW_PIXEL_RGB:
    case VW_PIXEL_RGBA:
      switch(texture_fmt.channel_type) {
      case VW_CHANNEL_UINT8:
        do_projection<PixelRGBA<uint8> >( opt, output_dem, drg_georef,
                                          camera_model ); break;
      case VW_CHANNEL_INT16:
        do_projection<PixelRGBA<int16> >( opt, output_dem, drg_georef,
                                          camera_model ); break;
      case VW_CHANNEL_UINT16:
        do_projection<PixelRGBA<uint16> >( opt, output_dem, drg_georef,
                                           camera_model ); break;
      default:
        do_projection<PixelRGBA<float32> >( opt, output_dem, drg_georef,
                                            camera_model ); break;
      }
      break;

      // 2.0. Grayscale images with alpha channel
    case VW_PIXEL_GRAYA:
      switch(texture_fmt.channel_type) {
      case VW_CHANNEL_UINT8:
        do_projection<PixelGrayA<uint8> >( opt, output_dem, drg_georef,
                                           camera_model ); break;
      case VW_CHANNEL_INT16:
        do_projection<PixelGrayA<int16> >( opt, output_dem, drg_georef,
                                           camera_model ); break;
      case VW_CHANNEL_UINT16:
        do_projection<PixelGrayA<uint16> >( opt, output_dem, drg_georef,
                                            camera_model ); break;
      default:
        do_projection<PixelGrayA<float32> >( opt, output_dem, drg_georef,
                                             camera_model ); break;
      }
      break;

      // 3.0. Grayscale images without alpha channel and all other
      // cases.
    case VW_PIXEL_GRAY:
    default:
      switch(texture_fmt.channel_type) {
      case VW_CHANNEL_UINT8:
        do_projection_scalar<uint8>( opt, output_dem, drg_georef,
                                     camera_model ); break;
      case VW_CHANNEL_INT16:
        do_projection_scalar<int16>( opt, output_dem, drg_georef,
                                     camera_model ); break;
      case VW_CHANNEL_UINT16:
        do_projection_scalar<uint16>( opt, output_dem, drg_georef,
                                      camera_model ); break;
      default:
        do_projection_scalar<float32>( opt, output_dem, drg_georef,
                                       camera_model ); break;
      }
      break;
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
