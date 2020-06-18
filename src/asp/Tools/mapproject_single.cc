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

/// \file mapproject.cc
///
/// This program will project a camera image onto a DEM using the
/// camera model.
/// TODO: This creates an RGBA output for an RGB input.
/// It should create an RBG output for RGB input,
/// and same for RGBA.

#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Image/Algorithms2.h>
#include <vw/Image/Filter.h>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>

#include <boost/algorithm/string/replace.hpp>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;



/// The pixel type used for the DEM data
typedef PixelMask<float> DemPixelT;


struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::string dem_file, image_file, camera_file, output_file, stereo_session,
    bundle_adjust_prefix;
  bool isQuery, noGeoHeaderInfo, nearest_neighbor;
  bool multithreaded_model; // This is set based on the session type.

  // Settings
  std::string target_srs_string, output_type, metadata;
  double nodata_value, tr, mpp, ppd, datum_offset;
  BBox2 target_projwin, target_pixelwin;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  double NaN = std::numeric_limits<double>::quiet_NaN();
  general_options.add_options()
    ("nodata-value",     po::value(&opt.nodata_value)->default_value(-32768),
     "No-data value to use unless specified in the input image.")
    ("t_srs",            po::value(&opt.target_srs_string)->default_value(""),
     "Specify the projection (PROJ.4 string). If not provided, use the one from the DEM.")
    ("tr",              po::value(&opt.tr)->default_value(NaN),
     "Set the output file resolution in projection space coordinates.")
    ("mpp",              po::value(&opt.mpp)->default_value(NaN),
     "Set the output file resolution in meters per pixel.")
    ("ppd",              po::value(&opt.ppd)->default_value(NaN),
     "Set the output file resolution in pixels per degree.")
    ("datum-offset",     po::value(&opt.datum_offset)->default_value(0),
     "When projecting to a datum instead of a DEM, use this elevation in meters from the datum.")
    ("query-projection", po::bool_switch(&opt.isQuery)->default_value(false),
      "Just display the computed projection information without actually doing the projection.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Choose 'rpc' if it is desired to later do stereo with the 'dg' session. [options: nadirpinhole pinhole isis dg rpc spot5 aster opticalbar csm]")
    ("t_projwin",        po::value(&opt.target_projwin),
     "Limit the map-projected image to this region, with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.")
    ("t_pixelwin",       po::value(&opt.target_pixelwin),
     "Limit the map-projected image to this region, with the corners given in pixels (xmin ymin xmax ymax). Max is exclusive.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    ("ot",  po::value(&opt.output_type)->default_value("Float32"), "Output data type, when the input is single channel. Supported types: Byte, UInt16, Int16, UInt32, Int32, Float32. If the output type is a kind of integer, values are rounded and then clamped to the limits of that type. This option will be ignored for multi-channel images, when the output type is set to be the same as the input type.")
    ("nearest-neighbor", po::bool_switch(&opt.nearest_neighbor)->default_value(false),
      "Use nearest neighbor interpolation.  Useful for classification images.")
    ("mo",  po::value(&opt.metadata)->default_value(""), "Write metadata to the output file. Provide as a string in quotes if more than one item, separated by a space, such as 'VAR1=VALUE1 VAR2=VALUE2'. Neither the variable names nor the values should contain spaces.")
    ("no-geoheader-info", po::bool_switch(&opt.noGeoHeaderInfo)->default_value(false),
     "Suppress writing some auxiliary information in geoheaders.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem",          po::value(&opt.dem_file))
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file))
    ("output-image" , po::value(&opt.output_file));

  po::positional_options_description positional_desc;
  positional_desc.add("dem",          1);
  positional_desc.add("camera-image", 1);
  positional_desc.add("camera-model", 1);
  positional_desc.add("output-image", 1);

  std::string usage("[options] <dem> <camera-image> <camera-model> <output-image>\nInstead of the DEM file, a datum can be provided, such as\nWGS84, NAD83, NAD27, D_MOON, D_MARS, and MOLA.");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( !vm.count("dem") || !vm.count("camera-image") || !vm.count("camera-model") )
    vw_throw( ArgumentErr() << usage << general_options );

  // If exactly three files were passed in, the last one must be the output file and the image file
  // must contain the camera model.
  if ( !vm.count("output-image") && vm.count("camera-model") ) {
    opt.output_file = opt.camera_file;
    opt.camera_file = "";
  }

  // We support map-projecting using the DG camera model, however, these images
  // cannot be used later to do stereo, as that process expects the images
  // to be map-projected using the RPC model.
  if (boost::to_lower_copy(opt.stereo_session) == "dg"){
    vw_out(WarningMessage) << "Images map-projected using the 'dg' camera model cannot be used later to run stereo with the 'dg' session. If that is desired, please specify here the 'rpc' camera model instead.\n";
  }

  if ( boost::iends_with(boost::to_lower_copy(opt.camera_file), ".xml") &&
       opt.stereo_session == "" ){
    opt.stereo_session = "rpc";
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  if (fs::path(opt.dem_file).extension() != "") {
    // A path to a real DEM file was provided, load it!
    GeoReference dem_georef;
    bool has_georef = vw::cartography::read_georeference(dem_georef, opt.dem_file);
    if (!has_georef)
      vw_throw( ArgumentErr() << "There is no georeference information in: "
                              << opt.dem_file << ".\n" );

    // Make sure the user did not actually pass in an RGB image
    boost::shared_ptr<DiskImageResource> dem_rsrc = vw::DiskImageResourcePtr(opt.dem_file);
    ImageFormat dem_fmt = dem_rsrc->format();
    const int num_input_channels = num_channels(dem_fmt.pixel_format);
    if (num_input_channels > 2)
      vw_throw( ArgumentErr() << "Too many channels in: " << opt.dem_file << ".\n" );


    // Store the datum from the DEM
    // TODO (oalexan1): Fix here
    asp::stereo_settings().datum = dem_georef.datum().name(); 
  }
  
}

// If the output type is some type of int, round and clamp to this
// type, both the pixels and the nodata-value. Else write as it is.
template <class ImageT>
void write_parallel_type( std::string              const& filename,
                          ImageT                   const& image,
                          GeoReference             const& georef,
                          bool has_nodata, double nodata_val,
                          Options                  const& opt,
                          TerminalProgressCallback const& tpc ) {

  typedef typename ImageT::pixel_type InputType;
  
  if (opt.output_type == "Float32") 
    write_parallel_cond(filename, image, georef, has_nodata, nodata_val, opt, tpc);
  else if (opt.output_type == "Byte") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint8, InputType>()),
			georef, has_nodata,
			vw::round_and_clamp<uint8>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "UInt16") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint16, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<uint16>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "Int16") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<int16, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<int16>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "UInt32") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint32, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<uint32>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "Int32") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<int32, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<int32>(nodata_val),
			opt, tpc);
  else
    vw_throw( NoImplErr() << "Unsupported output type: " << opt.output_type << ".\n" );
  
}

template <class ImageT>
void write_parallel_cond( std::string              const& filename,
                          ImageViewBase<ImageT>    const& image,
                          GeoReference             const& georef,
                          bool has_nodata, double nodata_val,
                          Options                  const& opt,
                          TerminalProgressCallback const& tpc ) {

  // TODO: Relying on StereoSession to handle this mapproject stuff is problematic!
  // Save the session type. Later in stereo we will check that we use
  // only images written by mapproject with the -t rpc session.
  // - Write the type of sensor model used, not the underlying session class.
  std::string session_type = opt.stereo_session;
  if (session_type == "isismapisis")
    session_type = "isis";
  if (session_type == "rpcmaprpc")
    session_type = "rpc";
  if (session_type == "pinholemappinhole")
    session_type = "pinhole";

  // Save some keywords that we will check later when using the mapprojected file
  std::map<std::string, std::string> keywords;
  if (! opt.noGeoHeaderInfo) {
    keywords["CAMERA_MODEL_TYPE" ]    = session_type;
    std::string prefix = asp::stereo_settings().bundle_adjust_prefix;;
    if (prefix == "") prefix = "NONE"; // to save the field, need to make it non-empty
    keywords["BUNDLE_ADJUST_PREFIX" ] = prefix;
    keywords["DEM_FILE" ]             = opt.dem_file;

    // Parse keywords from --mo.
    asp::parse_append_metadata(opt.metadata, keywords);
  }
  
  bool has_georef = true;

  // ISIS is not thread safe so we must switch out based on what the session is.
  vw_out() << "Writing: " << filename << "\n";
  if (opt.multithreaded_model) {
    vw::cartography::block_write_gdal_image(filename, image.impl(), has_georef, georef,
                                has_nodata, nodata_val, opt, tpc, keywords);
  } else {
    vw::cartography::write_gdal_image(filename, image.impl(), has_georef, georef,
                          has_nodata, nodata_val, opt, tpc, keywords);
  }

}

/// Compute which camera pixel observes a DEM pixel.
Vector2 demPixToCamPix(Vector2i const& dem_pixel,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<DemPixelT> const& dem,
                      GeoReference const &dem_georef)
{
  Vector2 lonlat = dem_georef.point_to_lonlat(dem_georef.pixel_to_point(dem_pixel));
  //vw_out() << "lonlat = " << lonlat << std::endl;
  DemPixelT height = dem(dem_pixel[0], dem_pixel[1]);
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian
                    (Vector3(lonlat[0], lonlat[1], height.child()));
  //vw_out() << "xyz = " << xyz << std::endl;
  // Throws if the projection fails ???
  Vector2i camera_pixel = camera_model->point_to_pixel(xyz);
  //vw_out() << "camera_pixel = " << camera_pixel << std::endl;
  return camera_pixel;
}

/// Expand the ground BBox to contain all the corners of the DEM if they intersect the camera.
/// - TODO: This method still does not guarantee all points will be included in the bbox.
/// - TODO: This should probably take pixel validity into account!
void expandBboxToContainCornerIntersections(boost::shared_ptr<camera::CameraModel> const& camera_model,
                                            ImageViewRef<DemPixelT> const& dem,
                                            GeoReference const &dem_georef,
                                            Vector2i const& image_size,
                                            BBox2 & bbox_on_ground)
{
  // Each of the corners of the DEM
  std::vector<Vector2> dem_pixel_list(4);
  dem_pixel_list[0] = Vector2(0,            0           );
  dem_pixel_list[1] = Vector2(dem.cols()-1, 0           );
  dem_pixel_list[2] = Vector2(dem.cols()-1, dem.rows()-1);
  dem_pixel_list[3] = Vector2(0,            dem.rows()-1);

  for (int i=0; i<4; ++i) {
    try{
      // Project the DEM corner into the input image
      Vector2 dem_pixel = dem_pixel_list[i];
      Vector2 cam_pixel = demPixToCamPix(dem_pixel, camera_model, dem, dem_georef);

      // Get the point on the ground
      Vector2 groundLoc = dem_georef.pixel_to_point(dem_pixel);

      // If there was in intersection...
      if ( (cam_pixel.x() >= 0)              && (cam_pixel.y() > 0) &&
           (cam_pixel.x() <  image_size.x()) && (cam_pixel.y() < image_size.y()) ) {
        //Vector2 lonlat    = dem_georef.point_to_lonlat(dem_georef.pixel_to_point(dem_pixel));
        // Expand the ground bbox to contain in
        bbox_on_ground.grow(groundLoc);
        //vw_out() << "Grow --> " << groundLoc  << std::endl;
      }
      else{
        //vw_out() << "Miss! "  << std::endl;
      }
    }catch(...){
      //vw_out() << "Bad projection! "  << std::endl;
    } // If a point failed to project
  } // End loop through DEM points

}

/// Compute output georeference to use
void calc_target_geom(// Inputs
                      bool calc_target_res,
                      Vector2i const& image_size,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<DemPixelT> const& dem,
                      GeoReference const& dem_georef, 
                      bool datum_dem,
                      Options const & opt,
                      // Outputs
                      BBox2 & cam_box, GeoReference & target_georef){

  // Find the camera bbox and the target resolution unless user-supplied.
  // - This call returns the bounding box of the camera view on the ground.
  // - The bounding box is in units defined by dem_georef and might not be meters.
  // - auto_res is an estimate of the ground resolution visible by the camera.
  //   This is in a unit defined by dem_georef and also might not be meters.
  // - This call WILL intersect pixels outside the dem valid area!
  // - TODO: Modify this function to optionally disable intersection outside the DEM
  float auto_res;
  bool quick = datum_dem; // The non-quick option does not make sense with huge DEMs.
  try {
    cam_box = camera_bbox(dem, dem_georef,
                          target_georef, 
                          camera_model,
                          image_size.x(), image_size.y(), auto_res, quick);
  } catch (std::exception const& e) {
    if (opt.target_projwin == BBox2() || calc_target_res) {
      vw_throw( ArgumentErr()
                << e.what() << "\n"
                << "Check your inputs. Or try specifying --t_projwin and --tr values.\n");
    }
  }

  // Use auto-calculated ground resolution if that option was selected
  double current_resolution;
  if (calc_target_res) {
    current_resolution = auto_res;
  } else {
    // Set the resolution from input options
    if (target_georef.is_projected()) {
      current_resolution = opt.mpp; // Use units of meters
    } else { // Not projected, GDC coordinates only.
      current_resolution = 1/opt.ppd; // Use units of degrees
                                      // Lat/lon degrees are different so we never want to do this!
    }
  }
  vw_out() << "Output pixel size: " << current_resolution << std::endl;

  // If an image bounding box (projected coordinates) was passed in,
  // override the camera's view on the ground with the custom box.
  // The user needs to know the georeference projected coordinate
  // system (using the query command) to do this.
  if ( opt.target_projwin != BBox2() ) {
    cam_box = opt.target_projwin;
    if ( cam_box.min().y() > cam_box.max().y() )
      std::swap( cam_box.min().y(), cam_box.max().y() );
    // The adjustments below are possibly to make the maximum
    // non-exclusive.
    cam_box.max().x() -= current_resolution; 
    cam_box.min().y() += current_resolution;
  }

  // In principle the corners of the projection box can be
  // arbitrary.  However, we will force them to be at integer
  // multiples of pixel dimensions. This is needed if we want to do
  // tiling, that is break the DEM into tiles, project on individual
  // tiles, and then combine the tiles nicely without seams into a
  // single projected image. The tiling solution provides a nice
  // speedup when dealing with ISIS images, when projection runs
  // only with one thread.
  double s = current_resolution;
  int min_x         = (int)round(cam_box.min().x() / s);
  int min_y         = (int)round(cam_box.min().y() / s);
  int output_width  = (int)round(cam_box.width()   / s);
  int output_height = (int)round(cam_box.height()  / s);
  cam_box = s * BBox2(min_x, min_y, output_width, output_height);

  // This transform is from pixel to projected coordinates
  Matrix3x3 T = target_georef.transform();
  // This polarity checking is to make sure the output has been
  // transposed after going through reprojection. Normally this is
  // the case. Yet with grid data from GMT, it is not.
  if ( T(0,0) < 0 ) // If X coefficient of affine transform is negative (cols go opposite direction from projected x coords)
    T(0,2) = cam_box.max().x();  // The maximum projected X coordinate is the starting offset
  else
    T(0,2) = cam_box.min().x(); // The minimum projected X coordinate is the starting offset
  T(0,0) =  current_resolution;  // Set col/X conversion to meters per pixel
  T(1,1) = -current_resolution;  // Set row/Y conversion to meters per pixel with a vertical flip (increasing row = down in Y)
  T(1,2) = cam_box.max().y();       // The maximum projected Y coordinate is the starting offset
  if ( target_georef.pixel_interpretation() ==
       GeoReference::PixelAsArea ) { // Meaning point [0][0] equals location (0.5, 0.5)
    T(0,2) -= 0.5 * current_resolution; // Apply a small shift to the offsets
    T(1,2) += 0.5 * current_resolution;
  }
  target_georef.set_transform( T ); // Overwrite the existing transform in target_georef


  // Compute output image size in pixels using bounding box in output projected space
  BBox2i target_image_size = target_georef.point_to_pixel_bbox( cam_box );

  // Last adjustment, to ensure 0 0 is always in the box corner
  target_georef = crop(target_georef, target_image_size.min().x(), target_image_size.min().y());

  return;
}


/// Map project the image with a nodata value.  Used for single channel images.
template <class ImagePixelT, class Map2CamTransT>
void project_image_nodata(Options & opt,
                          GeoReference const& croppedGeoRef,
                          Vector2i     const& virtual_image_size,
                          BBox2i       const& croppedImageBB,
                          Map2CamTransT const& transform) {

    typedef PixelMask<ImagePixelT> ImageMaskPixelT;

    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource> img_rsrc = 
          vw::DiskImageResourcePtr(opt.image_file);   

    // Update the nodata value from the input file if it is present.
    if (img_rsrc->has_nodata_read()) 
      opt.nodata_value = img_rsrc->nodata_read();

    bool            has_img_nodata = true;
    ImageMaskPixelT nodata_mask    = ImageMaskPixelT(); // invalid value for a PixelMask

    // TODO: This is a lot of code duplication, is there a better way?
    if (opt.nearest_neighbor) {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              apply_mask
              ( // Handle nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                create_mask(DiskImageView<ImagePixelT>(img_rsrc),
                                            opt.nodata_value), // Handle nodata
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                NearestPixelInterpolation(), nodata_mask
                                ),
              opt.nodata_value
              ),
              croppedImageBB
              ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    } else {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              apply_mask
              ( // Handle nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                create_mask(DiskImageView<ImagePixelT>(img_rsrc),
                                            opt.nodata_value), // Handle nodata
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                BicubicInterpolation(), nodata_mask
                                ),
              opt.nodata_value
              ),
              croppedImageBB
              ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    }

}

/// Map project the image with an alpha channel.  Used for multi-channel images.
template <class ImagePixelT, class Map2CamTransT>
void project_image_alpha(Options & opt,
                   GeoReference const& croppedGeoRef,
                   Vector2i     const& virtual_image_size,
                   BBox2i       const& croppedImageBB,
                   boost::shared_ptr<camera::CameraModel> const& camera_model,
                   Map2CamTransT const& transform) {

    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource> img_rsrc = 
          vw::DiskImageResourcePtr(opt.image_file);   

    const bool        has_img_nodata    = false;
    const ImagePixelT transparent_pixel = ImagePixelT();

    // TODO: Is it possible to reduce code duplication?
    if (opt.nearest_neighbor) {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              // Transparent pixels are inserted for nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                DiskImageView<ImagePixelT>(img_rsrc),
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ConstantEdgeExtension(),
                                NearestPixelInterpolation(), transparent_pixel
                                ),
              croppedImageBB
            ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    } else {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              // Transparent pixels are inserted for nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                DiskImageView<ImagePixelT>(img_rsrc),
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ConstantEdgeExtension(),
                                BicubicInterpolation(), transparent_pixel
                                ),
              croppedImageBB
            ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    }

}

// The two "pick" functions below select between the Map2CamTrans and Datum2CamTrans
// transform classes which will be passed to the image projection function.
// - TODO: Is there a good reason for the transform classes to be CRTP instead of virtual?

template <class ImagePixelT>
void project_image_nodata_pick_transform(Options & opt,
                          GeoReference const& dem_georef,
                          GeoReference const& target_georef,
                          GeoReference const& croppedGeoRef,
                          Vector2i     const& image_size,
                          Vector2i     const& virtual_image_size,
                          BBox2i       const& croppedImageBB,
                          boost::shared_ptr<camera::CameraModel> const& camera_model) {
  const bool        call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_nodata<ImagePixelT>(opt, croppedGeoRef,
                                             virtual_image_size, croppedImageBB,
                                             Map2CamTrans(// Converts coordinates in DEM
                                                          // georeference to camera pixels
                                                          camera_model.get(), target_georef,
                                                          dem_georef, opt.dem_file, image_size,
                                                          call_from_mapproject,
                                                          opt.nearest_neighbor));
  } else {
    // A constant datum elevation was provided
    return project_image_nodata<ImagePixelT>(opt, croppedGeoRef,
                                             virtual_image_size, croppedImageBB,
                                             Datum2CamTrans(// Converts coordinates in DEM
                                                            // georeference to camera pixels
                                                            camera_model.get(), target_georef,
                                                            dem_georef, opt.datum_offset, image_size,
                                                            call_from_mapproject,
                                                            opt.nearest_neighbor));
  }
}

template <class ImagePixelT>
void project_image_alpha_pick_transform(Options & opt,
                          GeoReference const& dem_georef,
                          GeoReference const& target_georef,
                          GeoReference const& croppedGeoRef,
                          Vector2i     const& image_size,
                          Vector2i     const& virtual_image_size,
                          BBox2i       const& croppedImageBB,
                          boost::shared_ptr<camera::CameraModel> const& camera_model) {
  const bool        call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_alpha<ImagePixelT>(opt, croppedGeoRef,
                                            virtual_image_size, croppedImageBB, camera_model, 
                                            Map2CamTrans( // Converts coordinates in DEM
                                                          // georeference to camera pixels
                                                         camera_model.get(), target_georef,
                                                         dem_georef, opt.dem_file, image_size,
                                                         call_from_mapproject,
                                                         opt.nearest_neighbor)
                                           );
  } else {
    // A constant datum elevation was provided
    return project_image_alpha<ImagePixelT>(opt, croppedGeoRef,
                                            virtual_image_size, croppedImageBB, camera_model, 
                                            Datum2CamTrans( // Converts coordinates in DEM
                                                            // georeference to camera pixels
                                                           camera_model.get(), target_georef,
                                                           dem_georef, opt.datum_offset, image_size,
                                                           call_from_mapproject,
                                                           opt.nearest_neighbor)
                                            );
  }
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);
  
    // TODO: Replace this using the new CameraModelLoader functions

    // We create a stereo session where both of the cameras and images
    // are the same, because we want to take advantage of the stereo
    // pipeline's ability to generate camera models for various
    // missions.  Hence, we create two identical camera models, but only one is used.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSessionFactory::create
                        (opt.stereo_session, // in-out
                         opt,
                         opt.image_file, opt.image_file, // The same file is passed in twice
                         opt.camera_file, opt.camera_file,
                         opt.output_file,
                         opt.dem_file,
                         false) ); // Do not allow promotion from normal to map projected session

    if ( opt.output_file.empty() )
      vw_throw( ArgumentErr() << "Missing output filename.\n" );

    // Initialize a camera model
    boost::shared_ptr<camera::CameraModel> camera_model =
      session->camera_model(opt.image_file, opt.camera_file);

    opt.multithreaded_model = session->supports_multi_threading();
      
    {
      // Safety check that the users are not trying to map project map
      // projected images. This should not be an error as sometimes
      // even raw images have some half-baked georeference attached to them.
      GeoReference dummy_georef;
      bool has_georef = vw::cartography::read_georeference( dummy_georef, opt.image_file );
      if (has_georef)
        vw_out(WarningMessage) << "Your input camera image is already map-"
                               << "projected. The expected input is required "
                               << "to be unprojected or raw camera imagery.\n";
    }

    // Load the DEM
    bool datum_dem = false;
    GeoReference dem_georef;
    ImageViewRef<DemPixelT> dem;
    if (fs::path(opt.dem_file).extension() != "") {
      // A path to a real DEM file was provided, load it!

      bool has_georef = vw::cartography::read_georeference(dem_georef, opt.dem_file);
      if (!has_georef)
        vw_throw( ArgumentErr() << "There is no georeference information in: " << opt.dem_file << ".\n" );

      boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(opt.dem_file));

      // If we have a nodata value, create a mask.
      DiskImageView<float> dem_disk_image(opt.dem_file);
      if (dem_rsrc->has_nodata_read()){
        dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
      }else{
        dem = pixel_cast<DemPixelT>(dem_disk_image);
      }      
    } else {
      // Projecting to a datum instead of a DEM
      datum_dem = true;
      std::string datum_name = opt.dem_file;

      // Use the camera center to determine whether to center the fake DEM on 0 or 180.
      Vector3 llr_camera_loc =
        cartography::XYZtoLonLatRadEstimateFunctor::apply( camera_model->camera_center(Vector2()) );
      float lonstart = 0;
      if ((llr_camera_loc[0] < 0) && (llr_camera_loc[0] > -180))
        lonstart = -180;
      dem_georef = GeoReference(Datum(datum_name),
                                Matrix3x3(1,  0, lonstart-0.5, // Need adjustments to work at boundaries!
                                          0, -1, 90+0.5,
                                          0,  0,  1) );
      dem = constant_view(PixelMask<float>(opt.datum_offset), 360, 180 );
      vw_out() << "\t--> Using flat datum \"" << datum_name << "\" as elevation model.\n";
      //std::cout << "dem_georef = " << dem_georef << std::endl;
    }
    // Finished setting up the datum

    // Read projection. Work out output bounding box in points using original camera model.
    GeoReference target_georef = dem_georef;

    // User specified the proj4 string for the output georeference
    if (opt.target_srs_string != ""){
      bool  have_user_datum = false;
      Datum user_datum;
      asp::set_srs_string(opt.target_srs_string, have_user_datum, user_datum, target_georef);
    }

    // Find the target resolution based --tr, --mpp, and --ppd if provided. Do
    // the math to convert pixel-per-degree to meter-per-pixel and vice-versa.
    int sum = (!std::isnan(opt.tr)) + (!std::isnan(opt.mpp)) + (!std::isnan(opt.ppd));
    if (sum >= 2){
      vw_throw( ArgumentErr() << "Must specify at most one of the options: --tr, --mpp, --ppd.\n" );
    }

    double radius = target_georef.datum().semi_major_axis();
    if ( !std::isnan(opt.tr) ){ // --tr was set
      if (target_georef.is_projected()) {
        if (std::isnan(opt.mpp)) opt.mpp = opt.tr; // User must have provided be meters per pixel
      }else {
        if (std::isnan(opt.ppd)) opt.ppd = 1.0/opt.tr; // User must have provided degrees per pixel
      }
    }
    
    if (!std::isnan(opt.mpp)){ // Meters per pixel was set
      if (std::isnan(opt.ppd)) opt.ppd = 2.0*M_PI*radius/(360.0*opt.mpp);
    }
    if (!std::isnan(opt.ppd)){ // Pixels per degree was set
      if (std::isnan(opt.mpp)) opt.mpp = 2.0*M_PI*radius/(360.0*opt.ppd);
    }
    
    bool user_provided_resolution = (!std::isnan(opt.ppd));
    bool     calc_target_res = !user_provided_resolution;
    Vector2i image_size      = vw::file_image_size(opt.image_file);
    BBox2    cam_box;
    calc_target_geom(// Inputs
                     calc_target_res, image_size, camera_model,
                     dem, dem_georef, datum_dem,
                     // Outputs
                     opt, cam_box, target_georef);

    vw_out() << "Projected space bounding box: " << cam_box << std::endl;

    // Compute output image size in pixels using bounding box in output projected space
    BBox2i target_image_size = target_georef.point_to_pixel_bbox( cam_box );

    vw_out() << "Image box: " << target_image_size << std::endl;
    
    // Very important note: this box may be in the middle of the
    // image.  However, the virtual image we create with
    // transform_nodata() below is assumed to start at (0, 0), and in
    // target_georef we assume the same thing. Hence, its width and
    // height are going to be the max values of target_image_size.
    // There is no performance hit here, since that potentially huge
    // image is never actually realized, we crop it as seen below
    // before finding its pixels. This could be made less confusing.
    int virtual_image_width  = target_image_size.max().x();
    int virtual_image_height = target_image_size.max().y();

    // Shrink output image BB if an output image BB was passed in
    GeoReference croppedGeoRef  = target_georef;
    BBox2i       croppedImageBB = target_image_size;
    if ( opt.target_pixelwin != BBox2() ) {
      // Replace with passed in bounding box
      croppedImageBB = opt.target_pixelwin;

      // Update output georeference to match the reduced image size
      croppedGeoRef = vw::cartography::crop(target_georef, croppedImageBB);
    }
    //vw_out() << "croppedImageBB = " << croppedImageBB << std::endl;
    //vw_out() << "\nCROPPED georeference:\n"        << croppedGeoRef << std::endl;

    // Important: Don't modify the line below, we count on it in mapproject.in.
    vw_out() << "Output image size:\n";
    vw_out() << "(width: " << virtual_image_width
             << " height: " << virtual_image_height << ")" << std::endl;

    if (opt.isQuery){ // Quit before we do any image work
      vw_out() << "Query finished, exiting mapproject tool.\n";
      return 0;
    }

    // For certain pinhole camera models the reverse check can make map projection very slow,
    // so we disable it here.  The check is very important for computing the bounding box safely
    // but we don't really need it when projecting the pixels back in to the camera.
    boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
                boost::dynamic_pointer_cast<vw::camera::PinholeModel>(camera_model);
    if (pinhole_ptr)
      pinhole_ptr->set_do_point_to_pixel_check(false);

    // Determine the pixel type of the input image
    boost::shared_ptr<DiskImageResource> image_rsrc = vw::DiskImageResourcePtr(opt.image_file);
    ImageFormat image_fmt = image_rsrc->format();
    const int num_input_channels = num_channels(image_fmt.pixel_format);

    // Prepare output directory
    vw::create_out_dir(opt.output_file);

    // Redirect to the correctly typed function to perform the actual map projection.
    // - Must correspond to the type of the input image.
    if (image_fmt.pixel_format == VW_PIXEL_RGB) {

      // We can't just use float for everything or the output will be cast
      //  into the -1 to 1 range which is probably not desired.
      // - Always use an alpha channel with RGB images.
      switch(image_fmt.channel_type) {
      case VW_CHANNEL_UINT8:
        project_image_alpha_pick_transform<PixelRGBA<uint8> >(opt, dem_georef, target_georef,
                                                              croppedGeoRef, image_size, 
                                                              Vector2i(virtual_image_width,
                                                                       virtual_image_height),
                                                              croppedImageBB, camera_model);
        break;
      case VW_CHANNEL_INT16:
        project_image_alpha_pick_transform<PixelRGBA<int16> >(opt, dem_georef, target_georef,
                                                              croppedGeoRef, image_size, 
                                                              Vector2i(virtual_image_width,
                                                                       virtual_image_height),
                                                              croppedImageBB, camera_model);
        break;
      case VW_CHANNEL_UINT16:
        project_image_alpha_pick_transform<PixelRGBA<uint16> >(opt, dem_georef, target_georef,
                                                               croppedGeoRef, image_size, 
                                                               Vector2i(virtual_image_width,
                                                                        virtual_image_height),
                                                               croppedImageBB, camera_model);
        break;
      default:
        project_image_alpha_pick_transform<PixelRGBA<float32> >(opt, dem_georef, target_georef,
                                                                croppedGeoRef, image_size, 
                                                                Vector2i(virtual_image_width,
                                                                         virtual_image_height),
                                                                croppedImageBB, camera_model);
        break;
      };
      
    } else {
      // If the input image is not RGB, only single channel images are supported.
      if (num_input_channels != 1 || image_fmt.planes != 1)
        vw_throw( ArgumentErr() << "Input images must be single channel or RGB!\n" );
      // This will cast to float but will not rescale the pixel values.
      project_image_nodata_pick_transform<float>(opt, dem_georef, target_georef, croppedGeoRef,
                                                 image_size, 
                           Vector2i(virtual_image_width, virtual_image_height),
                           croppedImageBB, camera_model);
    } 
    // Done map projecting!

  } ASP_STANDARD_CATCHES;

  return 0;
}




