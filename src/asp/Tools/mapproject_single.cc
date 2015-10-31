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

#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Camera/DG_XML.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef PixelMask<float> PMaskT;

struct Options : asp::BaseOptions {
  // Input
  std::string dem_file, image_file, camera_model_file, output_file, stereo_session,
    bundle_adjust_prefix;
  bool isQuery;

  // Settings
  std::string target_srs_string;
  double nodata_value, target_resolution, mpp, ppd;
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
    ("tr",               po::value(&opt.target_resolution)->default_value(NaN),
     "Set the output file resolution in target georeferenced units per pixel.")
    ("mpp",              po::value(&opt.mpp)->default_value(NaN),
     "Set the output file resolution in meters per pixel.")
    ("ppd",              po::value(&opt.ppd)->default_value(NaN),
     "Set the output file resolution in pixels per degree.")
    ("query-projection", po::bool_switch(&opt.isQuery)->default_value(false),
      "Just display the computed projection information without actually doing the projection.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Choose 'rpc' if it is desired to later do stereo with the 'dg' session. [options: pinhole isis rpc]")
    ("t_projwin",        po::value(&opt.target_projwin),
     "Limit the map-projected image to this region, with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.")
    ("t_pixelwin",       po::value(&opt.target_pixelwin),
     "Limit the map-projected image to this region, with the corners given in pixels (xmin ymin xmax ymax). Max is exclusive.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem",          po::value(&opt.dem_file))
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_model_file))
    ("output-file" , po::value(&opt.output_file));

  po::positional_options_description positional_desc;
  positional_desc.add("dem",         1);
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-file", 1);

  std::string usage("[options] <dem> <camera-image> <camera-model> <output>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( !vm.count("dem") || !vm.count("camera-image") || !vm.count("camera-model") )
    vw_throw( ArgumentErr() << usage << general_options );

  // We support map-projecting using the DG camera model, however, these images
  // cannot be used later to do stereo, as that process expects the images
  // to be map-projected using the RPC model.
  if (boost::to_lower_copy(opt.stereo_session) == "dg"){
    vw_out(WarningMessage) << "Images map-projected using the 'dg' camera model cannot be used later to run stereo with the 'dg' session. If that is desired, please specify here the 'rpc' camera model instead.\n";
  }

  if ( boost::iends_with(boost::to_lower_copy(opt.camera_model_file), ".xml") &&
       opt.stereo_session == "" ){
    opt.stereo_session = "rpc";
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
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
  keywords["CAMERA_MODEL_TYPE" ]    = session_type;
  std::string prefix = asp::stereo_settings().bundle_adjust_prefix;;
  if (prefix == "") prefix = "NONE"; // to save the field, need to make it non-empty
  keywords["BUNDLE_ADJUST_PREFIX" ] = prefix;
  keywords["DEM_FILE" ]             = opt.dem_file;

  bool has_georef = true;

  // ISIS is not thread safe so we must switch out base on what the session is.
  vw_out() << "Writing: " << filename << "\n";
  if ( session_type == "isis" ) {
    asp::write_gdal_image(filename, image.impl(), has_georef, georef,
                          has_nodata, nodata_val, opt, tpc, keywords);
  } else {
    asp::block_write_gdal_image(filename, image.impl(), has_georef, georef,
                                has_nodata, nodata_val, opt, tpc, keywords);
  }

}

/// Compute which camera pixel observes a DEM pixel.
Vector2 demPixToCamPix(Vector2i const& dem_pixel,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<PMaskT> const& dem,
                      GeoReference const &dem_georef)
{
  Vector2 lonlat = dem_georef.point_to_lonlat(dem_georef.pixel_to_point(dem_pixel));
  //vw_out() << "lonlat = " << lonlat << std::endl;
  PMaskT height = dem(dem_pixel[0], dem_pixel[1]);
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
                                            ImageViewRef<PMaskT> const& dem,
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
                      bool first_pass,
                      bool calc_target_res,
                      Vector2i const& image_size,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<PMaskT> const& dem,
                      GeoReference dem_georef, // make copy on purpose
                      BBox2  dem_box,         // make copy on purpose
                      // Outputs
                      Options & opt, BBox2 & cam_box, GeoReference & target_georef){

  // Find the camera bbox and the target resolution unless user-supplied.
  // - This call returns the bounding box of the camera view on the ground.
  // - The bounding box is in units defined by dem_georef and might not be meters.
  // - auto_res is an estimate of the ground resolution visible by the camera.
  //   This is in a unit defined by dem_georef and also might not be meters.
  // - This call WILL intersect pixels outside the dem valid area!
  // - TODO: Modify this function to optionally disable intersection outside the DEM
  float auto_res;
  cam_box = camera_bbox(dem, dem_georef, camera_model,
                        image_size.x(), image_size.y(), auto_res);

  //vw_out() << "\ncam_box calc1:\n" << cam_box << std::endl;

  if (first_pass) {
    // Project the four corners of the DEM into the camera; if any of them intersect,
    //  expand the bbox to include their coordinates
    // - This is not valid in the second pass because the dem_georef is no longer accurate!
    expandBboxToContainCornerIntersections(camera_model, dem, dem_georef, image_size, cam_box);
  }

  //vw_out() << "\ncam_box calc dem expanded:\n" << cam_box << std::endl;

  if (first_pass){
    // Convert bounding box from dem_georef coordinate system to
    //  target_georef coordinate system
    cam_box = target_georef.lonlat_to_point_bbox
      (dem_georef.point_to_lonlat_bbox(cam_box));

    //vw_out() << "\ncam_box calc trans:\n" << cam_box << std::endl;
  }

  // Use auto-calculated ground resolution if that option was selected
  if (calc_target_res)
    opt.target_resolution = auto_res;

  // If an image bounding box (projected coordinates) was passed in,
  // override the camera's view on the ground with the custom box.
  // The user needs to know the georeference projected coordinate
  // system (using the query command) to do this.
  if ( opt.target_projwin != BBox2() ) {
    cam_box = opt.target_projwin;
    if ( cam_box.min().y() > cam_box.max().y() )
      std::swap( cam_box.min().y(), cam_box.max().y() );
    cam_box.max().x() -= opt.target_resolution; //TODO: What are these adjustments?
    cam_box.min().y() += opt.target_resolution;
  }

  if ( opt.target_projwin == BBox2() ) {
    // Ensure the camera box does not extend beyond the DEM box.
    // Apply this only if the user did not explicitly request
    // a custom proj win.
    dem_box.max().x() -= opt.target_resolution; //TODO: What are these adjustments?
    dem_box.min().y() += opt.target_resolution;
    cam_box.crop(dem_box);
  }

  // In principle the corners of the projection box can be
  // arbitrary.  However, we will force them to be at integer
  // multiples of pixel dimensions. This is needed if we want to do
  // tiling, that is break the DEM into tiles, project on individual
  // tiles, and then combine the tiles nicely without seams into a
  // single projected image. The tiling solution provides a nice
  // speedup when dealing with ISIS images, when projection runs
  // only with one thread.
  double s = opt.target_resolution;
  int min_x         = (int)round(cam_box.min().x() / s);
  int min_y         = (int)round(cam_box.min().y() / s);
  int output_width  = (int)round(cam_box.width()   / s);
  int output_height = (int)round(cam_box.height()  / s);
  cam_box = s * BBox2(min_x, min_y, output_width, output_height);

  //vw_out() << "\ncam_box calc scaled:\n" << cam_box << std::endl;


  // This transform is from pixel to projected coordinates
  Matrix3x3 T = target_georef.transform();
  // This polarity checking is to make sure the output has been
  // transposed after going through reprojection. Normally this is
  // the case. Yet with grid data from GMT, it is not.
  if ( T(0,0) < 0 ) // If X coefficient of affine transform is negative (cols go opposite direction from projected x coords)
    T(0,2) = cam_box.max().x();  // The maximum projected X coordinate is the starting offset
  else
    T(0,2) = cam_box.min().x(); // The minimum projected X coordinate is the starting offset
  T(0,0) =  opt.target_resolution;  // Set col/X conversion to meters per pixel
  T(1,1) = -opt.target_resolution;  // Set row/Y conversion to meters per pixel with a vertical flip (increasing row = down in Y)
  T(1,2) = cam_box.max().y();       // The maximum projected Y coordinate is the starting offset
  if ( target_georef.pixel_interpretation() ==
       GeoReference::PixelAsArea ) { // Meaning point [0][0] equals location (0.5, 0.5)
    T(0,2) -= 0.5 * opt.target_resolution; // Apply a small shift to the offsets
    T(1,2) += 0.5 * opt.target_resolution;
  }
  target_georef.set_transform( T ); // Overwrite the existing transform in target_georef

  return;
}

int main( int argc, char* argv[] ) {

  Options opt;
  try {

    handle_arguments( argc, argv, opt );

    // TODO: Replace this using the new CameraModelLoader functions

    // We create a stereo session where both of the cameras and images
    // are the same, because we want to take advantage of the stereo
    // pipeline's ability to generate camera models for various
    // missions.  Hence, we create two identical camera models, but only one is used.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSessionFactory::create(opt.stereo_session, // in-out
                                                          opt,
                                                          opt.image_file, opt.image_file, // The same file is passed in twice
                                                          opt.camera_model_file,
                                                          opt.camera_model_file,
                                                          opt.output_file,
                                                          opt.dem_file,
                                                          false) ); // Do not allow promotion from normal to map projected session types!!!!
    // TODO: Take care of this in handle_argumensests?
    if ((session->name() == "isis" || session->name() == "isismapisis")
          && opt.output_file.empty() ){
      // The user did not provide an output file. Then the camera
      // information is contained within the image file and what is in
      // the camera file is actually the output file.
      opt.output_file       = opt.camera_model_file;
      opt.camera_model_file = opt.image_file;
    }
    if ( opt.output_file.empty() )
      vw_throw( ArgumentErr() << "Missing output filename.\n" );

    // Initialize a camera model
    boost::shared_ptr<camera::CameraModel> camera_model =
      session->camera_model(opt.image_file, opt.camera_model_file);

    {
      // Safety check that the users are not trying to map project map
      // projected images. This should not be an error as sometimes
      // even raw images have some half-baked georeference attached
      // to them.
      GeoReference dummy_georef;
      bool has_georef = read_georeference( dummy_georef, opt.image_file );
      if (has_georef)
        vw_out(WarningMessage) << "Your input camera image is already map-"
                               << "projected. The expected input is required "
                               << "to be unprojected or raw camera imagery.\n";
    }

    // Load the DEM
    GeoReference dem_georef;
    ImageViewRef<PMaskT> dem;
    bool has_georef = read_georeference(dem_georef, opt.dem_file);
    if (!has_georef)
      vw_throw( ArgumentErr() << "There is no georeference information in: " << opt.dem_file << ".\n" );

    boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResource::open(opt.dem_file));

    // If we have a nodata value, create a mask.
    DiskImageView<float> dem_disk_image(opt.dem_file);
    if (dem_rsrc->has_nodata_read()){
      dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
    }else{
      dem = pixel_cast<PMaskT>(dem_disk_image);
    }

    // Find the target resolution based on mpp or ppd if provided. Do
    // the math to convert pixel-per-degree to meter-per-pixel and vice-versa.
    int sum = (!std::isnan(opt.target_resolution)) + (!std::isnan(opt.mpp)) + (!std::isnan(opt.ppd));
    if (sum >= 2){
      vw_throw( ArgumentErr() << "Must specify at most one of the options: --tr, --mpp, --ppd.\n" );
    }
    double radius = dem_georef.datum().semi_major_axis();
    if ( !std::isnan(opt.mpp) ){ // Meters per pixel was set
      opt.ppd = 2.0*M_PI*radius/(360.0*opt.mpp);
    }else if ( !std::isnan(opt.ppd) ){ // Pixels per degree was set
      opt.mpp = 2.0*M_PI*radius/(360.0*opt.ppd);
    }
    if ( !std::isnan(opt.ppd) ) { // pixels per degree now available
      if (dem_georef.is_projected()) {
        opt.target_resolution = opt.mpp; // Use units of meters
      } else { // Not projected, GDC coordinates only.
        opt.target_resolution = 1/opt.ppd; // Use units of degrees
                                           // Lat/lon degrees are different so we never want to do this!
      }
    }

    // Read projection. Work out output bounding box in points using original camera model.
    GeoReference target_georef = dem_georef;

    // User specified the proj4 string for the output georeference
    if (opt.target_srs_string != ""){
      bool  have_user_datum = false;
      Datum user_datum;
      asp::set_srs_string(opt.target_srs_string, have_user_datum, user_datum, target_georef);
    }

    //vw_out() << "\n\nDEM georeference:\n"        << dem_georef << std::endl;
    //vw_out() << "\nTARGET georeference:\n"        << target_georef << std::endl;


    // Compute the dem BBox in the output projected space.
    // Here we could have used target_georef.lonlat_to_point_bbox(dem_georef.pixel_to_lonlat_bbox)
    // but that grows the box needlessly big. We will ensure the mapprojected image does
    // not go beyond dem_box.
    cartography::GeoTransform T(dem_georef, target_georef);
    BBox2 dem_box = T.forward_pixel_to_point_bbox(bounding_box(dem));

    // We compute the target_georef and camera box in two passes,
    // first in the DEM coordinate system and we rotate it to target's
    // coordinate system (which makes it grow), and then we tighten it
    // in target's coordinate system.
    bool     calc_target_res = std::isnan(opt.target_resolution);
    Vector2i image_size      = asp::file_image_size(opt.image_file);
    BBox2    cam_box;
    // First pass
    bool first_pass = true;
    calc_target_geom(// Inputs
                     first_pass, calc_target_res, image_size, camera_model,
                     dem, dem_georef, dem_box,
                     // Outputs
                     opt, cam_box, target_georef);
    // target_georef is now in the output coordinate system and location!
    vw_out() << "Calculated initial projected space bounding box: " << cam_box << std::endl;

    //vw_out() << "\nTARGET georeference 2:\n"        << target_georef << std::endl;

    // Second pass
    first_pass = false;

    // Transformed view indexes DEM based on target georeference
    // - Note that the width and height of trans_dem don't make sense!
    ImageViewRef<PMaskT> trans_dem
      = geo_transform(dem, dem_georef, target_georef,
                      ValueEdgeExtension<PMaskT>(PMaskT()),
                      BilinearInterpolation());

    calc_target_geom(// Inputs
                     first_pass, calc_target_res, image_size, camera_model,
                     trans_dem, target_georef, dem_box,
                     // Outputs
                     opt, cam_box, target_georef);

    vw_out() << "Refined projected space bounding box: " << cam_box << std::endl;

    // Compute output image size in pixels using bounding box in output projected space
    BBox2i target_image_size = target_georef.point_to_pixel_bbox( cam_box );

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

    // Important: Don't modify the line below, we count on it in mapproject.in.
    vw_out() << "Output image size:\n";
    vw_out() << "(width: " << virtual_image_width
             << " height: " << virtual_image_height << ")" << std::endl;

    if (opt.isQuery){ // Quit before we do any image work
      vw_out() << "Query finished, exiting mapproject tool.\n";
      return 0;
    }

    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource>
      img_rsrc( DiskImageResource::open( opt.image_file ) );

    // Write the output image. Use the nodata passed in by the user
    // if it is not available in the input file.
    if (img_rsrc->has_nodata_read()) opt.nodata_value = img_rsrc->nodata_read();
    vw::create_out_dir(opt.output_file);
    bool has_img_nodata = true;
    PMaskT nodata_mask = PMaskT(); // invalid value for a PixelMask
    bool call_from_mapproject = true;
    write_parallel_cond
      ( // Write to the output file
       opt.output_file,
       crop( // Apply crop (only happens if --t_pixelwin was specified)
            apply_mask
            ( // Handle nodata
             transform_nodata( // Apply the output from Map2CamTrans
                              create_mask(DiskImageView<float>(img_rsrc),
                                          opt.nodata_value), // Handle nodata
                              Map2CamTrans
                              ( // Converts coordinates in DEM
                                // georeference to camera pixels
                               camera_model.get(), target_georef,
                               dem_georef, opt.dem_file, image_size,
                               call_from_mapproject
                               ),
                              virtual_image_width,
                              virtual_image_height,
                              ValueEdgeExtension<PMaskT>(nodata_mask),
                              BicubicInterpolation(), nodata_mask
                              ),
             opt.nodata_value
             ),
            croppedImageBB
            ),
       croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
       TerminalProgressCallback("","")
       );

  } ASP_STANDARD_CATCHES;

  return 0;
}
