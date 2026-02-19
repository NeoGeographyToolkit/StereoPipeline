// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/MapprojectImage.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Core/DemUtils.h>
#include <asp/Core/CartographyUtils.h>

#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/FileTypes.h>

#include <fstream>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// The pixel type used for the DEM data
typedef PixelMask<float> DemPixelT;

void handle_arguments(int argc, char *argv[], asp::MapprojOptions& opt) {
  po::options_description general_options("");
  double NaN = std::numeric_limits<double>::quiet_NaN();
  general_options.add_options()
    ("nodata-value",     po::value(&opt.nodata_value)->default_value(-32768),
     "No-data value to use unless specified in the input image.")
    ("t_srs",            po::value(&opt.target_srs_string)->default_value(""),
     "Specify the output projection as a GDAL projection string (WKT, GeoJSON, or PROJ). If not provided, use the one from the DEM.")
    ("tr",              po::value(&opt.tr)->default_value(NaN),
     "Set the output file resolution (ground sample distance) in target georeferenced "
     "units per pixel. This may be in meters or degrees, depending on the projection. The "
     "center of each output pixel will be at integer multiples of this grid size, unless "
     "--gdal-tap is set.")
    ("mpp",              po::value(&opt.mpp)->default_value(NaN),
     "Set the output file resolution in meters per pixel.")
    ("ppd",              po::value(&opt.ppd)->default_value(NaN),
     "Set the output file resolution in pixels per degree.")
    ("datum-offset",     po::value(&opt.datum_offset)->default_value(0),
     "When projecting to a datum instead of a DEM, use this elevation in meters from the datum.")
    ("query-projection", po::bool_switch(&opt.query_projection)->default_value(false),
     "Display the computed projection information, estimated ground sample "
     "distance, and projected bounding box. Save the projection as a WKT "
     "file named <output image>.wkt. Quit afterwards.")
    ("session-type,t",      po::value(&opt.stereo_session),
     "Select the stereo session type to use for processing. Usually the program can select this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("t_projwin",        po::value(&opt.target_projwin),
     "Limit the mapprojected image to this region, with the corners given in georeferenced "
     "coordinates (xmin ymin xmax ymax). Max is exclusive, unless --gdal-tap is set.")
    ("t_pixelwin",       po::value(&opt.target_pixelwin),
     "Limit the mapprojected image to this region, with the corners given in pixels (xmin ymin xmax ymax). Max is exclusive.")
    ("gdal-tap", po::bool_switch(&opt.gdal_tap)->default_value(false),
     "Ensure that the output image bounds (as printed by gdalinfo) are integer multiples "
     "of the grid size (as set with --tr). When --t_projwin is set and its entries are "
     "integer multiples of the grid size, that precise extent will be produced on output. "
     "This functions as the GDAL -tap option.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    ("ref-map", po::value(&opt.ref_map)->default_value(""),
     "Read the projection and grid size from this mapprojected image.")
    ("ot",  po::value(&opt.output_type)->default_value("Float32"), 
     "Output data type, when the input is single channel. Supported types: Byte, UInt16, "
     "Int16, UInt32, Int32, Float32. If the output type is a kind of integer, values are "
     "rounded and then clamped to the limits of that type. This option will be ignored for "
     "multi-channel images, when the output type is set to be the same as the input type.")
    ("nearest-neighbor", po::bool_switch(&opt.nearest_neighbor)->default_value(false),
     "Use nearest neighbor interpolation.  Useful for classification images.")
    ("mo",  po::value(&opt.metadata)->default_value(""), "Write metadata to the output file. Provide as a string in quotes if more than one item, separated by a space, such as 'VAR1=VALUE1 VAR2=VALUE2'. Neither the variable names nor the values should contain spaces.")
    ("no-geoheader-info", po::bool_switch(&opt.noGeoHeaderInfo)->default_value(false),
     "Do not write metadata information in the geoheader. See the doc for more info.")
    ("query-pixel", po::value(&opt.query_pixel)->default_value(vw::Vector2(NaN, NaN)),
     "Trace a ray from this input image pixel (values start from 0) to the ground. "
     "Print the intersection point with the DEM as lon, lat, height, then "
     "as DEM column, row, height. Quit afterwards.")
    ("aster-use-csm",
     po::bool_switch(&opt.aster_use_csm)->default_value(false)->implicit_value(true),
     "Use the CSM model with ASTER cameras (-t aster).")
    ("parse-options", po::bool_switch(&opt.parseOptions)->default_value(false),
     "Parse the options and print the results. Used by the mapproject script.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("dem",          po::value(&opt.dem_file))
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file))
    ("output-image", po::value(&opt.output_file));

  po::positional_options_description positional_desc;
  positional_desc.add("dem",          1);
  positional_desc.add("camera-image", 1);
  positional_desc.add("camera-model", 1);
  positional_desc.add("output-image", 1);

  std::string usage("[options] <dem> <camera-image> <camera-model> <output-image>\nInstead of the DEM file, a datum can be provided, such as\nWGS84, NAD83, NAD27, D_MOON, D_MARS, and MOLA.");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (!vm.count("dem") || !vm.count("camera-image") || !vm.count("camera-model"))
    vw_throw(ArgumentErr() << "Not all of the input DEM, image, and camera were specified.\n"
             << usage << general_options);

  // If exactly three files were passed in, the last one must be the output file and the image file
  // must contain the camera model.
  if (!vm.count("output-image") && vm.count("camera-model")) {
    opt.output_file = opt.camera_file;
    opt.camera_file = "";
  }

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr()
             << "The value of --t_srs is empty. Then it must not be set at all.\n");

  if (vw::has_cam_extension(opt.output_file))
    vw_throw(ArgumentErr() << "The output file is a camera. Check your inputs.\n");

  if (opt.parseOptions) {
    // For the benefit of mapproject
    vw_out() << "dem," << opt.dem_file << "\n";
    vw_out() << "image," << opt.image_file << "\n";
    vw_out() << "camera," << opt.camera_file << "\n";
    vw_out() << "output_file," << opt.output_file << "\n";
    exit(0);
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
  asp::stereo_settings().aster_use_csm = opt.aster_use_csm;

  if (!opt.ref_map.empty()) {
    // Ensure that --ref-map and --t_srs are not both set
    if (!opt.target_srs_string.empty())
      vw_throw(ArgumentErr() << "Cannot specify both --ref-map and --t_srs.\n");
    if (!std::isnan(opt.tr))
      vw_throw(ArgumentErr() << "Cannot specify both --ref-map and --tr.\n");

    // Read the georeference from the reference map and get its wkt
    GeoReference ref_map_georef;
    bool has_georef = vw::cartography::read_georeference(ref_map_georef, opt.ref_map);
    if (!has_georef)
      vw_throw(ArgumentErr() << "The reference map has no georeference.\n");
    opt.target_srs_string = ref_map_georef.get_wkt();
    
    // Set opt.tr
    opt.tr = ref_map_georef.transform()(0, 0);
  }

  if (fs::path(opt.dem_file).extension() != "") {
    // A path to a real DEM file was provided, load it!
    GeoReference dem_georef;
    bool has_georef = vw::cartography::read_georeference(dem_georef, opt.dem_file);
    if (!has_georef)
      vw_throw(ArgumentErr() << "There is no georeference information in: "
                             << opt.dem_file << ".\n");

    // Make sure the user did not actually pass in an RGB image
    boost::shared_ptr<DiskImageResource> dem_rsrc = vw::DiskImageResourcePtr(opt.dem_file);
    ImageFormat dem_fmt = dem_rsrc->format();
    const int num_input_channels = num_channels(dem_fmt.pixel_format);
    if (num_input_channels > 2)
      vw_throw(ArgumentErr() << "Too many channels in: " << opt.dem_file << ".\n");

    // Store the datum from the DEM
    // TODO (oalexan1): Storing the datum name is fragile.
    asp::stereo_settings().datum = dem_georef.datum().name();
  }

  return;
}

/// Compute which camera pixel observes a DEM pixel.
Vector2 demPixToCamPix(Vector2i const& dem_pixel,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<DemPixelT> const& dem,
                      GeoReference const &dem_georef) {
  Vector2 lonlat = dem_georef.point_to_lonlat(dem_georef.pixel_to_point(dem_pixel));
  DemPixelT height = dem(dem_pixel[0], dem_pixel[1]);
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian
                    (Vector3(lonlat[0], lonlat[1], height.child()));
  // Throws if the projection fails ???
  Vector2i camera_pixel = camera_model->point_to_pixel(xyz);
  return camera_pixel;
}

/// Expand the ground BBox to contain all the corners of the DEM if they intersect the camera.
/// - TODO: This method still does not guarantee all points will be included in the bbox.
/// - TODO: This should probably take pixel validity into account!
void expandBboxToContainCornerIntersections(vw::CamPtr camera_model,
                                            ImageViewRef<DemPixelT> const& dem,
                                            GeoReference const &dem_georef,
                                            Vector2i const& image_size,
                                            BBox2 & bbox_on_ground) {
  // Each of the corners of the DEM
  std::vector<Vector2> dem_pixel_list(4);
  dem_pixel_list[0] = Vector2(0,            0);
  dem_pixel_list[1] = Vector2(dem.cols()-1, 0);
  dem_pixel_list[2] = Vector2(dem.cols()-1, dem.rows()-1);
  dem_pixel_list[3] = Vector2(0,            dem.rows()-1);

  for (int i=0; i<4; i++) {
    try{
      // Project the DEM corner into the input image
      Vector2 dem_pixel = dem_pixel_list[i];
      Vector2 cam_pixel = demPixToCamPix(dem_pixel, camera_model, dem, dem_georef);

      // Get the point on the ground
      Vector2 groundLoc = dem_georef.pixel_to_point(dem_pixel);

      // If there was in intersection
      if (cam_pixel.x() >= 0 && cam_pixel.y() > 0 &&
          cam_pixel.x() < image_size.x() && cam_pixel.y() < image_size.y()) {
        bbox_on_ground.grow(groundLoc);
      }
    } catch(...) {} // projection failed
  } // End loop through DEM points

}

/// Compute output georeference to use
void calc_target_geom(// Inputs
                      bool calc_target_res,
                      Vector2i const& image_size,
                      boost::shared_ptr<camera::CameraModel> const& camera_model,
                      ImageViewRef<DemPixelT> const& dem,
                      GeoReference const& dem_georef,
                      bool proj_on_datum,
                      asp::MapprojOptions const & opt,
                      // Outputs
                      BBox2 & cam_box, GeoReference & target_georef) {

  // Find the camera bbox and the target resolution unless user-supplied.
  // - This call returns the bounding box of the camera view on the ground.
  // - The bounding box is in units defined by dem_georef and might not be meters.
  // - auto_res is an estimate of the ground resolution visible by the camera.
  //   This is in a unit defined by dem_georef and also might not be meters.
  // - This call WILL intersect pixels outside the dem valid area!
  // - TODO: Modify this function to optionally disable intersection outside the DEM
  float auto_res = -1.0;  // will be updated
  bool quick = proj_on_datum; // Quick mode when no actual DEM
  try {
    cam_box = camera_bbox(dem, dem_georef, target_georef, camera_model,
                          image_size.x(), image_size.y(), auto_res, quick);
  } catch (std::exception const& e) {
    if (opt.target_projwin == BBox2() || calc_target_res) {
      vw_throw(ArgumentErr()
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
      // Use units of degrees. Lat/lon degrees are different so, this should be avoided.
      current_resolution = 1/opt.ppd;
    }
  }

  // Important sanity checks
  if (current_resolution < 0.001 * auto_res)
        vw::vw_throw(vw::ArgumentErr() 
          << "The user-set grid size (option --tr) is so small that likely it is in degrees, "
          << "while meters are expected.\n");
  // For a lesser discrepancy, just print a warning.
  if (current_resolution < 0.1 * auto_res)
    vw_out(vw::WarningMessage) 
          << "The user-provided grid size (--tr) is " << current_resolution << ", "
          << "which is smaller than the auto-estimated grid size of " 
          << auto_res << ". Likely the resulting mapprojected image will not be accurate.\n";

  // If an image bounding box (projected coordinates) was passed in,
  // override the camera's view on the ground with the custom box.
  // The user needs to know the georeference projected coordinate
  // system (using the query command) to do this.
  if (opt.target_projwin != BBox2()) {
    cam_box = opt.target_projwin;
    if (cam_box.min().y() > cam_box.max().y())
      std::swap(cam_box.min().y(), cam_box.max().y());
    // Make the maximum non-exclusive, unless --gdal-tap is set.
    if (!opt.gdal_tap) {
      cam_box.max().x() -= current_resolution;
      cam_box.max().y() -= current_resolution;
    }
  }

  // In principle the corners of the projection box can be arbitrary.  However,
  // we will force them to be at integer multiples of pixel size. This is needed
  // if we want to do tiling, that is break the DEM into tiles, project on
  // individual tiles, and then combine the tiles nicely without seams into a
  // single projected image. The tiling solution provides a nice speedup when
  // dealing with ISIS images, when projection runs only with one thread.
  // This is adjusted below if --gdal-tap is set.
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
  if (T(0, 0) < 0) // If X coefficient of affine transform is negative (cols go opposite direction from projected x coords)
    T(0,2) = cam_box.max().x();  // The maximum projected X coordinate is the starting offset
  else
    T(0,2) = cam_box.min().x(); // The minimum projected X coordinate is the starting offset
  T(0,0) = current_resolution;  // Set col/X conversion to meters per pixel
  T(1,1) = -current_resolution;  // Set row/Y conversion to meters per pixel with a vertical flip (increasing row = down in Y)
  T(1,2) = cam_box.max().y();       // The maximum projected Y coordinate is the starting offset
  
  // The default behavior is that center of pixel is at integer multiples of the
  // pixel size. If --gdal-tap is set, emulate the GDAL -tap option, so corners
  // of pixels are at integer multiples of the grid.
  double delta = 0.0;
  if (target_georef.pixel_interpretation() == GeoReference::PixelAsArea &&
      !opt.gdal_tap)
    delta = 0.5 * current_resolution; // center of pixel (0, 0) is (0.5, 0.5)

  // Apply this behavior to the transform
  T(0,2) -= delta; 
  T(1,2) += delta;
  target_georef.set_transform(T);

  // Compute output image size in pixels using bounding box in output projected space
  // Do not use point_to_pixel_bbox() as that one exaggerates the size of the box.
  // TODO(oalexan1): Below, target_image_size must be of type BBox2. This will break a lot
  // of tests though.
  BBox2i target_image_size;
  vw::Vector2 d(delta, delta);
  target_image_size.grow(target_georef.point_to_pixel(cam_box.min() + d));
  target_image_size.grow(target_georef.point_to_pixel(cam_box.max() + d));
  target_image_size.min() = round(target_image_size.min());
  target_image_size.max() = round(target_image_size.max());

  // Last adjustment, to ensure 0 0 is always in the box corner
  target_georef = crop(target_georef, target_image_size.min().x(),
                       target_image_size.min().y());

  return;
}

// Automatic projection determination
void calcAutoProj(GeoReference const& dem_georef,
                  vw::CamPtr camera_model,
                  Vector2i const& image_size,
                  bool proj_on_datum,
                  vw::ImageViewRef<DemPixelT> const& dem,
                  GeoReference& target_georef) {

  float auto_res = false; 
  bool quick = proj_on_datum; // Quick mode when no actual DEM
  std::vector<vw::Vector3> *coords = NULL;
  int num_samples = 100; // enough for projection center determination
  BBox2 cam_box = camera_bbox(dem, dem_georef, target_georef, camera_model,
                    image_size.x(), image_size.y(), auto_res, quick, coords, num_samples);
  
  BBox2 ll_box = target_georef.point_to_lonlat_bbox(cam_box);
  if (ll_box.empty())
    vw_throw(ArgumentErr() << "Failed to determine the output projection.\n");

  // The projection center is the box center
  vw::Vector2 lonlat_ctr = (ll_box.min() + ll_box.max()) / 2.0;

  asp::setAutoProj(lonlat_ctr.y(), lonlat_ctr.x(), target_georef);
}                               

int main(int argc, char* argv[]) {

  asp::MapprojOptions opt;
  try {
    handle_arguments(argc, argv, opt);

    // TODO: Replace this using the new CameraModelLoader functions. But those
    // may not have the session guessing logic.
    // We create a stereo session where both of the cameras and images
    // are the same, because we want to take advantage of the stereo
    // pipeline's ability to generate camera models for various
    // missions.  Hence, we create two identical camera models, but only one is used.
    asp::SessionPtr session(asp::StereoSessionFactory::create
                       (opt.stereo_session, // in-out
                        opt,
                        opt.image_file, opt.image_file, // The same file is passed in twice
                        opt.camera_file, opt.camera_file,
                        opt.output_file,
                        "", // Do not use a DEM to not make the session mapprojected
                        false)); // Do not allow promotion from normal to map projected session

    if (opt.output_file.empty())
      vw_throw(ArgumentErr() << "Missing output filename.\n");

    // Additional checks once the stereo session is determined.

    if (opt.stereo_session == "perusat")
      vw_out(WarningMessage) << "Images mapprojected using the '" << opt.stereo_session
                             << "' camera model cannot be used later for stereo. "
                             << "If that is desired, please run mapproject with "
                             << "'-t rpc' and a camera file having an RPC model.\n";

    // If nothing else works
    // TODO(oalexan1): Likely StereoSessionFactory already have this logic.
    if (boost::iends_with(boost::to_lower_copy(opt.camera_file), ".xml") &&
         opt.stereo_session == "")
      opt.stereo_session = "rpc";

    // Initialize the camera model
    opt.camera_model = session->camera_model(opt.image_file, opt.camera_file);

    opt.multithreaded_model = session->supports_multi_threading();

    {
      // Safety check that the users are not trying to map project map
      // projected images. This should not be an error as sometimes
      // even raw images have some half-baked georeference attached to them.
      GeoReference dummy_georef;
      bool has_georef = vw::cartography::read_georeference(dummy_georef, opt.image_file);
      if (has_georef)
        vw_out(WarningMessage) << "Your input camera image is already map-"
                               << "projected. The expected input is required "
                               << "to be unprojected or raw camera imagery.\n";
    }

    // If the query pixel option was set, run the query and exit
    if (!std::isnan(opt.query_pixel[0]) && !std::isnan(opt.query_pixel[1])) {
      asp::queryPixel(opt.dem_file, opt.camera_model, opt.query_pixel);
      return 0;
    }

    // Load the DEM
    bool proj_on_datum = false;
    GeoReference dem_georef;
    ImageViewRef<DemPixelT> dem;
    if (fs::path(opt.dem_file).extension() != "") {
      // A path to a real DEM file was provided, load it!

      bool has_georef = vw::cartography::read_georeference(dem_georef, opt.dem_file);
      if (!has_georef)
        vw_throw(ArgumentErr() << "There is no georeference information in: "
                  << opt.dem_file << ".\n");

      boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(opt.dem_file));

      // If we have a nodata value, create a mask.
      DiskImageView<float> dem_disk_image(opt.dem_file);
      if (dem_rsrc->has_nodata_read()) {
        dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
      } else {
        dem = pixel_cast<DemPixelT>(dem_disk_image);
      }
    } else {
      // Projecting to a datum instead of a DEM
      proj_on_datum = true;
      std::string datum_name = opt.dem_file;

      // Use the camera center to determine whether to center the fake DEM on 0 or 180.
      Vector3 cam_ctr = opt.camera_model->camera_center(Vector2());
      Vector3 llr_camera_loc = cartography::XYZtoLonLatRadEstimateFunctor::apply(cam_ctr);
      float lonstart = 0;
      if ((llr_camera_loc[0] < 0) && (llr_camera_loc[0] > -180))
        lonstart = -180;
      dem_georef = GeoReference(Datum(datum_name),
                                // Need adjustments to work at boundaries
                                vw::Matrix3x3(1,  0, lonstart-0.5,
                                              0, -1, 90+0.5,
                                              0,  0,  1));
      dem = constant_view(PixelMask<float>(opt.datum_offset), 360.0, 180.0);
      vw_out() << "\t--> Using flat datum: " << datum_name << " as elevation model.\n";
    }
    // Finished setting up the datum

    Vector2i image_size = vw::file_image_size(opt.image_file);

    // Read projection. Work out output bounding box in points using original camera model.
    GeoReference target_georef = dem_georef;

    if ((opt.target_srs_string.empty() && !target_georef.is_projected()) ||
        boost::to_lower_copy(opt.target_srs_string) == "auto") {
       // Automatic projection determination
       calcAutoProj(dem_georef, opt.camera_model, image_size, proj_on_datum, dem, 
                    target_georef); // output
    } else if (!opt.target_srs_string.empty()) {
      // Use specified proj4 string for the output georeference
      bool have_user_datum = false;
      Datum user_datum;
      vw::cartography::set_srs_string(opt.target_srs_string, have_user_datum, user_datum,
                          target_georef);
    }

    // The user datum and DEM datum must agree
    bool warn_only = false;
    vw::checkDatumConsistency(dem_georef.datum(), target_georef.datum(), warn_only);

    // Find the target resolution based --tr, --mpp, and --ppd if provided. Do
    // the math to convert pixel-per-degree to meter-per-pixel and vice-versa.
    int sum = (!std::isnan(opt.tr)) + (!std::isnan(opt.mpp)) + (!std::isnan(opt.ppd));
    if (sum >= 2)
      vw::vw_throw(vw::ArgumentErr()
               << "Must specify at most one of the options: --tr, --mpp, --ppd.\n");

    double radius = target_georef.datum().semi_major_axis();
    if (!std::isnan(opt.tr)) { // --tr was set
      if (target_georef.is_projected()) {
        if (std::isnan(opt.mpp)) 
          opt.mpp = opt.tr; // User must have provided be m / pixel
      } else {
        if (std::isnan(opt.ppd)) 
          opt.ppd = 1.0/opt.tr; // User must have provided degrees per pixel
      }
    }

    if (!std::isnan(opt.mpp) && std::isnan(opt.ppd)) // Meters per pixel was set
      opt.ppd = 2.0*M_PI*radius/(360.0*opt.mpp);
    if (!std::isnan(opt.ppd) && std::isnan(opt.mpp)) // Pixels per degree was set
      opt.mpp = 2.0*M_PI*radius/(360.0*opt.ppd);

    bool user_provided_resolution = (!std::isnan(opt.ppd));
    bool     calc_target_res = !user_provided_resolution;
    BBox2    cam_box;
    calc_target_geom(// Inputs
                     calc_target_res, image_size, opt.camera_model,
                     dem, dem_georef, proj_on_datum, opt,
                     // Outputs
                     cam_box, target_georef);

    // Set a high precision, as the numbers can come out big for UTM
    vw_out() << std::setprecision(17) << "Projected space bounding box: " << cam_box << "\n";

    // Compute output image size in pixels using bounding box in output projected space
    BBox2i target_image_size = target_georef.point_to_pixel_bbox(cam_box);

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
    GeoReference crop_georef = target_georef;
    BBox2i crop_bbox = target_image_size;
    if (opt.target_pixelwin != BBox2()) {
      // Replace with passed-in bounding box
      crop_bbox = opt.target_pixelwin;

      // Update output georeference to match the reduced image size
      crop_georef = vw::cartography::crop(target_georef, crop_bbox);
    }

    // Print an explanation for a potential problem.
    if (virtual_image_width <= 0 || virtual_image_height <= 0)
      vw_throw(ArgumentErr() << "Computed output image size is not positive. "
                << "This can happen if the projection is in meters while the "
                << "grid size is either in degrees or too large for the given input.\n");

    // Form the lon-lat bounding box of the output image. This helps with
    // geotransform operations and should be done any time a georef is modified.
    BBox2 image_bbox(0, 0, virtual_image_width, virtual_image_height);
    crop_georef.ll_box_from_pix_box(image_bbox);

    if (opt.query_projection) {
      // Save the projection to a WKT file named <output>.wkt
      std::string wkt_file = opt.output_file + ".wkt";
      std::ofstream ofs(wkt_file.c_str());
      if (!ofs.good())
        vw_throw(ArgumentErr() << "Failed to open for writing: "
                 << wkt_file << "\n");
      ofs << crop_georef.get_wkt() << "\n";
      ofs.close();

      // Structured output for the Python mapproject wrapper to parse.
      // The comma separator must be in sync with the Python side.
      vw_out() << std::setprecision(17)
               << "Query results:\n"
               << "image_width," << virtual_image_width << "\n"
               << "image_height," << virtual_image_height << "\n"
               << "pixel_size," << crop_georef.transform()(0, 0) << "\n"
               << "proj_box_xmin," << cam_box.min().x() << "\n"
               << "proj_box_ymin," << cam_box.min().y() << "\n"
               << "proj_box_xmax," << cam_box.max().x() << "\n"
               << "proj_box_ymax," << cam_box.max().y() << "\n"
               << "projection_wkt_file," << wkt_file << "\n";

      return 0;
    }

    // For certain pinhole camera models the reverse check can make map
    // projection very slow, so we disable it here.  The check is very important
    // for computing the bounding box safely but we don't really need it when
    // projecting the pixels back in to the camera.
    auto pinhole_ptr = boost::dynamic_pointer_cast<vw::camera::PinholeModel>(opt.camera_model);
    if (pinhole_ptr)
      pinhole_ptr->set_do_point_to_pixel_check(false);

    // Project the image depending on image format.
    project_image(opt, dem_georef, target_georef, crop_georef, image_size,
                  virtual_image_width, virtual_image_height, crop_bbox);

  } ASP_STANDARD_CATCHES;

  return 0;
}
