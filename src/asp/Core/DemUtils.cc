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

#include <asp/Core/DemUtils.h>
#include <asp/Core/PointUtils.h>

#include <vw/Image/AntiAliasing.h>
#include <vw/Image/Filter.h>
#include <vw/Image/InpaintView.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace vw;
namespace asp {

DemOptions::DemOptions():
  nodata_value(-std::numeric_limits<float>::max()),
  semi_major(0), semi_minor(0), fsaa(1),
  dem_hole_fill_len(0), ortho_hole_fill_len(0), ortho_hole_fill_extra_len(0),
  remove_outliers_with_pct(true), use_tukey_outlier_removal(false),
  max_valid_triangulation_error(0),
  erode_len(0), search_radius_factor(0), sigma_factor(0),
  default_grid_size_multiplier(1.0), use_surface_sampling(false),
  has_las_or_csv_or_pcd(false), max_output_size(9999999, 9999999), 
  auto_proj_center(false), input_is_projected(false) {}

// Create an antialiased DEM. This is old code. Needs to be wiped at some point.
ImageViewRef<PixelGray<float>>
generate_fsaa_raster(asp::OrthoRasterizerView const& rasterizer, 
                     DemOptions const& opt) {
  // This probably needs a Lanczos filter. Sinc filter is the ideal since it is
  // the ideal brick filter. Or possibly apply the blur on a linear scale
  // (pow(0,2.2), blur, then exp).

  float fsaa_sigma  = 1.0f * float(opt.fsaa)/2.0f;
  int   kernel_size = vw::compute_kernel_size(fsaa_sigma);

  ImageViewRef<PixelGray<float>> rasterizer_fsaa;
  if (opt.fsaa > 1) {
    // subsample with antialiasing
    // TODO(oalexan1): Break up this sorry mess
    rasterizer_fsaa = apply_mask(vw::resample_aa(translate(gaussian_filter
                       (fill_nodata_with_avg(create_mask(rasterizer.impl(), opt.nodata_value),
                         kernel_size), fsaa_sigma),
                        -double(opt.fsaa-1)/2.0, double(opt.fsaa-1)/2.0,
                        ConstantEdgeExtension()), 1.0/opt.fsaa),
                        opt.nodata_value);
  } else {
    rasterizer_fsaa = rasterizer.impl();
  }
  return rasterizer_fsaa;
}

// The files will be input point clouds, and if opt.do_ortho is
// true, also texture files. If texture files are present, there
// must be one for each point cloud, and each cloud must have the
// same dimensions as its texture file.
void parse_input_clouds_textures(std::vector<std::string> const& files,
                                 DemOptions& opt) {

  int num = files.size();
  if (num == 0)
    vw_throw(ArgumentErr() << "Missing input point clouds.\n");

  // Ensure there were no unrecognized options
  for (int i = 0; i < num; i++){
    if (!files[i].empty() && files[i][0] == '-'){
      vw_throw(ArgumentErr() << "Unrecognized option: " << files[i] << ".\n");
    }
  }

  // Ensure that files exist
  for (int i = 0; i < num; i++){
    if (!fs::exists(files[i])){
      vw_throw(ArgumentErr() << "File does not exist: " << files[i] << ".\n");
    }
  }

  if (opt.do_ortho){
    if (num <= 1)
      vw_throw(ArgumentErr() << "Missing input texture files.\n");
    if (num%2 != 0)
      vw_throw(ArgumentErr()
                << "There must be as many texture files as input point clouds.\n");
  }

  // Separate the input point clouds from the textures
  opt.pointcloud_files.clear(); opt.texture_files.clear();
  for (int i = 0; i < num; i++){
    if (asp::is_las_or_csv_or_pcd(files[i]) || get_num_channels(files[i]) >= 3)
      opt.pointcloud_files.push_back(files[i]);
    else
      opt.texture_files.push_back(files[i]);
  }

  if (opt.pointcloud_files.empty())
    vw_throw(ArgumentErr() << "No valid point cloud files were provided.\n");

  if (!opt.do_ortho && !opt.texture_files.empty())
    vw_throw(ArgumentErr() << "No ortho image was requested, yet texture files were passed as inputs.\n");

  // Must have this check here before we start assuming all input files
  // are tif.
  opt.has_las_or_csv_or_pcd = false;
  for (int i = 0; i < (int)files.size(); i++)
    opt.has_las_or_csv_or_pcd = (opt.has_las_or_csv_or_pcd || 
                                 asp::is_las_or_csv_or_pcd(files[i]));
  if (opt.has_las_or_csv_or_pcd && opt.do_ortho)
    vw_throw(ArgumentErr() 
             << "Cannot create orthoimages if point clouds are LAS or CSV.\n");

  if (opt.do_ortho){

    if (opt.pointcloud_files.size() != opt.texture_files.size())
      vw_throw(ArgumentErr() << "There must be as many input point clouds "
                              << "as texture files to be able to create orthoimages.\n");

    for (int i = 0; i < (int)opt.pointcloud_files.size(); i++){
      // Here we ignore that a point cloud file may have many channels.
      // We just want to verify that the cloud file and texture file
      // have the same number of rows and columns.
      DiskImageView<float> cloud(opt.pointcloud_files[i]);
      DiskImageView<float> texture(opt.texture_files[i]);
      if (cloud.cols() != texture.cols() || cloud.rows() != texture.rows()) {
        vw_throw(ArgumentErr() << "Point cloud " << opt.pointcloud_files[i]
                                << " and texture file " << opt.texture_files[i]
                                << " do not have the same dimensions.\n");
      }
    }
  }

  return;
}

// Convert any LAS or CSV files to ASP tif files. We do some binning
// to make the spatial data more localized, to improve performance.
// - We will later wipe these temporary tifs.
void las_or_csv_or_pcd_to_tifs(DemOptions& opt, vw::cartography::Datum const& datum,
                               std::vector<std::string> & tmp_tifs) {

  if (!opt.has_las_or_csv_or_pcd)
    return;

  Stopwatch sw;
  sw.start();

  // Error checking for CSV
  int num_files = opt.pointcloud_files.size();
  for (int i = 0; i < num_files; i++){
    if (!asp::is_csv(opt.pointcloud_files[i]))
      continue;
    if (opt.csv_format_str == "")
      vw_throw(ArgumentErr() << "CSV files were passed in, but the "
                             << "CSV format string was not set.\n");
  }

  // Extract georef info from PC or las files.
  vw::cartography::GeoReference pc_georef;
  bool have_pc_georef = asp::georef_from_pc_files(opt.pointcloud_files, pc_georef);

  // Configure a CSV converter object according to the input parameters
  asp::CsvConv csv_conv;
  csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str); // Modifies csv_conv

  // Set the georef for CSV files, if user's csv_proj4_str if specified
  vw::cartography::GeoReference csv_georef;
  csv_conv.parse_georef(csv_georef);

  // TODO: This may be a bug. What if the csv-proj4 and t_srs strings use
  // datums with different radii? 
  csv_georef.set_datum(datum);

  if (!have_pc_georef) // if we have no georef so far, the csv georef is our best guess.
    pc_georef = csv_georef;

  // There are situations in which some files will already be tif, and
  // others will be LAS or CSV. When we convert the latter to tif,
  // we'd like to be able to match the number of rows of the existing
  // tif files, so later when we concatenate all these files from left
  // to right for the purpose of creating the DEM, we waste little space.
  std::int64_t num_rows = 0;
  for (int i = 0; i < num_files; i++){
    if (asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i]))
      continue;
    DiskImageView<float> img(opt.pointcloud_files[i]);
    // Record the max number of rows across all input tifs
    num_rows = std::max(num_rows, std::int64_t(img.rows())); 
  }

  // No tif files exist. Find a reasonable value for the number of rows.
  if (num_rows == 0) {
    std::int64_t max_num_pts = 0;
    for (int i = 0; i < num_files; i++){
      std::string file = opt.pointcloud_files[i];
      if (asp::is_las(file))  max_num_pts = std::max(max_num_pts, asp::las_file_size(file));
      if (asp::is_csv(file))  max_num_pts = std::max(max_num_pts, asp::csv_file_size(file));
      if (asp::is_pcd(file))  max_num_pts = std::max(max_num_pts, asp::pcd_file_size(file)); // Note: PCD support needs to be tested!
      // No need to check for other cases; At least one file must be las or csv or pcd!
    }
    num_rows = std::max(std::int64_t(1), (std::int64_t)ceil(sqrt(double(max_num_pts))));
  }

  // This is very important. For efficiency later, we don't want to
  // create blocks smaller than what OrthoImageView will use later.
  int block_size = ASP_MAX_SUBBLOCK_SIZE;

  // For csv and las files, create temporary tif files. In those files
  // we'll have the points binned so that nearby points have nearby
  // indices.  This is key to fast rasterization later.
  for (int i = 0; i < num_files; i++){

    if (!asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i])) // Skip tif files
      continue;
    std::string in_file = opt.pointcloud_files[i];
    std::string stem    = fs::path(in_file).stem().string();
    std::string suffix;
    if (opt.out_prefix.find(stem) != std::string::npos)
      suffix = ".tif";
    else
      suffix = "-" + stem + ".tif";
    std::string out_file = opt.out_prefix + "-tmp" + suffix;

    // Handle the case when the output file may exist
    const int NUM_TEMP_NAME_RETRIES = 1000;
    for (int count = 0; count < NUM_TEMP_NAME_RETRIES; count++){
      if (!fs::exists(out_file))
        break;
      // File exists, try a different name
      vw_out() << "File exists: " << out_file << std::endl;
      std::ostringstream os; os << count;
      out_file = opt.out_prefix + "-tmp-" + os.str() + suffix;
    }
    if (fs::exists(out_file))
      vw_throw(ArgumentErr() << "Too many attempts at creating a temporary file.\n");

    // TODO: This if statement should not be needed, the function should handle it!
    // Perform the actual conversion to a tif file
    if (asp::is_las(in_file))
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, &opt, pc_georef, csv_conv);
    else // CSV
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, &opt, csv_georef, csv_conv);

    opt.pointcloud_files[i] = out_file; // so we can use it instead of the las file
    tmp_tifs.push_back(out_file); // so we can wipe it later
  }

  sw.stop();
  vw_out(DebugMessage,"asp") << "LAS or CSV to TIF conversion time: "
                             << sw.elapsed_seconds() << " seconds.\n";

}

// Write an image to disk while handling some common options.
template<class ImageT>
void save_image(DemOptions& opt, ImageT img, vw::cartography::GeoReference const& georef,
                int hole_fill_len, std::string const& imgName){

  // When hole-filling is used, we need to look hole_fill_len beyond
  // the current block.  If the block size is 256, and hole fill len
  // is big, like 512 or 1024, we end up processing a huge block
  // only to save a small center block.  For that reason, save
  // temporarily with big blocks, and then re-save with small blocks.
  if (hole_fill_len > 512)
    vw_out(WarningMessage) << "Detected large hole-fill length. "
                            << "Memory usage and run-time may go up.\n";

  int block_size = nextpow2(2.0*hole_fill_len);
  block_size = std::max(256, block_size);

  // Append a tag if desired to compute the min, max, etc. Later on, in OrthoRasterizer
  // we do a full validation of opt.filter.
  std::string tag = "";
  if (opt.filter != "weighted_average")
    tag = "-" + opt.filter; 

  std::string output_file = opt.out_prefix + tag + "-" + imgName
    + "." + opt.output_file_type;
  vw_out() << "Writing: " << output_file << "\n";
  TerminalProgressCallback tpc("asp", imgName + ": ");
  bool has_georef = true, has_nodata = true;
  if (opt.output_file_type == "tif")
    asp::save_with_temp_big_blocks(block_size, output_file, img,
                                    has_georef, georef,
                                    has_nodata, opt.nodata_value, opt, tpc);
  else 
    vw::cartography::write_gdal_image(output_file, img, georef, opt, tpc);
} // End function save_image

// Round pixels in given image to multiple of given scale.
// Don't round nodata values.
template <class PixelT>
struct RoundImagePixelsSkipNoData: public vw::ReturnFixedType<PixelT> {

  double m_scale, m_nodata;

  RoundImagePixelsSkipNoData(double scale, double nodata): 
  m_scale(scale), m_nodata(nodata) {}

  PixelT operator() (PixelT const& pt) const {

    // We will pass in m_scale = 0 if we don't want rounding to happen.
    if (m_scale <= 0)
      return pt;

    // Skip given pixel if any channels are nodata
    int num_channels = PixelNumChannels<PixelT>::value;
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    for (int c = 0; c < num_channels; c++){
      if ((double)compound_select_channel<channel_type const&>(pt,c) == m_nodata)
        return pt;
    }

    return PixelT(m_scale*round(channel_cast<double>(pt)/m_scale));
  }

};

// If the third component of a vector is NaN, mask that vector as invalid
template<class VectorT>
struct NaN2Mask: public ReturnFixedType< PixelMask<VectorT> > {
  NaN2Mask(){}
  PixelMask<VectorT> operator() (VectorT const& vec) const {
    if (boost::math::isnan(vec.z()))
      return PixelMask<VectorT>(); // invalid
    else
      return PixelMask<VectorT>(vec); // valid
  }
};

// Reverse the operation of NaN2Mask
template<class VectorT>
struct Mask2NaN: public ReturnFixedType<VectorT> {
  Mask2NaN(){}
  VectorT operator() (PixelMask<VectorT> const& pvec) const {
    if (!is_valid(pvec))
      return VectorT(0, 0, std::numeric_limits<typename VectorT::value_type>::quiet_NaN());
    else
      return pvec.child();
  }
};

// If the third component of a vector is NaN, assign to it the given no-data value
struct NaN2NoData: public ReturnFixedType<Vector3> {
  NaN2NoData(float nodata_val):m_nodata_val(nodata_val){}
  float m_nodata_val;
  Vector3 operator() (Vector3 const& vec) const {
    if (boost::math::isnan(vec.z()))
      return Vector3(m_nodata_val, m_nodata_val, m_nodata_val); // invalid
    else
      return vec; // valid
  }
};

// Take a given point xyz and the error at that point. Convert the
// error to the NED (North-East-Down) coordinate system.
struct ErrorToNED: public ReturnFixedType<Vector3> {
  vw::cartography::GeoReference m_georef;
  ErrorToNED(vw::cartography::GeoReference const& georef):m_georef(georef) {}

  Vector3 operator() (Vector6 const& pt) const {

    Vector3 xyz = subvector(pt, 0, 3);
    if (xyz == Vector3()) return Vector3();

    Vector3   err     = subvector(pt, 3, 3);
    Vector3   geo     = m_georef.datum().cartesian_to_geodetic(xyz);
    Matrix3x3 M       = m_georef.datum().lonlat_to_ned_matrix(subvector(geo, 0, 2));
    Vector3   ned_err = inverse(M)*err;
    return ned_err;
  }
};
template <class ImageT>
UnaryPerPixelView<ImageT, ErrorToNED>
inline error_to_NED(ImageViewBase<ImageT> const& image, 
                    vw::cartography::GeoReference const& georef) {
  return UnaryPerPixelView<ImageT, ErrorToNED>(image.impl(), ErrorToNED(georef));
}

template <class ImageT>
vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type> >
inline round_image_pixels_skip_nodata(vw::ImageViewBase<ImageT> const& image,
                                        double scale, double nodata) {
  return vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type> >
    (image.impl(), RoundImagePixelsSkipNoData<typename ImageT::pixel_type>(scale, nodata));
}

// A class for combining the three channels of errors and finding their absolute values.
// TODO(oalexan1): Move this to a separate file.
// TODO(oalexan1): Make this not be a class but rather working with 
// ImageViewRef<PixelGray<float>>. This will be a big change.
class CombinedView: public ImageViewBase<CombinedView> {
  double m_nodata_value;
  ImageViewRef<PixelGray<float>> m_image1;
  ImageViewRef<PixelGray<float>> m_image2;
  ImageViewRef<PixelGray<float>> m_image3;

public:

  typedef Vector3f pixel_type;
  typedef const Vector3f result_type;
  typedef ProceduralPixelAccessor<CombinedView> pixel_accessor;

  CombinedView(double nodata_value,
                ImageViewRef<PixelGray<float>> image1,
                ImageViewRef<PixelGray<float>> image2,
                ImageViewRef<PixelGray<float>> image3):
    m_nodata_value(nodata_value),
    m_image1(image1),
    m_image2(image2),
    m_image3(image3){}

  inline int32 cols  () const { return m_image1.cols(); }
  inline int32 rows  () const { return m_image1.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()(size_t i, size_t j, size_t p=0) const {

    Vector3f error(m_image1(i, j), m_image2(i, j), m_image3(i, j));

    if (error[0] == m_nodata_value || error[1] == m_nodata_value || 
        error[2] == m_nodata_value) {
      return Vector3f(m_nodata_value, m_nodata_value, m_nodata_value);
    }

    return Vector3f(std::abs(error[0]), std::abs(error[1]), std::abs(error[2]));
  }

  typedef CombinedView prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    return prerasterize_type(m_nodata_value,
                              m_image1.prerasterize(bbox),
                              m_image2.prerasterize(bbox),
                              m_image3.prerasterize(bbox));
  }
  template <class DestT> 
  inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
CombinedView combine_channels(double nodata_value,
                              ImageViewRef<PixelGray<float>> image1,
                              ImageViewRef<PixelGray<float>> image2,
                              ImageViewRef<PixelGray<float>> image3) {
  VW_ASSERT(image1.cols() == image2.cols() &&
            image2.cols() == image3.cols() &&
            image1.rows() == image2.rows() &&
            image2.rows() == image3.rows(),
            ArgumentErr() << "Expecting the error channels to have the same size.");

  return CombinedView(nodata_value, image1, image2, image3);
}

// Rasterize a DEM
// TODO(oalexan1): This function is huge and must be broken up
void do_software_rasterization(asp::OrthoRasterizerView& rasterizer,
                               DemOptions& opt,
                               vw::cartography::GeoReference& georef,
                               ImageViewRef<double> const& error_image,
                               double estim_max_error,
                               std::int64_t * num_invalid_pixels) {

  vw_out() << "\t-- Starting DEM rasterization --\n";
  vw_out() << "\t--> DEM spacing: " <<     rasterizer.spacing() << " pt/px\n";
  vw_out() << "\t             or: " << 1.0/rasterizer.spacing() << " px/pt\n";

  // TODO: Maybe put a warning or check here if the size is too big

  // Now we are ready to specify the affine transform.
  georef.set_transform(rasterizer.geo_transform());

  // If the user requested FSAA, we temporarily increase the
  // resolution, apply a blur, then resample to the original
  // resolution. This results in a DEM with less antialiasing.  Note
  // that the georef above is set with the spacing before resolution
  // is increased, which will be the final spacing as well.
  if (opt.fsaa > 1)
    rasterizer.set_spacing(rasterizer.spacing() / double(opt.fsaa));

  // If the user specified the ULLR .. update the georeference
  // transform here. The generate_fsaa_raster will be responsible
  // for making sure we have the correct pixel crop.
  if (opt.target_projwin != BBox2()) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) = opt.target_projwin.min().x();
    transform(1,2) = opt.target_projwin.max().y();
    georef.set_transform(transform);
  }

  // Fix have pixel offset required if pixel_interpretation is
  // PixelAsArea. We could have done that earlier, but it makes
  // the above easier to not think about it.
  if (georef.pixel_interpretation() == vw::cartography::GeoReference::PixelAsArea) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) -= 0.5 * transform(0,0);
    transform(1,2) -= 0.5 * transform(1,1);
    georef.set_transform(transform);
  }

  // Do not round the DEM heights for small bodies
  if (georef.datum().semi_major_axis() <= asp::MIN_RADIUS_FOR_ROUNDING ||
      georef.datum().semi_minor_axis() <= asp::MIN_RADIUS_FOR_ROUNDING) {
    opt.rounding_error = 0.0;
  }

  ImageViewRef<PixelGray<float>> rasterizer_fsaa
    = generate_fsaa_raster(rasterizer, opt);

  // Write out the DEM. We've set the texture to be the height.
  Vector2 tile_size(vw_settings().default_tile_size(),
                    vw_settings().default_tile_size());
  if (!opt.no_dem){
    Stopwatch sw2;
    sw2.start();
    ImageViewRef<PixelGray<float>> dem
      = asp::round_image_pixels_skip_nodata(rasterizer_fsaa, opt.rounding_error,
                                            opt.nodata_value);

    int hole_fill_len = opt.dem_hole_fill_len;
    if (hole_fill_len > 0){
      // Note that we first cache the tiles of the rasterized DEM, and
      // fill holes later. This greatly improves the performance.
      dem = apply_mask
        (vw::fill_holes_grass(create_mask
                               (block_cache(dem, tile_size, opt.num_threads),
                                opt.nodata_value),
                               hole_fill_len),
         opt.nodata_value);
    }

    // Stop the program if it is going to create too large a DEM, this will
    // cause a crash.
    Vector2i dem_size = bounding_box(dem).size();
    vw_out()<< "Creating output file that is " << dem_size << " px.\n";
    if ((dem_size[0] > opt.max_output_size[0]) || (dem_size[1] > opt.max_output_size[1]))
      vw_throw(ArgumentErr()
                << "Requested DEM size is too large, max allowed output size is "
                << opt.max_output_size << " pixels.\n");

    asp::save_image(opt, dem, georef, hole_fill_len, "DEM");
    sw2.stop();
    vw_out(DebugMessage,"asp") 
    << "DEM render time: " << sw2.elapsed_seconds() << ".\n";

    double num_invalid_pixelsD = *num_invalid_pixels;

    // This is to compensate for the fact that for fsaa > 1 we use a
    // finer rendering grid than the final one. Later a blur and a
    // resampling to the original grid will happen, so the actual
    // count is not as simple as what we do here, but this may be good
    // enough since the fsaa option is not the default and may need
    // wiping at some point.
    if (opt.fsaa > 1)
      num_invalid_pixelsD = num_invalid_pixelsD / double(opt.fsaa) / double(opt.fsaa);
    
    // Below we convert to double first and multiply later, to avoid
    // 32-bit integer overflow.
    double num_total_pixels = double(dem_size[0]) * double(dem_size[1]);

    double invalid_ratio = num_invalid_pixelsD / num_total_pixels;
    vw_out() << "Percentage of valid pixels: " 
             << 100.0*(1.0 - invalid_ratio) << "%\n";
    *num_invalid_pixels = 0; // Reset this count
  }

  bool has_sd = asp::has_stddev(opt.pointcloud_files);
  if (opt.propagate_errors && !has_sd) {
    // Do not throw an error. Go on and save at least the intersection
    // error and orthoimage.
    vw_out() << "Cannot grid the horizontal and vertical stddev as the point cloud file is "
             << "not in the expected format.\n";
    opt.propagate_errors = false;
  }
  
  // Write triangulation error image if requested
  if (opt.do_error) {
    int num_channels = asp::num_channels(opt.pointcloud_files);
    
    int hole_fill_len = 0;
    if (num_channels == 4 || (num_channels == 6 && has_sd)) {
      // The error is a scalar (4 channels or 6 channels but last two are stddev)
      ImageViewRef<double> error_channel = asp::point_cloud_error_image(opt.pointcloud_files);
      rasterizer.set_texture(error_channel);
      rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
      save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer_fsaa,
                                                          opt.rounding_error,
                                                          opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    } else if (num_channels == 6) {
      // The error is a 3D vector. Convert it to NED coordinate system, and rasterize it.
      ImageViewRef<Vector6> point_disk_image = asp::form_point_cloud_composite<Vector6>
        (opt.pointcloud_files, ASP_MAX_SUBBLOCK_SIZE);
      ImageViewRef<Vector3> ned_err = asp::error_to_NED(point_disk_image, georef);
      std::vector<ImageViewRef<PixelGray<float>>>  rasterized(3);
      for (int ch_index = 0; ch_index < 3; ch_index++){
        ImageViewRef<double> ch = select_channel(ned_err, ch_index);
        rasterizer.set_texture(ch);
        rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
        rasterized[ch_index] =
          block_cache(rasterizer_fsaa, tile_size, opt.num_threads);
      }
      save_image(opt, asp::round_image_pixels_skip_nodata
                              (asp::combine_channels(opt.nodata_value, rasterized[0], 
                                                     rasterized[1], rasterized[2]),
                               opt.rounding_error, opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    }else{
      // Note: We don't throw here. We still would like to write the
      // DRG (below) even if we can't write the error image.
      vw_out() << "The point cloud files must have an equal number of channels which "
               << "must be 4 or 6 to be able to process the intersection error.\n";
    }
  }

  if (opt.propagate_errors) {
    int num_channels = asp::num_channels(opt.pointcloud_files);
    double rounding_error = 0.0;
    vw_out() << "Not rounding propagated errors (option: --rounding-error) to avoid "
             << "introducing step artifacts.\n";
    
    // Note: We don't throw here. We still would like to write the
    // DRG (below) even if we can't write the stddev.
    if (num_channels != 6) {
      vw_out() << "The input point cloud(s) must have 6 channels to be able to "
               << "grid the horizontal and vertical stddev.\n";
    } else {
      int hole_fill_len = 0;
      ImageViewRef<Vector6> point_disk_image = asp::form_point_cloud_composite<Vector6>
        (opt.pointcloud_files, ASP_MAX_SUBBLOCK_SIZE);
      
      ImageViewRef<double> horizontal_cov_channel = select_channel(point_disk_image, 4);
      rasterizer.set_texture(horizontal_cov_channel);
      rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
      save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer_fsaa,
                                                          rounding_error, // local value
                                                          opt.nodata_value),
                 georef, hole_fill_len, "HorizontalStdDev");
      
      ImageViewRef<double> vertical_cov_channel = select_channel(point_disk_image, 5);
      rasterizer.set_texture(vertical_cov_channel);
      rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
      save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer_fsaa,
                                                          rounding_error, // local value
                                                          opt.nodata_value),
                 georef, hole_fill_len, "VerticalStdDev");
    }
  }
  
  // Write out a normalized version of the DEM, if requested (for debugging)
  if (opt.do_normalize) {
    int hole_fill_len = 0;
    DiskImageView< PixelGray<float> > dem_image(opt.out_prefix + "-DEM." + opt.output_file_type);
    asp::save_image(opt, apply_mask(channel_cast<uint8>
                                    (normalize(create_mask(dem_image,opt.nodata_value),
                                               rasterizer.bounding_box().min().z(),
                                               rasterizer.bounding_box().max().z(),
                                               0, 255))),
                    georef, hole_fill_len, "DEM-normalized");
  }

  // Write DRG if the user requested and provided a texture file.
  // This must be at the end, as we may be messing with the point
  // image in irreversible ways.
  if (opt.do_ortho) {
    Stopwatch sw3;
    sw3.start();
    ImageViewRef<PixelGray<float>> texture
      = asp::form_point_cloud_composite<PixelGray<float>>
      (opt.texture_files, ASP_MAX_SUBBLOCK_SIZE);
    rasterizer.set_texture(texture);

    if (opt.ortho_hole_fill_len > 0) {

      // Need to convert the hole fill length from output image pixels
      // to input point cloud pixels as we will fill holes in the cloud
      // itself.
      int hole_fill_len = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_len);

      // Fetch a reference to the current point image
      ImageViewRef<Vector3> point_image = rasterizer.get_point_image();

      // Mask the NaNs
      ImageViewRef<PixelMask<Vector3>> point_image_mask
        = per_pixel_filter(point_image, asp::NaN2Mask<Vector3>());

      // If to grow the cloud a bit, to help hole-filling later. This should
      // not be large as it creates artifacts. The main work better
      // be done by grassfire later. 
      if (opt.ortho_hole_fill_extra_len > 0) {
        int hole_fill_mode = 2;
        int hole_fill_num_smooth_iter = 3;
        int hole_fill_extra_len
          = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_extra_len);
        point_image_mask = vw::fill_holes(point_image_mask, hole_fill_mode,
                                          hole_fill_num_smooth_iter,
                                          hole_fill_extra_len);

        // Use big tiles, to reduce the overhead
        // of expanding each tile by hole size. 
        int big_block_size = nextpow2(2.0*hole_fill_extra_len);
        big_block_size = std::max(256, big_block_size);

        // Cache each hole-filled point cloud tile as likely we will
        // need it again in the future when rasterizing a different
        // portion of the output ortho image.
        point_image_mask = block_cache(point_image_mask, 
                                       vw::Vector2(big_block_size, big_block_size),
                                       opt.num_threads);
      }

      // Fill the holes
      point_image_mask =vw::fill_holes_grass(point_image_mask, hole_fill_len);

      // back to NaNs
      point_image = per_pixel_filter(point_image_mask, asp::Mask2NaN<Vector3>());

      // When filling holes, use big tiles, to reduce the overhead
      // of expanding each tile by hole size. 
      int big_block_size = nextpow2(2.0*hole_fill_len);
      big_block_size = std::max(256, big_block_size);

      // Cache each hole-filled point cloud tile as likely we will
      // need it again in the future when rasterizing a different
      // portion of the output ortho image.
      point_image = block_cache(point_image, 
                                vw::Vector2(big_block_size, big_block_size),
                                opt.num_threads);

      // Pass to the rasterizer the point image with the holes filled
      rasterizer.set_point_image(point_image);
    }

    rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
    asp::save_image(opt, rasterizer_fsaa, georef,
                    0, // no need for a buffer here, as we cache hole-filled tiles
                    "DRG");
    sw3.stop();
    vw_out(DebugMessage,"asp") << "DRG render time: " << sw3.elapsed_seconds() << "\n";
  }

} // End do_software_rasterization

} // end namespace asp
