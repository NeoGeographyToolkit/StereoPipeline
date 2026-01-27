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

// This program works with ASTER V004 data, in .hdf format, and with older V0003
// data, when the inputs are expected to be a directory.

// For V003, data. The workflow is as follows. Take as input a directory of
// ASTER images. Apply radiometric correction to the VNIR_Band3N and VNIR_Band3B
// images, writing <output-prefix>-Band3N.tif and <output-prefix>-Band3B.tif.
// Generate RPC coefficients from input metadata and write
// <output-prefix>-Band3N.xml and <output-prefix>-Band3B.xml. These files can
// then be used as input for stereo with -t rpc.

// References:
// -----------
// ASTER User Handbook Version 2
// https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf
//
// ASTER User Guide Version 4 (for V004 HDF format)
// https://lpdaac.usgs.gov/documents/2265/ASTER_User_Guide_V4_pcP80n5.pdf
//
// https://lpdaac.usgs.gov/documents/175/ASTER_L1_Product_Specifications.pdf

// IMPROVEMENT OF DEM GENERATION FROM ASTER IMAGES USING SATELLITE
// JITTER ESTIMATION AND OPEN SOURCE IMPLEMENTATION
// Luc Girod, Christopher Nutha, and Andreas Kaab
// http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XL-1-W5/249/2015/isprsarchives-XL-1-W5-249-2015.pdf

#include <asp/Camera/RPCModelGen.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/Macros.h>

#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Core/StringUtils.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>

#include <cstring>
#include <limits>
#include <iomanip>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <gdal_priv.h>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::cartography;

struct Options: public vw::GdalWriteOptions {
  std::string input, output_prefix;
  double min_height, max_height;
  std::int64_t num_samples;
  double penalty_weight;
  bool keep_tmp_dir;
  Options(): min_height(-1), max_height(-1), num_samples(-1), penalty_weight(-1),
             keep_tmp_dir(false) {}
};

void handle_arguments(int argc, char *argv[], Options &opt) {
  
  po::options_description general_options("");
  general_options.add_options()
  ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.")
  ("min-height", po::value(&opt.min_height)->default_value(0),
   "The minimum height (in meters) above the WGS84 datum of the simulation "
   "box in which to compute the RPC approximation.")
  ("max-height", po::value(&opt.max_height)->default_value(8.0e+3),
   "The maximum height (in meters) above the WGS84 datum of the simulation "
   "box in which to compute the RPC approximation.")
  ("num-samples", po::value(&opt.num_samples)->default_value(100),
   "How many samples to use between the minimum and maximum heights.")
  ("penalty-weight", po::value(&opt.penalty_weight)->default_value(0.1),
   "Penalty weight to use to keep the higher-order RPC coefficients small. "
   "Higher penalty weight results in smaller such coefficients.")
  ("keep-tmp-dir", po::bool_switch(&opt.keep_tmp_dir)->default_value(false),
   "Keep the temporary directory where HDF data is extracted (for debugging).")
  ;

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
  ("input", po::value(&opt.input), "An ASTER HDF file or input data directory.");

  po::positional_options_description positional_desc;
  positional_desc.add("input", 1);

  std::string usage("<input hdf or directory> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm = asp::check_command_line(
      argc, argv, opt, general_options, general_options, positional,
      positional_desc, usage, allow_unregistered, unregistered);

  if (opt.input.empty())
    vw_throw(ArgumentErr() << "Missing input.\n"
                           << usage << general_options);

  if (opt.output_prefix.empty())
    vw_throw(ArgumentErr() << "Missing output prefix.\n"
                           << usage << general_options);

  // Create the output directory
  vw::create_out_dir(opt.output_prefix);
}

// See if the input file matches the given pattern. If yes, store the result
// in matched_file. If matched_file is not empty originally, that means
// we have more than one match, which is not good.
bool match_file(std::string const &input_file, std::string const &pattern,
                std::string &matched_file) {

  std::size_t it = input_file.find(pattern);

  // No match
  if (it == std::string::npos)
    return false;

  // Check if it does not match at the end of the string
  if (it + pattern.size() != input_file.size())
    return false;

  // Must match L1A images
  if (input_file.find("L1A") == std::string::npos)
    return false;

  // We expect only one match
  if (matched_file != "")
    vw_throw(ArgumentErr() << "Found two files matching: '" << pattern
                           << "'. Those are: " << matched_file << " and "
                           << input_file << "\n");

  matched_file = input_file;

  return true;
}

// Identify the appropriate inputs in the ASTER directory
void locate_inputs(Options const &opt, std::string &nadir_image,
                   std::string &back_image, std::string &nadir_sat_pos,
                   std::string &back_sat_pos, std::string &nadir_sight_vec,
                   std::string &back_sight_vec, std::string &nadir_corr_table,
                   std::string &back_corr_table, std::string &nadir_longitude,
                   std::string &back_longitude, std::string &nadir_latitude,
                   std::string &back_latitude, std::string &nadir_lattice_point,
                   std::string &back_lattice_point) {

  // Iterate through all files
  for (fs::directory_iterator itr(opt.input); itr != fs::directory_iterator(); itr++) {

    std::string file = itr->path().string();

    match_file(file, "VNIR_Band3N.ImageData.tif", nadir_image);
    match_file(file, "VNIR_Band3B.ImageData.tif", back_image);

    match_file(file, "VNIR_Band3N.SatellitePosition.txt", nadir_sat_pos);
    match_file(file, "VNIR_Band3B.SatellitePosition.txt", back_sat_pos);

    match_file(file, "VNIR_Band3N.SightVector.txt", nadir_sight_vec);
    match_file(file, "VNIR_Band3B.SightVector.txt", back_sight_vec);

    match_file(file, "VNIR_Band3N.RadiometricCorrTable.txt", nadir_corr_table);
    match_file(file, "VNIR_Band3B.RadiometricCorrTable.txt", back_corr_table);

    match_file(file, "VNIR_Band3N.Longitude.txt", nadir_longitude);
    match_file(file, "VNIR_Band3B.Longitude.txt", back_longitude);

    match_file(file, "VNIR_Band3N.Latitude.txt", nadir_latitude);
    match_file(file, "VNIR_Band3B.Latitude.txt", back_latitude);

    match_file(file, "VNIR_Band3N.LatticePoint.txt", nadir_lattice_point);
    match_file(file, "VNIR_Band3B.LatticePoint.txt", back_lattice_point);
  }

  if (nadir_image == "")
    vw_throw(ArgumentErr() << "Could not locate the nadir-looking camera image "
                           << "(VNIR_Band3N.ImageData.tif).\n");
  if (back_image == "")
    vw_throw(
        ArgumentErr() << "Could not locate the backward-looking camera image "
                      << "(VNIR_Band3B.ImageData.tif).\n");

  if (nadir_sat_pos == "")
    vw_throw(ArgumentErr()
             << "Could not locate the nadir-looking satellite position "
             << "(VNIR_Band3N.SatellitePosition.txt).\n");
  if (back_sat_pos == "")
    vw_throw(
        ArgumentErr() << "Could not locate the back-looking satellite position "
                      << "(VNIR_Band3B.SatellitePosition.txt).\n");

  if (nadir_sight_vec == "")
    vw_throw(ArgumentErr() << "Could not locate the nadir-looking sight vector "
                           << "(VNIR_Band3N.SightVector.txt).\n");
  if (back_sight_vec == "")
    vw_throw(ArgumentErr() << "Could not locate the back-looking sight vector "
                           << "(VNIR_Band3B.SightVector.txt).\n");

  if (nadir_corr_table == "")
    vw_throw(
        ArgumentErr()
        << "Could not locate the nadir-looking radiometric correction table "
        << "(VNIR_Band3N.RadiometricCorrTable.txt).\n");
  if (back_corr_table == "")
    vw_throw(
        ArgumentErr()
        << "Could not locate the back-looking radiometric correction table "
        << "(VNIR_Band3B.RadiometricCorrTable.txt).\n");

  if (nadir_longitude == "")
    vw_throw(
        ArgumentErr() << "Could not locate the nadir-looking longitude file "
                      << "(VNIR_Band3N.Longitude.txt).\n");
  if (back_longitude == "")
    vw_throw(
        ArgumentErr() << "Could not locate the back-looking longitude file "
                      << "(VNIR_Band3B.Longitude.txt).\n");

  if (nadir_latitude == "")
    vw_throw(
        ArgumentErr() << "Could not locate the nadir-looking latitude file "
                      << "(VNIR_Band3N.Latitude.txt).\n");
  if (back_latitude == "")
    vw_throw(ArgumentErr() << "Could not locate the back-looking latitude file "
                           << "(VNIR_Band3B.Latitude.txt).\n");

  if (nadir_lattice_point == "")
    vw_throw(ArgumentErr()
             << "Could not locate the nadir-looking lattice point file "
             << "(VNIR_Band3N.LatticePoint.txt).\n");
  if (back_lattice_point == "")
    vw_throw(
        ArgumentErr() << "Could not locate the back-looking lattice point file "
                      << "(VNIR_Band3B.LatticePoint.txt).\n");
}

// Apply the radiometric corrections
template <class ImageT>
class RadioCorrectView : public ImageViewBase<RadioCorrectView<ImageT>> {
  ImageT m_img;
  std::vector<Vector3> const &m_corr; // alias
  bool m_has_nodata;
  double m_nodata;

public:
  RadioCorrectView(ImageT const &img, std::vector<Vector3> const &corr,
                   bool has_nodata, double nodata): 
  m_img(img), m_corr(corr), m_has_nodata(has_nodata), m_nodata(nodata) {}

  typedef typename ImageT::pixel_type input_type;
  typedef float pixel_type;
  typedef float result_type;
  typedef ProceduralPixelAccessor<RadioCorrectView> pixel_accessor;

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double /*i*/, double /*j*/,
                               int32 /*p*/ = 0) const {
    vw_throw(
        NoImplErr() << "RadioCorrectView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const &bbox) const {

    ImageView<input_type> input_tile = crop(m_img, bbox); // to speed things up
    ImageView<result_type> tile(bbox.width(), bbox.height());
    for (std::int64_t col = bbox.min().x(); col < bbox.max().x(); col++) {
      Vector3 C = m_corr[col];
      for (std::int64_t row = bbox.min().y(); row < bbox.max().y(); row++) {
        input_type val = input_tile(col - bbox.min().x(), row - bbox.min().y());
        if (m_has_nodata && val == m_nodata)
          tile(col - bbox.min().x(), row - bbox.min().y()) = val;
        else
          tile(col - bbox.min().x(), row - bbox.min().y()) =
              C[1] * val / C[2] + C[0];
      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(), cols(),
                             rows());
  }

  template <class DestT>
  inline void rasterize(DestT const &dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
template <class ImageT>
RadioCorrectView<ImageT> radio_correct(ImageT const &img,
                                       std::vector<Vector3> const &corr,
                                       bool has_nodata, double nodata) {
  return RadioCorrectView<ImageT>(img, corr, has_nodata, nodata);
}

// ASTER L1A images come with radiometric corrections appended, but not applied.
// There is one correction per image column.
void apply_radiometric_corrections(Options const &opt,
                                   std::string const &input_image,
                                   std::string const &corr_table,
                                   std::string const &out_image) {

  // Extract the corrections
  std::vector<Vector3> corr;
  asp::read_3d_points(corr_table, corr);

  DiskImageView<float> input_img(input_image);

  // Sanity check
  if (input_img.cols() != int(corr.size()))
    vw_throw(ArgumentErr() << "Expecting as many corrections in " << corr_table
                           << " as image columns in " << input_image << "\n");

  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max();
  // See if there is a no-data value
  {
    boost::shared_ptr<DiskImageResource> img_rsrc(
        new DiskImageResourceGDAL(input_image));
    if (img_rsrc->has_nodata_read()) {
      has_nodata = true;
      nodata = img_rsrc->nodata_read();
    }
  }

  // No georef, this being L1A imagery
  vw::cartography::GeoReference georef;
  bool has_georef = read_georeference(georef, input_image);
  if (has_georef)
    vw_throw(ArgumentErr()
             << "ASTER L1A images are not supposed to be georeferenced.\n");

  vw_out() << "Writing: " << out_image << std::endl;
  vw::cartography::block_write_gdal_image(
      out_image, radio_correct(input_img, corr, has_nodata, nodata), has_georef,
      georef, has_nodata, nodata, opt,
      TerminalProgressCallback("asp", "\t-->: "));
}

// Generate lon-lat-height to image pixel correspondences that we will
// use to create the RPC model.
void generate_point_pairs(// Inputs
                          double min_height, double max_height, std::int64_t num_samples,
                          double penalty_weight, std::string const &sat_pos_file,
                          std::string const &sight_vec_file, std::string const &longitude_file,
                          std::string const &latitude_file, std::string const &lattice_file,
                          // Outputs
                          std::vector<std::vector<vw::Vector3>> &world_sight_mat, Vector3 &llh_scale,
                          Vector3 &llh_offset, Vector2 &pixel_scale, Vector2 &pixel_offset,
                          Vector<double> &normalized_llh, Vector<double> &normalized_pixels) {

  // Read the sight vectors
  std::vector<Vector3> sight_vec;
  asp::read_3d_points(sight_vec_file, sight_vec);

  // Read the satellite positions
  std::vector<Vector3> sat_pos;
  asp::read_3d_points(sat_pos_file, sat_pos);

  std::int64_t num_rows = sat_pos.size();
  std::int64_t num_pts = sight_vec.size();
  std::int64_t num_cols = num_pts / num_rows;

  // Sight vector in world coordinates
  world_sight_mat.clear();
  world_sight_mat.resize(num_rows);

  if (num_rows * num_cols != num_pts)
    vw_throw(ArgumentErr() << "Found " << num_rows << " satellite positions in "
                           << sat_pos_file << " and " << num_pts
                           << " sight vectors in " << sight_vec_file
                           << ". The latter must be a multiple of the former.");

  // For each satellite position there many sight vectors
  // corresponding to pixel positions on that image line. Clone the
  // satellite positions to make them one per each sight vector. It is
  // easier to work with things that way later.
  std::vector<Vector3> full_sat_pos(num_pts);
  std::int64_t count = 0;
  for (std::int64_t row = 0; row < num_rows; row++) {
    for (std::int64_t col = 0; col < num_cols; col++) {
      full_sat_pos[count] = sat_pos[row];
      count++;
    }
  }
  if (count != num_pts)
    vw_throw(ArgumentErr() << "Book-keeping failure!\n");

  std::vector<double> longitude;
  asp::read_vec(longitude_file, longitude);
  if (std::int64_t(longitude.size()) != num_pts)
    vw_throw(ArgumentErr() << "Expecting " << num_pts << " longitude values in "
                           << longitude_file << " but got instead "
                           << longitude.size() << ".\n");

  std::vector<double> latitude;
  asp::read_vec(latitude_file, latitude);
  if (std::int64_t(latitude.size()) != num_pts)
    vw_throw(ArgumentErr() << "Expecting " << num_pts << " latitude values in "
                           << latitude_file << " but got instead "
                           << latitude.size() << ".\n");

  // Covert geocentric latitude to geodetic latitude. Pages 60 and 79 of
  // https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf
  // Geodetic = Arctan [(tan (Latitude)) / 0.99330562]
  double deg2rad = M_PI / 180.0;
  for (std::size_t i = 0; i < latitude.size(); i++) {
    latitude[i] = atan(tan(deg2rad * latitude[i]) / 0.99330562) / deg2rad;
  }

  std::vector<Vector2> pixels;
  asp::read_2d_points(lattice_file, pixels);
  if (std::int64_t(pixels.size()) != num_pts)
    vw_throw(ArgumentErr() << "Expecting " << num_pts << " pixels in "
                           << lattice_file << " but got instead "
                           << pixels.size() << ".\n");

  // Convert from geodetic coordinates to xyz
  cartography::Datum datum;
  datum.set_well_known_datum("WGS84");
  std::vector<Vector3> ground_xyz(num_pts);
  for (std::int64_t i = 0; i < num_pts; i++)
    ground_xyz[i] =
        datum.geodetic_to_cartesian(Vector3(longitude[i], latitude[i], 0));

  // Create the sight vectors, from the camera center to the ground, in world
  // coordinates, rather than in spacecraft coordinates, like sight_vec.
  for (std::int64_t pt = 0; pt < num_pts; pt++) {
    Vector3 G = ground_xyz[pt];
    Vector3 C = full_sat_pos[pt];
    std::int64_t row = pt / num_cols;
    world_sight_mat[row].push_back((G - C) / norm_2(G - C));
  }
  if (world_sight_mat.empty() ||
      (std::int64_t)world_sight_mat[0].size() != num_cols)
    vw_throw(ArgumentErr() << "Incorrect number of world sight vectors.\n");

  // Form num_samples layers between min_height and max_height.
  // Each point there will have its corresponding pixel value.
  std::int64_t num_total_pts = num_pts * num_samples;
  std::vector<Vector3> all_llh(num_total_pts);
  std::vector<Vector2> all_pixels(num_total_pts);
  count = 0;
  for (std::int64_t sample = 0; sample < num_samples; sample++) {
    double height = min_height + double(sample) * (max_height - min_height) /
                                     (num_samples - 1.0);

    // Find an xyz position at roughly that height on the line
    // connecting the original ground point and the satellite
    // center. We need to solve a quadratic equation for that. We
    // assume the Earth is a sphere.
    for (std::int64_t pt = 0; pt < num_pts; pt++) {

      Vector3 A = ground_xyz[pt];
      Vector3 B = full_sat_pos[pt];
      Vector3 D = B - A;

      // Find t such that norm(A + t*D) = norm(A) + height
      double d = dot_prod(A, D) * dot_prod(A, D) +
                 dot_prod(D, D) * (height * height + 2 * norm_2(A) * height);
      double t = (-dot_prod(A, D) + sqrt(d)) / dot_prod(D, D);
      Vector3 P = A + t * D;

      all_llh[count] = datum.cartesian_to_geodetic(P);
      all_pixels[count] = pixels[pt];
      count++;
    }
  }

  // Find the range of lon-lat-heights
  BBox3 llh_box;
  for (std::size_t i = 0; i < all_llh.size(); i++)
    llh_box.grow(all_llh[i]);

  // Find the range of pixels
  BBox2 pixel_box;
  for (std::size_t i = 0; i < all_pixels.size(); i++)
    pixel_box.grow(all_pixels[i]);

  llh_scale = (llh_box.max() - llh_box.min()) / 2.0;  // half range
  llh_offset = (llh_box.max() + llh_box.min()) / 2.0; // center point

  pixel_scale = (pixel_box.max() - pixel_box.min()) / 2.0;  // half range
  pixel_offset = (pixel_box.max() + pixel_box.min()) / 2.0; // center point

  normalized_llh.set_size(asp::RPCModel::GEODETIC_COORD_SIZE * num_total_pts);
  normalized_pixels.set_size(asp::RPCModel::IMAGE_COORD_SIZE * num_total_pts +
                             asp::RpcSolveLMA::NUM_PENALTY_TERMS);
  for (std::size_t i = 0; i < normalized_pixels.size(); i++) {
    // Important: The extra penalty terms are all set to zero here.
    normalized_pixels[i] = 0.0;
  }

  // Form the arrays of normalized pixels and normalized llh
  for (std::int64_t pt = 0; pt < num_total_pts; pt++) {

    // Normalize the pixel to -1 <> 1 range
    Vector3 llh_n = elem_quot(all_llh[pt] - llh_offset, llh_scale);
    Vector2 pixel_n = elem_quot(all_pixels[pt] - pixel_offset, pixel_scale);

    subvector(normalized_llh, asp::RPCModel::GEODETIC_COORD_SIZE * pt,
              asp::RPCModel::GEODETIC_COORD_SIZE) = llh_n;
    subvector(normalized_pixels, asp::RPCModel::IMAGE_COORD_SIZE * pt,
              asp::RPCModel::IMAGE_COORD_SIZE) = pixel_n;
  }

  return;
}

// Save an XML file having all RPC information
void save_xml(std::int64_t image_cols, std::int64_t image_rows,
              std::vector<std::vector<Vector2>> const &lattice_mat,
              std::vector<std::vector<Vector3>> const &sight_mat,
              std::vector<std::vector<Vector3>> const &world_sight_mat,
              std::vector<Vector3> const &sat_pos, Vector3 const &llh_scale,
              Vector3 const &llh_offset, Vector2 const &pixel_scale,
              Vector2 const &pixel_offset,
              asp::RPCModel::CoeffVec const &line_num,
              asp::RPCModel::CoeffVec const &line_den,
              asp::RPCModel::CoeffVec const &samp_num,
              asp::RPCModel::CoeffVec const &samp_den,
              std::string const &out_cam_file) {

  std::string lineoffset = vw::num_to_str(pixel_offset.y());
  std::string sampoffset = vw::num_to_str(pixel_offset.x());
  std::string latoffset = vw::num_to_str(llh_offset.y());
  std::string longoffset = vw::num_to_str(llh_offset.x());
  std::string heightoffset = vw::num_to_str(llh_offset.z());

  std::string linescale = vw::num_to_str(pixel_scale.y());
  std::string sampscale = vw::num_to_str(pixel_scale.x());
  std::string latscale = vw::num_to_str(llh_scale.y());
  std::string longscale = vw::num_to_str(llh_scale.x());
  std::string heightscale = vw::num_to_str(llh_scale.z());

  std::string linenumcoef = vw::vec_to_str(line_num);
  std::string linedencoef = vw::vec_to_str(line_den);
  std::string sampnumcoef = vw::vec_to_str(samp_num);
  std::string sampdencoef = vw::vec_to_str(samp_den);

  vw_out() << "Writing: " << out_cam_file << std::endl;
  std::ofstream ofs(out_cam_file.c_str());

  ofs << "<isd>\n";

  // Rigorous camera model

  // Lattice points
  ofs << "    <LATTICE_POINT>\n";
  for (std::size_t row = 0; row < lattice_mat.size(); row++) {
    for (std::size_t col = 0; col < lattice_mat[row].size(); col++) {
      ofs << vw::vec_to_str(lattice_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </LATTICE_POINT>\n";

  // Sight vector
  ofs << "    <SIGHT_VECTOR>\n";
  for (std::size_t row = 0; row < sight_mat.size(); row++) {
    for (std::size_t col = 0; col < sight_mat[row].size(); col++) {
      ofs << vw::vec_to_str(sight_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </SIGHT_VECTOR>\n";

  // Sight vector in world coordinates
  ofs << "    <WORLD_SIGHT_VECTOR>\n";
  for (std::size_t row = 0; row < world_sight_mat.size(); row++) {
    for (std::size_t col = 0; col < world_sight_mat[row].size(); col++) {
      ofs << vw::vec_to_str(world_sight_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </WORLD_SIGHT_VECTOR>\n";

  // Satellite position
  ofs << "    <SAT_POS>\n";
  for (std::size_t row = 0; row < sat_pos.size(); row++) {
    ofs << vw::vec_to_str(sat_pos[row]) << std::endl;
  }
  ofs << "    </SAT_POS>\n";

  // Image size
  ofs << "    <IMAGE_COLS>" << image_cols << "</IMAGE_COLS>\n";
  ofs << "    <IMAGE_ROWS>" << image_rows << "</IMAGE_ROWS>\n";

  // RPC
  ofs << "    <RPB>\n";
  ofs << "        <SATID>ASTER_L1A_VNIR_Band3</SATID>\n";
  ofs << "        <IMAGE>\n";
  ofs << "            <LINEOFFSET>"         << lineoffset   << "</LINEOFFSET>\n";
  ofs << "            <SAMPOFFSET>"         << sampoffset   << "</SAMPOFFSET>\n";
  ofs << "            <LATOFFSET>"          << latoffset    << "</LATOFFSET>\n";
  ofs << "            <LONGOFFSET>"         << longoffset   << "</LONGOFFSET>\n";
  ofs << "            <HEIGHTOFFSET>"       << heightoffset << "</HEIGHTOFFSET>\n";
  ofs << "            <LINESCALE>"          << linescale    << "</LINESCALE>\n";
  ofs << "            <SAMPSCALE>"          << sampscale    << "</SAMPSCALE>\n";
  ofs << "            <LATSCALE>"           << latscale     << "</LATSCALE>\n";
  ofs << "            <LONGSCALE>"          << longscale    << "</LONGSCALE>\n";
  ofs << "            <HEIGHTSCALE>"        << heightscale  << "</HEIGHTSCALE>\n";
  ofs << "            <LINENUMCOEFList>\n";
  ofs << "                <LINENUMCOEF>"    << linenumcoef  << "</LINENUMCOEF>\n";
  ofs << "            </LINENUMCOEFList>\n";
  ofs << "            <LINEDENCOEFList>\n";
  ofs << "                <LINEDENCOEF>"    << linedencoef  << "</LINEDENCOEF>\n";
  ofs << "            </LINEDENCOEFList>\n";
  ofs << "            <SAMPNUMCOEFList>\n";
  ofs << "                <SAMPNUMCOEF>"    << sampnumcoef  << "</SAMPNUMCOEF>\n";
  ofs << "            </SAMPNUMCOEFList>\n";
  ofs << "            <SAMPDENCOEFList>\n";
  ofs << "                <SAMPDENCOEF>"    << sampdencoef  << "</SAMPDENCOEF>\n";
  ofs << "            </SAMPDENCOEFList>\n";
  ofs << "        </IMAGE>\n";
  ofs << "    </RPB>\n";
  ofs << "</isd>\n";
  ofs.close();
}

// Create XML files containing rigorous camera info, and compute the RPC
// coefficients as well.
void gen_xml(double min_height, double max_height, std::int64_t num_samples,
             double penalty_weight, std::string const &image_file,
             std::string const &sat_pos_file, std::string const &sight_vec_file,
             std::string const &longitude_file,
             std::string const &latitude_file, std::string const &lattice_file,
             std::string const &out_cam_file) {

  std::vector<std::vector<Vector2>> lattice_mat;
  asp::read_matrix_from_file(lattice_file, lattice_mat);

  std::vector<std::vector<Vector3>> sight_mat;
  asp::read_matrix_from_file(sight_vec_file, sight_mat);

  if (lattice_mat.empty() || sight_mat.empty() ||
      lattice_mat.size() != sight_mat.size() ||
      lattice_mat[0].size() != sight_mat[0].size()) {
    vw_throw(ArgumentErr()
             << "Inconsistent lattice point and sight vector information.\n");
  }

  // Read the satellite positions
  std::vector<Vector3> sat_pos;
  asp::read_3d_points(sat_pos_file, sat_pos);
  if (sat_pos.size() != sight_mat.size())
    vw_throw(
        ArgumentErr()
        << "Inconsistent satellite position and sight vector information.\n");

  Vector3 llh_scale, llh_offset;
  Vector2 pixel_scale, pixel_offset;
  Vector<double> normalized_llh;
  Vector<double> normalized_pixels;
  std::vector<std::vector<vw::Vector3>>
      world_sight_mat;  // sight dir in world coords
  generate_point_pairs(// inputs
                       min_height, max_height, num_samples, penalty_weight, sat_pos_file,
                       sight_vec_file, longitude_file, latitude_file, lattice_file,
                       // Outputs
                       world_sight_mat, llh_scale, llh_offset, pixel_scale, pixel_offset,
                       normalized_llh, normalized_pixels);

  // Find the RPC coefficients
  asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
  bool refine_only = false;
  asp::gen_rpc(// Inputs
               penalty_weight, normalized_llh, normalized_pixels, refine_only,
               // Outputs
               line_num, line_den, samp_num, samp_den);

  DiskImageView<float> input_img(image_file);
  std::int64_t image_cols = input_img.cols();
  std::int64_t image_rows = input_img.rows();

  save_xml(image_cols, image_rows, lattice_mat, sight_mat, world_sight_mat,
           sat_pos, llh_scale, llh_offset, pixel_scale, pixel_offset, line_num,
           line_den, samp_num, samp_den, out_cam_file);
}

// Generate a temporary directory for extracting HDF subdatasets. If the
// output_prefix has a parent directory (e.g., "run/out"), create "run/tmp".
// Otherwise, if it has no parent path (e.g., just "out"), we must create a
// unique directory in the current directory to avoid conflicts when multiple
// instances run.
std::string genTmpDir(std::string const &output_prefix) {
  fs::path prefix_path(output_prefix);
  std::string tmp_dir;
  
  if (prefix_path.has_parent_path()) {
    // output_prefix is like "run/out", make tmp dir "run/tmp"
    tmp_dir = prefix_path.parent_path().string() + "/tmp";
  } else {
    // output_prefix is just "out", make tmp dir unique like "tmp_aster_12345"
    // Use process ID to ensure uniqueness
    tmp_dir = "tmp_aster_" + vw::num_to_str(getpid());
  }
  
  fs::create_directories(tmp_dir);
  return tmp_dir;
}

// Find a subdataset by pattern matching in the HDF metadata.
std::string findSubdataset(char** subdatasets, std::string const& pattern) {
  for (int i = 0; subdatasets[i] != NULL; i++) {
    std::string line(subdatasets[i]);
    if (line.find("_NAME=") != std::string::npos && 
        line.find(pattern) != std::string::npos) {
      // Extract the subdataset path after the "="
      std::size_t pos = line.find("=");
      return line.substr(pos + 1);
    }
  }
  return "";  // Not found
}

// Extract a single ASTER band image from HDF subdatasets to a TIF file.
void extractBandImage(char** subdatasets, std::string const& band_name, 
                      std::string const& tmp_dir) {
  
  // Find the ImageData subdataset for this band
  std::string search_pattern = band_name + ":ImageData";
  std::string band_subdataset = findSubdataset(subdatasets, search_pattern);
  
  if (band_subdataset.empty())
    vw_throw(ArgumentErr() << "Could not find " << band_name << " in the HDF file.\n");
  
  vw_out() << "Found " << band_name << " subdataset: " << band_subdataset << "\n";
  
  // Open the subdataset
  GDALDataset* band_ds = (GDALDataset*)GDALOpen(band_subdataset.c_str(), GA_ReadOnly);
  if (!band_ds)
    vw_throw(ArgumentErr() << "Failed to open " << band_name << " subdataset.\n");
  
  // Write to output TIF
  std::string out_file = tmp_dir + "/AST_L1A_" + band_name + ".ImageData.tif";
  vw_out() << "Extracting " << band_name << " to: " << out_file << "\n";
  
  GDALDriver* gtiff_driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* out_ds = gtiff_driver->CreateCopy(out_file.c_str(), band_ds, 
                                                  FALSE, NULL, NULL, NULL);
  if (!out_ds)
    vw_throw(ArgumentErr() << "Failed to write " << band_name << " to TIF.\n");
  
  GDALClose(out_ds);
  GDALClose(band_ds);
}

// Extract metadata array from HDF subdataset to text file.
// Writes 3D arrays as space-separated values per line.
void extractMetadataArray(char** subdatasets, std::string const& band_name,
                          std::string const& metadata_type,
                          std::string const& tmp_dir) {
  
  // Find the metadata subdataset
  std::string search_pattern = band_name + ":" + metadata_type;
  std::string subdataset_path = findSubdataset(subdatasets, search_pattern);
  
  if (subdataset_path.empty())
    vw_throw(ArgumentErr() << "Could not find " << search_pattern 
                           << " in the HDF file.\n");
  
  vw_out() << "Found " << search_pattern << " subdataset\n";
  
  // Open the subdataset
  GDALDataset* ds = (GDALDataset*)GDALOpen(subdataset_path.c_str(), GA_ReadOnly);
  if (!ds)
    vw_throw(ArgumentErr() << "Failed to open " << search_pattern << " subdataset.\n");
  
  // Get dimensions
  int cols = ds->GetRasterXSize();
  int rows = ds->GetRasterYSize();
  int bands = ds->GetRasterCount();
  
  vw_out() << "Reading " << rows << "x" << cols << "x" << bands << " array\n";
  
  // Read data from all bands
  std::vector<double> data(rows * cols * bands);
  for (int band = 1; band <= bands; band++) {
    GDALRasterBand* raster_band = ds->GetRasterBand(band);
    
    CPLErr err = raster_band->RasterIO(GF_Read, 0, 0, cols, rows,
                                       data.data() + (band - 1) * rows * cols, 
                                       cols, rows, GDT_Float64, 0, 0);
    if (err != CE_None)
      vw_throw(ArgumentErr() << "Failed to read " << search_pattern << " data.\n");
  }
  
  GDALClose(ds);
  
  // Write to text file
  std::string out_file = tmp_dir + "/AST_L1A_" + band_name + "." + metadata_type + ".txt";
  vw_out() << "Writing to: " << out_file << "\n";
  
  std::ofstream ofs(out_file);
  if (!ofs)
    vw_throw(ArgumentErr() << "Failed to open output file: " << out_file << "\n");
  
  // Set high precision for output
  ofs << std::setprecision(17);
  
  // Handle different array structures:
  // - If bands == 1 (like SatellitePosition [12x3x1]): rows=time, cols=XYZ
  //   Write: time1: X Y Z \n time2: X Y Z \n ...
  // - If bands > 1 (like SightVector [11x3x16]): rows=spatial, cols=XYZ, bands=time
  //   Write in time blocks, each with spatial rows of XYZ values
  std::cout << "--rows = " << rows << ", cols = " << cols << ", bands = " << bands << std::endl;
  
  if (bands == 1) {
    // Simple format: each row (time) has all columns (XYZ) on one line
    for (int row = 0; row < rows; row++) {
      for (int col = 0; col < cols; col++) {
        int idx = row * cols + col;
        ofs << data[idx];
        if (col < cols - 1)
          ofs << " ";
      }
      ofs << "\n";
    }
  } else {
    // Block format: each band (time point) is a block containing rows (spatial) lines
    // Each line has all col values (XYZ)
    for (int band = 0; band < bands; band++) {
      for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
          int idx = band * rows * cols + row * cols + col;
          ofs << data[idx];
          if (col < cols - 1)
            ofs << " ";
        }
        ofs << "\n";
      }
      ofs << "\n"; // Blank line between time blocks
    }
  }
  
  ofs.close();
}

// Compute latitude/longitude for lattice points from orbital geometry.
// Transforms sight vectors from local orbital frame to ECEF and ray-traces to ellipsoid.
// Based on orbital mechanics described in ASTER User Guide V4.
void computeLatLonLattice(char** subdatasets, std::string const& band_name,
                          std::string const& tmp_dir) {
  
  vw_out() << "Computing Latitude/Longitude for " << band_name << " lattice points\n";
  
  // WGS84 ellipsoid parameters
  const double a = 6378137.0;           // semi-major axis (m)
  const double b = 6356752.314245;      // semi-minor axis (m)
  
  // Find required subdatasets
  std::string sat_pos_path = findSubdataset(subdatasets, band_name + ":SatellitePosition");
  std::string sat_vel_path = findSubdataset(subdatasets, band_name + ":SatelliteVelocity");
  std::string sight_vec_path = findSubdataset(subdatasets, band_name + ":SightVector");
  
  if (sat_pos_path.empty() || sat_vel_path.empty() || sight_vec_path.empty())
    vw_throw(ArgumentErr() << "Missing required subdatasets for " << band_name << "\n");
  
  // Open and read geometry data
  GDALDataset* pos_ds = (GDALDataset*)GDALOpen(sat_pos_path.c_str(), GA_ReadOnly);
  GDALDataset* vel_ds = (GDALDataset*)GDALOpen(sat_vel_path.c_str(), GA_ReadOnly);
  GDALDataset* sight_ds = (GDALDataset*)GDALOpen(sight_vec_path.c_str(), GA_ReadOnly);
  
  if (!pos_ds || !vel_ds || !sight_ds)
    vw_throw(ArgumentErr() << "Failed to open geometry subdatasets for " << band_name << "\n");
  
  // Get dimensions
  int num_time_steps = sight_ds->GetRasterCount();
  int num_lattice_cols = sight_ds->GetRasterXSize();
  
  // Read all data
  std::vector<double> sat_pos_data(pos_ds->GetRasterXSize() * pos_ds->GetRasterYSize());
  std::vector<double> sat_vel_data(vel_ds->GetRasterXSize() * vel_ds->GetRasterYSize());
  std::vector<double> sight_vec_data(sight_ds->GetRasterXSize() * 
                                      sight_ds->GetRasterYSize() * 
                                      sight_ds->GetRasterCount());
  
  CPLErr err = pos_ds->GetRasterBand(1)->RasterIO(GF_Read, 0, 0,
                                                    pos_ds->GetRasterXSize(), pos_ds->GetRasterYSize(),
                                                    sat_pos_data.data(),
                                                    pos_ds->GetRasterXSize(), pos_ds->GetRasterYSize(),
                                                    GDT_Float64, 0, 0);
  if (err != CE_None)
    vw::vw_throw(vw::IOErr() << "Failed to read satellite position data.\n");

  err = vel_ds->GetRasterBand(1)->RasterIO(GF_Read, 0, 0,
                                            vel_ds->GetRasterXSize(), vel_ds->GetRasterYSize(),
                                            sat_vel_data.data(),
                                            vel_ds->GetRasterXSize(), vel_ds->GetRasterYSize(),
                                            GDT_Float64, 0, 0);
  if (err != CE_None)
    vw::vw_throw(vw::IOErr() << "Failed to read satellite velocity data.\n");

  for (int band = 1; band <= num_time_steps; band++) {
    err = sight_ds->GetRasterBand(band)->RasterIO(GF_Read, 0, 0,
                                                   num_lattice_cols, 3,
                                                   sight_vec_data.data() + (band - 1) * num_lattice_cols * 3,
                                                   num_lattice_cols, 3,
                                                   GDT_Float64, 0, 0);
    if (err != CE_None)
      vw::vw_throw(vw::IOErr() << "Failed to read sight vector data for band " << band << ".\n");
  }
  
  GDALClose(pos_ds);
  GDALClose(vel_ds);
  GDALClose(sight_ds);
  
  // Compute lat/lon for each lattice point
  std::vector<double> lat_lattice(num_time_steps * num_lattice_cols);
  std::vector<double> lon_lattice(num_time_steps * num_lattice_cols);
  
  for (int t = 0; t < num_time_steps; t++) {
    // Get satellite position and velocity for this time step (rows=time, cols=XYZ)
    double sat_pos[3] = {sat_pos_data[t * 3 + 0],
                         sat_pos_data[t * 3 + 1],
                         sat_pos_data[t * 3 + 2]};
    double sat_vel[3] = {sat_vel_data[t * 3 + 0],
                         sat_vel_data[t * 3 + 1],
                         sat_vel_data[t * 3 + 2]};
    
    // Build orbital frame rotation matrix
    // z_orb points from Earth center to satellite (negative of sat_pos direction)
    double pos_norm = std::sqrt(sat_pos[0]*sat_pos[0] + sat_pos[1]*sat_pos[1] + sat_pos[2]*sat_pos[2]);
    double z_orb[3] = {-sat_pos[0]/pos_norm, -sat_pos[1]/pos_norm, -sat_pos[2]/pos_norm};
    
    // Orbit normal = sat_pos cross sat_vel
    double orbit_normal[3] = {sat_pos[1]*sat_vel[2] - sat_pos[2]*sat_vel[1],
                              sat_pos[2]*sat_vel[0] - sat_pos[0]*sat_vel[2],
                              sat_pos[0]*sat_vel[1] - sat_pos[1]*sat_vel[0]};
    double normal_norm = std::sqrt(orbit_normal[0]*orbit_normal[0] + 
                                    orbit_normal[1]*orbit_normal[1] + 
                                    orbit_normal[2]*orbit_normal[2]);
    double y_orb[3] = {-orbit_normal[0]/normal_norm,
                       -orbit_normal[1]/normal_norm,
                       -orbit_normal[2]/normal_norm};
    
    // x_orb = y_orb cross z_orb
    double x_orb[3] = {y_orb[1]*z_orb[2] - y_orb[2]*z_orb[1],
                       y_orb[2]*z_orb[0] - y_orb[0]*z_orb[2],
                       y_orb[0]*z_orb[1] - y_orb[1]*z_orb[0]};
    
    // Process each lattice column
    for (int c = 0; c < num_lattice_cols; c++) {
      // Get sight vector in orbital frame (data is [time][col][xyz])
      int idx = t * num_lattice_cols * 3 + c * 3;
      double sv_orb[3] = {sight_vec_data[idx + 0],
                          sight_vec_data[idx + 1],
                          sight_vec_data[idx + 2]};
      
      double sv_norm = std::sqrt(sv_orb[0]*sv_orb[0] + sv_orb[1]*sv_orb[1] + sv_orb[2]*sv_orb[2]);
      if (sv_norm < 1e-10)
        continue;
      
      // Transform sight vector from orbital frame to ECEF
      double sv_ecef[3] = {x_orb[0]*sv_orb[0] + y_orb[0]*sv_orb[1] + z_orb[0]*sv_orb[2],
                           x_orb[1]*sv_orb[0] + y_orb[1]*sv_orb[1] + z_orb[1]*sv_orb[2],
                           x_orb[2]*sv_orb[0] + y_orb[2]*sv_orb[1] + z_orb[2]*sv_orb[2]};
      
      // Normalize
      double sv_ecef_norm = std::sqrt(sv_ecef[0]*sv_ecef[0] + sv_ecef[1]*sv_ecef[1] + sv_ecef[2]*sv_ecef[2]);
      sv_ecef[0] /= sv_ecef_norm;
      sv_ecef[1] /= sv_ecef_norm;
      sv_ecef[2] /= sv_ecef_norm;
      
      // Ray-ellipsoid intersection: find t where ray hits WGS84 ellipsoid
      // Ray: P = sat_pos + t * sv_ecef
      // Ellipsoid: (x/a)^2 + (y/a)^2 + (z/b)^2 = 1
      double A = (sv_ecef[0]/a)*(sv_ecef[0]/a) + (sv_ecef[1]/a)*(sv_ecef[1]/a) + (sv_ecef[2]/b)*(sv_ecef[2]/b);
      double B = 2*((sat_pos[0]*sv_ecef[0]/(a*a)) + (sat_pos[1]*sv_ecef[1]/(a*a)) + (sat_pos[2]*sv_ecef[2]/(b*b)));
      double C = (sat_pos[0]/a)*(sat_pos[0]/a) + (sat_pos[1]/a)*(sat_pos[1]/a) + (sat_pos[2]/b)*(sat_pos[2]/b) - 1;
      
      double disc = B*B - 4*A*C;
      if (disc < 0)
        continue;
      
      double t_param = (-B - std::sqrt(disc)) / (2*A);
      if (t_param <= 0)
        continue;
      
      // Ground intersection point
      double gx = sat_pos[0] + t_param * sv_ecef[0];
      double gy = sat_pos[1] + t_param * sv_ecef[1];
      double gz = sat_pos[2] + t_param * sv_ecef[2];
      
      // Convert to geocentric lat/lon
      double p = std::sqrt(gx*gx + gy*gy);
      lon_lattice[t * num_lattice_cols + c] = std::atan2(gy, gx) * 180.0 / M_PI;
      lat_lattice[t * num_lattice_cols + c] = std::atan2(gz, p) * 180.0 / M_PI;
    }
  }
  
  // Write latitude file
  std::string lat_file = tmp_dir + "/AST_L1A_" + band_name + ".Latitude.txt";
  std::ofstream lat_ofs(lat_file);
  if (!lat_ofs)
    vw_throw(ArgumentErr() << "Failed to open output file: " << lat_file << "\n");
  
  lat_ofs << std::setprecision(17);
  for (int t = 0; t < num_time_steps; t++) {
    for (int c = 0; c < num_lattice_cols; c++) {
      lat_ofs << lat_lattice[t * num_lattice_cols + c];
      if (c < num_lattice_cols - 1)
        lat_ofs << " ";
    }
    lat_ofs << "\n";
  }
  lat_ofs.close();
  vw_out() << "Wrote: " << lat_file << "\n";
  
  // Write longitude file
  std::string lon_file = tmp_dir + "/AST_L1A_" + band_name + ".Longitude.txt";
  std::ofstream lon_ofs(lon_file);
  if (!lon_ofs)
    vw_throw(ArgumentErr() << "Failed to open output file: " << lon_file << "\n");
  
  lon_ofs << std::setprecision(17);
  for (int t = 0; t < num_time_steps; t++) {
    for (int c = 0; c < num_lattice_cols; c++) {
      lon_ofs << lon_lattice[t * num_lattice_cols + c];
      if (c < num_lattice_cols - 1)
        lon_ofs << " ";
    }
    lon_ofs << "\n";
  }
  lon_ofs.close();
  vw_out() << "Wrote: " << lon_file << "\n";
}

// Extract data from HDF file to temporary directory. Updates opt.input to point
// to the temp directory and sets tmp_dir, which we will wipe when not needed.
void extractHdfData(Options& opt, std::string& tmp_dir) {
  vw_out() << "Reading HDF file: " << opt.input << "\n";
  tmp_dir = genTmpDir(opt.output_prefix);
  vw_out() << "Creating directory " << tmp_dir 
           << " for extracting data from HDF.\n";
  
  // Open HDF file with GDAL
  GDALAllRegister();
  GDALDataset* hdf_ds = (GDALDataset*)GDALOpen(opt.input.c_str(), GA_ReadOnly);
  if (!hdf_ds)
    vw_throw(ArgumentErr() << "Failed to open HDF file: " << opt.input << "\n");
  
  // Get subdatasets
  char** subdatasets = hdf_ds->GetMetadata("SUBDATASETS");
  if (!subdatasets)
    vw_throw(ArgumentErr() << "No subdatasets found in HDF file.\n");
  
  // In the HDF4_EOS format, each VNIR band has its own subdatasets:
  // VNIR_Band3N:ImageData, VNIR_Band3N:SatellitePosition, etc.
  // VNIR_Band3B:ImageData, VNIR_Band3B:SatellitePosition, etc.
  
  // Extract both band images
  extractBandImage(subdatasets, "VNIR_Band3N", tmp_dir);
  extractBandImage(subdatasets, "VNIR_Band3B", tmp_dir);
  
  // Extract satellite position metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "SatellitePosition", tmp_dir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "SatellitePosition", tmp_dir);
  
  // Extract sight vector metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "SightVector", tmp_dir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "SightVector", tmp_dir);
  
  // Extract lattice point metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "LatticePoint", tmp_dir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "LatticePoint", tmp_dir);
  
  // Extract radiometric correction table for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "RadiometricCorrTable", tmp_dir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "RadiometricCorrTable", tmp_dir);
  
  // Compute latitude/longitude from orbital geometry
  // V004 HDF does not include these as subdatasets - they must be computed from
  // SatellitePosition, SatelliteVelocity, and SightVector
  computeLatLonLattice(subdatasets, "VNIR_Band3N", tmp_dir);
  computeLatLonLattice(subdatasets, "VNIR_Band3B", tmp_dir);
  
  GDALClose(hdf_ds);
  
  vw_out() << "HDF extraction complete.\n";
  
  // Point the input to the temp directory so the rest of the code can find the files
  opt.input = tmp_dir;
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Track if we created a temp directory (for cleanup later)
    std::string tmp_dir = "";

    // Check if input is an HDF file and extract data if so
    if (boost::iends_with(opt.input, ".hdf"))
      extractHdfData(opt, tmp_dir);

    std::string nadir_image, back_image, nadir_sat_pos, back_sat_pos;
    std::string nadir_sight_vec, back_sight_vec;
    std::string nadir_corr_table, back_corr_table;
    std::string nadir_longitude, back_longitude;
    std::string nadir_latitude, back_latitude;
    std::string nadir_lattice_point, back_lattice_point;

    locate_inputs(opt, nadir_image, back_image, nadir_sat_pos, back_sat_pos,
                  nadir_sight_vec, back_sight_vec, nadir_corr_table,
                  back_corr_table, nadir_longitude, back_longitude,
                  nadir_latitude, back_latitude, nadir_lattice_point,
                  back_lattice_point);

    std::string out_nadir_image = opt.output_prefix + "-Band3N.tif";
    std::string out_back_image = opt.output_prefix + "-Band3B.tif";

    std::string out_nadir_cam = opt.output_prefix + "-Band3N.xml";
    std::string out_back_cam = opt.output_prefix + "-Band3B.xml";

    // Oddly, writing the xml files first, with output prefix "run", results in
    // images later wiping the xml camera files. So write the images before the
    // xml files.
    apply_radiometric_corrections(opt, nadir_image, nadir_corr_table,
                                  out_nadir_image);
    apply_radiometric_corrections(opt, back_image, back_corr_table,
                                  out_back_image);

    vw_out() << "Computing the camera models.\n";

    gen_xml(opt.min_height, opt.max_height, opt.num_samples, opt.penalty_weight,
            nadir_image, nadir_sat_pos, nadir_sight_vec, nadir_longitude,
            nadir_latitude, nadir_lattice_point, out_nadir_cam);

    gen_xml(opt.min_height, opt.max_height, opt.num_samples, opt.penalty_weight,
            back_image, back_sat_pos, back_sight_vec, back_longitude,
            back_latitude, back_lattice_point, out_back_cam);
    
    // Clean up temporary directory if we created one
    if (!tmp_dir.empty()) {
      if (opt.keep_tmp_dir) {
        vw_out() << "Keeping temporary directory for debugging: " << tmp_dir << "\n";
      } else {
        vw_out() << "Removing temporary directory: " << tmp_dir << "\n";
        fs::remove_all(tmp_dir);
      }
    }
  }
  ASP_STANDARD_CATCHES;

  return 0;
}
