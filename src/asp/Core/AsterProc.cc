// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Core/AsterProc.h>
#include <asp/Core/FileUtils.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Core/StringUtils.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography/GeoReferenceUtils.h>

#include <boost/filesystem.hpp>
#include <gdal_priv.h>

#include <fstream>
#include <iomanip>
#include <cmath>

namespace fs = boost::filesystem;

namespace asp {

namespace {

// Internal helper functions for HDF extraction

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
void extractBandImage(char** subdatasets,
                      std::string const& band_name,
                      std::string const& tmp_dir) {

  // Find the ImageData subdataset for this band
  std::string search_pattern = band_name + ":ImageData";
  std::string band_subdataset = findSubdataset(subdatasets, search_pattern);

  if (band_subdataset.empty())
    vw::vw_throw(vw::ArgumentErr() << "Could not find " << band_name
                                   << " in the HDF file.\n");

  // Open the subdataset
  GDALDataset* band_ds = (GDALDataset*)GDALOpen(band_subdataset.c_str(), GA_ReadOnly);
  if (!band_ds)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open " << band_name << " subdataset.\n");

  // Write to output TIF
  std::string out_file = tmp_dir + "/AST_L1A_" + band_name + ".ImageData.tif";
  vw::vw_out() << "Extracting " << band_name << " to: " << out_file << "\n";

  GDALDriver* gtiff_driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  GDALDataset* out_ds = gtiff_driver->CreateCopy(out_file.c_str(), band_ds,
                                                  FALSE, NULL, NULL, NULL);
  if (!out_ds)
    vw::vw_throw(vw::ArgumentErr() << "Failed to write " << band_name << " to TIF.\n");

  GDALClose(out_ds);
  GDALClose(band_ds);
}

// Extract metadata array from HDF subdataset to text file.
// Writes 3D arrays as space-separated values per line.
void extractMetadataArray(char** subdatasets,
                          std::string const& band_name,
                          std::string const& metadata_type,
                          std::string const& tmp_dir) {

  // Find the metadata subdataset
  std::string search_pattern = band_name + ":" + metadata_type;
  std::string subdataset_path = findSubdataset(subdatasets, search_pattern);

  if (subdataset_path.empty())
    vw::vw_throw(vw::ArgumentErr() << "Could not find " << search_pattern
                           << " in the HDF file.\n");

  // Open the subdataset
  GDALDataset* ds = (GDALDataset*)GDALOpen(subdataset_path.c_str(), GA_ReadOnly);
  if (!ds)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open " << search_pattern
                                   << " subdataset.\n");

  // Get dimensions
  int cols = ds->GetRasterXSize();
  int rows = ds->GetRasterYSize();
  int bands = ds->GetRasterCount();

  // Read data from all bands
  std::vector<double> data(rows * cols * bands);
  for (int band = 1; band <= bands; band++) {
    GDALRasterBand* raster_band = ds->GetRasterBand(band);

    CPLErr err = raster_band->RasterIO(GF_Read, 0, 0, cols, rows,
                                       data.data() + (band - 1) * rows * cols,
                                       cols, rows, GDT_Float64, 0, 0);
    if (err != CE_None)
      vw::vw_throw(vw::ArgumentErr() << "Failed to read " << search_pattern
                                     << " data.\n");
  }

  GDALClose(ds);

  // Write to text file
  std::string out_file = tmp_dir + "/AST_L1A_" + band_name + "." + metadata_type + ".txt";

  std::ofstream ofs(out_file);
  if (!ofs)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open output file: " << out_file << "\n");

  // Set high precision for output
  ofs << std::setprecision(17);

  // Handle different array structures:
  // - If bands == 1 (like SatellitePosition [12x3x1]): rows=time, cols=XYZ
  //   Write: time1: X Y Z \n time2: X Y Z \n ...
  // - If bands > 1 (like SightVector [11x3x16]): rows=spatial, cols=XYZ, bands=time
  //   Write in time blocks, each with spatial rows of XYZ values

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
    // Complex format: write data grouped by band (time step)
    // For each time step (band), write all spatial rows with XYZ columns
    for (int band = 1; band <= bands; band++) {
      int band_offset = (band - 1) * rows * cols;
      for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
          int idx = band_offset + row * cols + col;
          ofs << data[idx];
          if (col < cols - 1)
            ofs << " ";
        }
        ofs << "\n";
      }
      // Add blank line between time steps (except after last one)
      if (band < bands)
        ofs << "\n";
    }
  }

  ofs.close();
}

// Compute latitude and longitude for lattice points from orbital geometry. V004
// HDF format does not include Lat/Lon datasets - they must be computed from
// SatellitePosition, SatelliteVelocity, and SightVector using ray-ellipsoid
// intersection.
void computeLatLonLattice(char** subdatasets,
                          std::string const& band_name,
                          std::string const& tmp_dir) {

  // Find required subdatasets
  std::string pos_path = findSubdataset(subdatasets, band_name + ":SatellitePosition");
  std::string vel_path = findSubdataset(subdatasets, band_name + ":SatelliteVelocity");
  std::string sight_vec_path = findSubdataset(subdatasets, band_name + ":SightVector");

  if (pos_path.empty() || vel_path.empty() || sight_vec_path.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing required geometry datasets for "
                                   << band_name << "\n");

  // Open datasets
  GDALDataset* pos_ds = (GDALDataset*)GDALOpen(pos_path.c_str(), GA_ReadOnly);
  GDALDataset* vel_ds = (GDALDataset*)GDALOpen(vel_path.c_str(), GA_ReadOnly);
  GDALDataset* sight_ds = (GDALDataset*)GDALOpen(sight_vec_path.c_str(), GA_ReadOnly);

  if (!pos_ds || !vel_ds || !sight_ds)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open geometry subdatasets for "
                                   << band_name << "\n");

  // Get dimensions
  // SightVector has shape: RasterCount=time_steps, YSize=lattice_cols, XSize=3 (XYZ)
  int num_time_steps = sight_ds->GetRasterCount();
  int num_lattice_cols = sight_ds->GetRasterYSize();

  // Read all data
  std::vector<double> sat_pos_data(pos_ds->GetRasterXSize() * pos_ds->GetRasterYSize());
  std::vector<double> sat_vel_data(vel_ds->GetRasterXSize() * vel_ds->GetRasterYSize());
  std::vector<double> sight_vec_data(sight_ds->GetRasterXSize() *
                                      sight_ds->GetRasterYSize() *
                                      sight_ds->GetRasterCount());

  CPLErr err = pos_ds->GetRasterBand(1)->RasterIO(GF_Read, 0, 0,
                                                   pos_ds->GetRasterXSize(),
                                                   pos_ds->GetRasterYSize(),
                                                   sat_pos_data.data(),
                                                   pos_ds->GetRasterXSize(),
                                                   pos_ds->GetRasterYSize(),
                                                   GDT_Float64, 0, 0);
  if (err != CE_None)
    vw::vw_throw(vw::IOErr() << "Failed to read satellite position data.\n");

  err = vel_ds->GetRasterBand(1)->RasterIO(GF_Read, 0, 0,
                                           vel_ds->GetRasterXSize(),
                                           vel_ds->GetRasterYSize(),
                                           sat_vel_data.data(),
                                           vel_ds->GetRasterXSize(),
                                           vel_ds->GetRasterYSize(),
                                           GDT_Float64, 0, 0);
  if (err != CE_None)
    vw::vw_throw(vw::IOErr() << "Failed to read satellite velocity data.\n");

  for (int band = 1; band <= num_time_steps; band++) {
    err = sight_ds->GetRasterBand(band)->RasterIO(GF_Read, 0, 0, 3, num_lattice_cols,
                                                   sight_vec_data.data() +
                                                     (band - 1) * num_lattice_cols * 3,
                                                   3, num_lattice_cols,
                                                   GDT_Float64, 0, 0);
    if (err != CE_None)
      vw::vw_throw(vw::IOErr() << "Failed to read sight vector data for band "
                               << band << ".\n");
  }

  GDALClose(pos_ds);
  GDALClose(vel_ds);
  GDALClose(sight_ds);

  // Compute lat/lon for each lattice point using WGS84 datum
  vw::cartography::Datum wgs84;
  wgs84.set_well_known_datum("WGS84");

  std::vector<double> lat_lattice(num_time_steps * num_lattice_cols);
  std::vector<double> lon_lattice(num_time_steps * num_lattice_cols);

  // For each time step
  for (int t = 0; t < num_time_steps; t++) {
    // Get satellite position and velocity for this time step as vectors
    vw::Vector3 sat_pos(sat_pos_data[t * 3 + 0],
                        sat_pos_data[t * 3 + 1],
                        sat_pos_data[t * 3 + 2]);
    vw::Vector3 sat_vel(sat_vel_data[t * 3 + 0],
                        sat_vel_data[t * 3 + 1],
                        sat_vel_data[t * 3 + 2]);

    // Build orbital frame rotation matrix using VW vector utilities
    // z_orb = -normalized(sat_pos) [geocentric nadir direction]
    vw::Vector3 z_orb = -normalize(sat_pos);

    // orbit_normal = sat_pos x sat_vel
    vw::Vector3 orbit_normal = cross_prod(sat_pos, sat_vel);

    // y_orb = -normalized(orbit_normal) [cross-track direction]
    vw::Vector3 y_orb = -normalize(orbit_normal);

    // x_orb = y_orb x z_orb [along-track direction]
    vw::Vector3 x_orb = cross_prod(y_orb, z_orb);

    // Build rotation matrix R = [x_orb | y_orb | z_orb]
    vw::Matrix<double,3,3> R_orb_to_ecef;
    select_col(R_orb_to_ecef, 0) = x_orb;
    select_col(R_orb_to_ecef, 1) = y_orb;
    select_col(R_orb_to_ecef, 2) = z_orb;

    // For each lattice column
    for (int c = 0; c < num_lattice_cols; c++) {
      // Get sight vector in orbital frame
      int idx = t * num_lattice_cols * 3 + c * 3;
      vw::Vector3 sv_orb(sight_vec_data[idx + 0],
                         sight_vec_data[idx + 1],
                         sight_vec_data[idx + 2]);

      // Check for zero vector
      if (norm_2(sv_orb) < 1e-10) {
        lat_lattice[t * num_lattice_cols + c] = 0.0;
        lon_lattice[t * num_lattice_cols + c] = 0.0;
        continue;
      }

      // Transform sight vector from orbital frame to ECEF
      vw::Vector3 sv_ecef = R_orb_to_ecef * sv_orb;

      // Ray-datum intersection using VW's proven implementation
      vw::Vector3 ground_pt = vw::cartography::datum_intersection(wgs84, sat_pos, sv_ecef);

      // Check for failed intersection (returns zero vector)
      if (ground_pt == vw::Vector3()) {
        lat_lattice[t * num_lattice_cols + c] = 0.0;
        lon_lattice[t * num_lattice_cols + c] = 0.0;
        continue;
      }

      // Convert ECEF ground point to geocentric lat/lon
      // NOTE: We intentionally use geocentric (not geodetic) coordinates here
      // to match the V003 format convention. The conversion from geocentric to
      // geodetic happens later in aster2asp.cc for both V003 and V004 data,
      // ensuring consistent downstream processing.
      double p = std::sqrt(ground_pt.x() * ground_pt.x() +
                          ground_pt.y() * ground_pt.y());
      lon_lattice[t * num_lattice_cols + c] 
        = std::atan2(ground_pt.y(), ground_pt.x()) * 180.0 / M_PI;
      lat_lattice[t * num_lattice_cols + c] = std::atan2(ground_pt.z(), p) * 180.0 / M_PI;
    }
  }

  // Write latitude file
  std::string lat_file = tmp_dir + "/AST_L1A_" + band_name + ".Latitude.txt";
  vw::vw_out() << "Writing: " << lat_file << "\n";
  std::ofstream lat_ofs(lat_file);
  if (!lat_ofs)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open output file: " << lat_file << "\n");

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

  // Write longitude file
  std::string lon_file = tmp_dir + "/AST_L1A_" + band_name + ".Longitude.txt";
  vw::vw_out() << "Writing: " << lon_file << "\n";
  std::ofstream lon_ofs(lon_file);
  if (!lon_ofs)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open output file: " << lon_file << "\n");

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
}

// Internal implementation detail for radiometric corrections
class RadioCorrectView: public vw::ImageViewBase<RadioCorrectView> {
  vw::DiskImageView<float> m_img;
  std::vector<vw::Vector3> const &m_corr;
  bool m_has_nodata;
  double m_nodata;

public:
  RadioCorrectView(vw::DiskImageView<float> const &img,
                   std::vector<vw::Vector3> const &corr,
                   bool has_nodata, double nodata):
  m_img(img), m_corr(corr), m_has_nodata(has_nodata), m_nodata(nodata) {}

  typedef float pixel_type;
  typedef float result_type;
  typedef vw::ProceduralPixelAccessor<RadioCorrectView> pixel_accessor;

  inline vw::int32 cols() const { return m_img.cols(); }
  inline vw::int32 rows() const { return m_img.rows(); }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double /*i*/, double /*j*/, vw::int32 /*p*/ = 0) const {
    vw::vw_throw(vw::NoImplErr()
             << "RadioCorrectView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef vw::CropView<vw::ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const &bbox) const {
    vw::ImageView<float> input_tile = vw::crop(m_img, bbox);
    vw::ImageView<float> tile(bbox.width(), bbox.height());
    for (std::int64_t col = bbox.min().x(); col < bbox.max().x(); col++) {
      vw::Vector3 C = m_corr[col];
      for (std::int64_t row = bbox.min().y(); row < bbox.max().y(); row++) {
        float val = input_tile(col - bbox.min().x(), row - bbox.min().y());
        if (m_has_nodata && val == m_nodata)
          tile(col - bbox.min().x(), row - bbox.min().y()) = val;
        else
          tile(col - bbox.min().x(), row - bbox.min().y()) =
              C[1] * val / C[2] + C[0];
      }
    }
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(), cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const &dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

// Helper function to create RadioCorrectView
RadioCorrectView radioCorrect(vw::DiskImageView<float> const &img,
                               std::vector<vw::Vector3> const &corr,
                               bool has_nodata, double nodata) {
  return RadioCorrectView(img, corr, has_nodata, nodata);
}

} // anonymous namespace

// Public API functions

// Generate a temporary directory for HDF extraction. If the output_prefix has a parent
// path (e.g., "run/out"), create "run/tmp". Otherwise, create a unique directory.
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

// Extract data from HDF file to temporary directory.
void extractHdfData(std::string const& hdf_file, std::string const& hdfOutDir) {
  // Open HDF file with GDAL
  GDALAllRegister();
  GDALDataset* hdf_ds = (GDALDataset*)GDALOpen(hdf_file.c_str(), GA_ReadOnly);
  if (!hdf_ds)
    vw::vw_throw(vw::ArgumentErr() << "Failed to open HDF file: " << hdf_file << "\n");

  // Get subdatasets
  char** subdatasets = hdf_ds->GetMetadata("SUBDATASETS");
  if (!subdatasets)
    vw::vw_throw(vw::ArgumentErr() << "No subdatasets found in HDF file.\n");

  // In the HDF4_EOS format, each VNIR band has its own subdatasets:
  // VNIR_Band3N:ImageData, VNIR_Band3N:SatellitePosition, etc.
  // VNIR_Band3B:ImageData, VNIR_Band3B:SatellitePosition, etc.

  // Extract both band images
  extractBandImage(subdatasets, "VNIR_Band3N", hdfOutDir);
  extractBandImage(subdatasets, "VNIR_Band3B", hdfOutDir);

  // Extract satellite position metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "SatellitePosition", hdfOutDir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "SatellitePosition", hdfOutDir);

  // Extract sight vector metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "SightVector", hdfOutDir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "SightVector", hdfOutDir);

  // Extract lattice point metadata for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "LatticePoint", hdfOutDir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "LatticePoint", hdfOutDir);

  // Extract radiometric correction table for both bands
  extractMetadataArray(subdatasets, "VNIR_Band3N", "RadiometricCorrTable", hdfOutDir);
  extractMetadataArray(subdatasets, "VNIR_Band3B", "RadiometricCorrTable", hdfOutDir);

  // Compute latitude/longitude from orbital geometry
  // V004 HDF does not include these as subdatasets - they must be computed from
  // SatellitePosition, SatelliteVelocity, and SightVector
  computeLatLonLattice(subdatasets, "VNIR_Band3N", hdfOutDir);
  computeLatLonLattice(subdatasets, "VNIR_Band3B", hdfOutDir);

  GDALClose(hdf_ds);
}

// ASTER L1A images come with radiometric corrections appended, but not applied.
// There is one correction per image column.
void applyRadiometricCorrections(std::string const& input_image,
                                 std::string const& corr_table,
                                 std::string const& out_image,
                                 vw::GdalWriteOptions const& opt) {

  // Extract the corrections
  std::vector<vw::Vector3> corr;
  asp::read_3d_points(corr_table, corr);

  vw::DiskImageView<float> input_img(input_image);

  // Sanity check
  if (input_img.cols() != int(corr.size()))
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many corrections in " << corr_table
                                   << " as image columns in " << input_image << "\n");

  // Read nodata value from the image
  boost::shared_ptr<vw::DiskImageResource> img_rsrc(
      new vw::DiskImageResourceGDAL(input_image));
  bool has_nodata = img_rsrc->has_nodata_read();
  double nodata = has_nodata ? img_rsrc->nodata_read() :
                  -std::numeric_limits<float>::max();

  // No georef, this being L1A imagery
  vw::cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, input_image);
  if (has_georef)
    vw::vw_throw(vw::ArgumentErr()
                 << "ASTER L1A images are not supposed to be georeferenced.\n");

  // Apply radiometric corrections and write output
  vw::vw_out() << "Writing: " << out_image << std::endl;
  vw::TerminalProgressCallback tpc("asp", "\t-->: ");
  vw::ImageViewRef<float> corrected_img = radioCorrect(input_img, corr, has_nodata, nodata);
  vw::cartography::block_write_gdal_image(out_image, corrected_img, has_georef,
                                          georef, has_nodata, nodata, opt, tpc);
}

} // namespace asp
