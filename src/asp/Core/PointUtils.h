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

/// \file PointUtils.h

// Utilities for handling point cloud files (las, PC.tif, csv).

#ifndef __ASP_CORE_POINT_UTILS_H__
#define __ASP_CORE_POINT_UTILS_H__

#include <asp/Core/GdalUtils.h>

#include <vw/Core/Functors.h>
#include <vw/Image/PerPixelViews.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/StringUtils.h>

#include <string>

namespace vw{
  namespace cartography {
    class Datum;
    class GeoReference;
  }
}

const int ASP_POINT_CLOUD_TILE_LEN = 2048;
const int ASP_MAX_SUBBLOCK_SIZE = 128;

namespace asp {

  class BaseOptions;

  /// A Data structure which converts from CSV to Cartesian and vice-versa.
  class CsvConv {

  public: // Definitions

    /// Any valid input file must contain one of these sets of fields
    /// - Note that the data is not stored in this order internally.
    enum CsvFormat{
      XYZ, HEIGHT_LAT_LON, LAT_LON_RADIUS_M,
      LAT_LON_RADIUS_KM, EASTING_HEIGHT_NORTHING, PIXEL_XYVAL};

    /// Object used to store the data parsed from a CSV line.
    struct CsvRecord{
      vw::Vector3 point_data;
      std::string file;
    };

  public: // Functions

    /// Default Constructor, the object is not ready to use.
    CsvConv() : format(XYZ), utm_zone(-1), utm_north(false),
                num_fields(0) {}

    bool      is_configured() const {return csv_format_str != "";}
    CsvFormat get_format   () const {return format;}

    /// Writes out a header string containing each of the extracted column names
    /// in the order they were specified.
    std::string write_header_string(std::string const delimiter = ", ") const;

    /// Initialize using a pair of CSV format strings
    void parse_csv_format(std::string const& csv_format_str,
                          std::string const& csv_srs,
                          int min_num_fields = 3);

    /// If the user passed in a csv file containing easting, northing, height
    /// above datum, and either a utm zone or a custom proj4 string,
    /// pass that info into the georeference for the purpose of converting
    /// later from easting and northing to lon and lat.
    bool parse_georef(vw::cartography::GeoReference & georef) const;

    /// Extract values we care about from a csv string and pack them in a vector.
    /// - This extracts the values in the same order they appear in the file.
    /// - Use one of the conversion functions to get coordinates from this vector.
    CsvRecord parse_csv_line(bool & is_first_line, bool & success,
                              std::string const& line) const;

    /// Reads an entire CSV file and stores a record for each line.
    /// - Intended for use with smaller files.
    size_t read_csv_file(std::string const    & file_path,
                         std::list<CsvRecord> & output_list) const;
      
    /// Reads an entire CSV file having polygons. Individual
    /// polygons are separated by a newline or some other unexpected text.
    size_t read_poly_file(std::string     const   & file_path,
                         std::list<CsvRecord>     & output_list,
                         std::vector<int>         & contiguous_blocks,
                         std::vector<std::string> & colors) const;
      
    /// Convert values read from a csv file using parse_csv_line (in
    /// the same order they appear in the file) to a Cartesian
    /// point. If return_point_height is true, and the csv point is
    /// not in xyz format, return instead the projected point and
    /// height above datum.
    vw::Vector3 csv_to_cartesian_or_point_height(CsvRecord const& csv,
                                                 vw::cartography::GeoReference const& geo,
                                                 bool return_point_height) const;

    /// Convert values read from a csv file using parse_csv_line to a Cartesian point.
    vw::Vector3 csv_to_cartesian(CsvRecord const& csv,
                                 vw::cartography::GeoReference const& geo) const;

    /// Convert values read from a csv file using parse_csv_line to a lon/lat/height point.
    vw::Vector3 csv_to_geodetic(CsvRecord const& csv,
                                vw::cartography::GeoReference const& geo) const;

    /// Convert values read from a csv file using parse_csv_line to a lon/lat point.
    vw::Vector2 csv_to_lonlat(CsvRecord const& csv,
                              vw::cartography::GeoReference const& geo) const;

    /// Extracts the file name read from the csv line, or "" if it was not present.
    std::string file_from_csv(CsvRecord const& csv) const {return csv.file;}

    /// Re-order an xyz point so the values appear in the format and order they
    /// did in the csv file. This function does nothing about all the fields we
    /// ignored when we read the file.
    vw::Vector3 cartesian_to_csv(vw::Vector3 const& xyz,
                                 vw::cartography::GeoReference const& geo,
                                 double mean_longitude) const;

    /// Converts the point data vector from the order the fields appeared in the file
    ///  to the order according to the "format" member variable.
    vw::Vector3 sort_parsed_vector3(CsvRecord const& v) const;

    /// Performs the reverse of sort_parsed_vector3, putting the values in the
    /// order that they originally appeared in the file (ignores the file
    /// field).
    vw::Vector3 unsort_vector3(vw::Vector3 const& v) const;

  private: // Variables
    std::map<std::string,int>  name2col; ///< Target names -> Column index in input csv
    std::map<int, std::string> col2name; ///< Target column in input csv -> Name
    std::map<int, int>         col2sort; ///< Which input columns went in which vector indices (numbers only)

    std::string csv_format_str;
    std::string csv_srs;
    CsvFormat   format;
    int         utm_zone;
    bool        utm_north;
    int         num_fields; ///< The number of elements to extract from each CSV line
    
    friend class CsvReader;

  private: // Functions

    /// This function hard-codes the location in the parsed vector where each
    ///  column type will go.
    static int get_sorted_index_for_name(std::string const& name);

  }; // End class CsvConv

  bool is_tif(std::string const& file); ///< Return true if this is a TIF file
  bool is_las(std::string const& file); ///< Return true if this is a LAS file
  bool is_csv(std::string const& file); ///< Return true if this is a CSV file
  bool is_pcd(std::string const& file); ///< Return true if this is a PCD file
  bool is_las_or_csv_or_pcd(std::string const& file);

  /// Builds a datum object out of the input arguments
  bool read_user_datum(double semi_major, double semi_minor,
                       std::string const& reference_spheroid,
                       vw::cartography::Datum& datum );

  /// Parse a UTM string such as "58N"
  void parse_utm_str(std::string const& utm, int & zone, bool & north);

  /// CSV separator
  inline std::string csv_separator() { return ", \t"; }

  /// Need this for pc_align and point2dem
  inline std::string csv_opt_caption() {
    return "Specify the format of input CSV files as a list of entries column_index:column_type (indices start from 1). Examples: '1:x 2:y 3:z', '2:file 5:lon 6:lat 7:radius_m', '3:lat 2:lon 1:height_above_datum 5:file', '1:easting 2:northing 3:height_above_datum' (need to set --csv-srs). Can also use radius_km for column_type.";
  }

  /// A valid line is not empty and does not start with '#'.
  bool is_valid_csv_line(std::string const& line);

  /// If a line has spaces only
  bool hasSpacesOnly(std::string const& str);
  
  /// Returns the number of points contained in a CSV file
  std::int64_t csv_file_size(std::string const& file);

  /// Returns the number of points contained in a PCD file
  std::int64_t pcd_file_size(std::string const& file);

  // Peek at the first valid line in a file to find how many columns it has
  int fileNumCols(std::string const& file);
  
  /// Erases a file suffix if one exists and returns the base string
  std::string prefix_from_pointcloud_filename(std::string const& filename);

  /// Read a point cloud file in the format written by ASP.
  /// Given a point cloud with n channels, return the first m channels.
  /// We must have 1 <= m <= n <= 6.
  /// If the image was written by subtracting a shift, put that shift back.
  template<int m>
  vw::ImageViewRef<vw::Vector<double, m>> read_asp_point_cloud(std::string const& filename);

  /// Hide these functions from external users
  namespace point_utils_private {

    // These two functions choose between two possible inputs for the
    // form_point_cloud_composite function.

    /// Read a texture file
    template<class PixelT>
    typename boost::enable_if<boost::is_same<PixelT, vw::PixelGray<float>>, vw::ImageViewRef<PixelT>>::type
    read_point_cloud_compatible_file(std::string const& file) {
      return vw::DiskImageView<PixelT>(file);
    }
    /// Read a point cloud file
    template<class PixelT>
    typename boost::disable_if<boost::is_same<PixelT, vw::PixelGray<float>>, vw::ImageViewRef<PixelT>>::type
    read_point_cloud_compatible_file(std::string const& file) {
      return asp::read_asp_point_cloud<vw::math::VectorSize<PixelT>::value >(file);
    }

  } // end namespace point_utils_private

  /// Read multiple image files pack them into a single patchwork tiled image.
  /// - Relies on the vw::mosaic::ImageComposite class.
  template<class PixelT>
  inline vw::ImageViewRef<PixelT>
  form_point_cloud_composite(std::vector<std::string> const & files, int spacing=0);

  // Apply an offset to the points in the PointImage
  class PointOffsetFunc : public vw::UnaryReturnSameType {
    vw::Vector3 m_offset;

  public:
    PointOffsetFunc(vw::Vector3 const& offset) : m_offset(offset) {}

    template <class T>
    T operator()(T const& p) const {
      if (p == T()) return p; // Zero points are invalid. This is buggy.
      return p + m_offset;
    }
  }; // End class PointOffsetFunc

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, PointOffsetFunc>
  inline point_image_offset( vw::ImageViewBase<ImageT> const& image, vw::Vector3 const& offset) {
    return vw::UnaryPerPixelView<ImageT,PointOffsetFunc>( image.impl(), PointOffsetFunc(offset));
  }

  // TODO: Move center lon functions and use consistently.

  // Center Longitudes
  class CenterLongitudeFunc : public vw::UnaryReturnSameType {
    double center;
  public:
    CenterLongitudeFunc(double c = 0) : center(c) {}

    vw::Vector3 operator()( vw::Vector3 const& v ) const {
      if ( v[0] < center - 180 )
        return (*this)(v + vw::Vector3(360,0,0));
      else if ( v[0] > center + 180 )
        return (*this)(v - vw::Vector3(360,0,0));
      return v;
    }
  }; // End class CenterLongitudeFunc

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, CenterLongitudeFunc>
  inline recenter_longitude( vw::ImageViewBase<ImageT> const& image, double center ) {
    return vw::UnaryPerPixelView<ImageT, CenterLongitudeFunc>(image.impl(),
                                                              CenterLongitudeFunc(center));
  }

  /// ImageView operation that applies a transform matrix to every point in the image.
  class PointTransFunc : public vw::ReturnFixedType<vw::Vector3> {
    vw::Matrix3x3 m_trans;
  public:
    PointTransFunc(vw::Matrix3x3 const& trans) : m_trans(trans) {}
    vw::Vector3 operator() (vw::Vector3 const& pt) const { return m_trans*pt; }
  };

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, PointTransFunc>
  inline point_transform( vw::ImageViewBase<ImageT> const& image,
                          vw::Matrix3x3 const& t ) {
    return vw::UnaryPerPixelView<ImageT, PointTransFunc>(image.impl(),
                                                         PointTransFunc(t));
  }

  /// Compute bounding box of the given cloud. If is_geodetic is false,
  /// that means a cloud of raw xyz cartesian values, then Vector3()
  /// signifies no-data. If is_geodetic is true, no-data is suggested
  /// by having the z component of the point be NaN.
  vw::BBox3 pointcloud_bbox(vw::ImageViewRef<vw::Vector3> const& point_image,
                            bool is_geodetic);


  // Class to read points from CSV one point at a
  // time. Now that LAS support is via PDAL, this need
  // not imitate the old LAS reader in libLAS.
  class BaseReader {
  public:
    std::int64_t m_num_points;
    bool         m_has_georef; // if the dataset has a georeference
    vw::cartography::GeoReference m_georef;
    
    virtual bool        ReadNextPoint() = 0;
    virtual vw::Vector3 GetPoint() = 0;
    virtual ~BaseReader() {}
  };
 
 // In the header file for the test, the others would ideally also have a test.
 /// Reader for .pcd files created by the PCL library.
 /// - Supports ascii and binary, but only three element GCC data.
 /// - This class needs to be tested before it is used!!!!
 class PcdReader: public BaseReader{
  private:
    std::string  m_pcd_file;
    bool         m_has_valid_point;
    bool         m_binary_format;
    int          m_size_bytes;
    char         m_type;
    size_t       m_header_length_bytes;
    vw::Vector3  m_curr_point;
    std::ifstream * m_ifs;
    
    /// Read the text header of the PCD file
    void read_header();
    
  public:
    PcdReader(std::string const & pcd_file);
    virtual bool ReadNextPoint();
    virtual vw::Vector3 GetPoint();
    virtual ~PcdReader();
  }; // End class PcdReader

// Template function definitions

/// Read given files and form an image composite.
template<class PixelT>
vw::ImageViewRef<PixelT> form_point_cloud_composite(std::vector<std::string> const & files,
                                                    int spacing) {

  VW_ASSERT(files.size() >= 1, vw::ArgumentErr() << "Expecting at least one file.\n");

  vw::mosaic::ImageComposite<PixelT> composite_image;
  composite_image.set_draft_mode(true); // images will be disjoint, no need for fancy stuff

  for (int i = 0; i < (int)files.size(); i++) {

    vw::ImageViewRef<PixelT> I
      = point_utils_private::read_point_cloud_compatible_file<PixelT>(files[i]);

    // We will stack the images in the composite side by side. Images which
    // are wider than tall will be transposed.
    // To do: A more efficient approach would be to also stack one
    // image on top of each other, if some images are not too tall.
    // --> Does this code ever get images where rows != cols??
    // TODO(oalexan1): Is this transpose really useful?
    if (files.size() > 1 && I.rows() < I.cols())
      I = transpose(I);

    int start = composite_image.cols();
    if (i > 0) {
      // Insert the spacing
      start = spacing*(int)ceil(double(start)/spacing) + spacing;
    }
    composite_image.insert(I, start, 0);

  } // End loop through files

  return composite_image;
}

// Determine if we should be using a longitude range between
// [-180, 180] or [0,360]. The former is used, unless the latter
// results in a tighter range of longitudes, such as when crossing
// the international date line.
vw::BBox2 estim_lonlat_box(vw::ImageViewRef<vw::Vector3> const& point_image, 
                           vw::cartography::Datum const& datum);

// Find the median longitude and latitude for a subset of the point cloud
void median_lon_lat(vw::ImageViewRef<vw::Vector3> const& point_image,
                    vw::cartography::GeoReference const& georef,
                    double & lon, double & lat);

std::string get_cloud_type(std::string const& file_name);
  
// Find the number of channels in the point clouds.
// If the point clouds have inconsistent number of channels,
// return the minimum of 3 and the minimum number of channels.
// This will be used to flag that we cannot reliable extract the
// error channels, which start at channel 4.
int num_channels(std::vector<std::string> const& pc_files);

// See if all the input point cloud files have stddev values
bool has_stddev(std::vector<std::string> const& pc_files);
    
// Per pixel operator returning the norm of a vector  
template<class VectorT>
struct VectorNorm: public vw::ReturnFixedType<double> {
  VectorNorm() {}
  double operator() (VectorT const& vec) const {
    return norm_2(vec);
  }
};
  
// Read the error channels from the point clouds, and take their norm
template<int num_ch>
vw::ImageViewRef<double> error_norm(std::vector<std::string> const& pc_files) {

  VW_ASSERT(pc_files.size() >= 1, vw::ArgumentErr() << "Expecting at least one file.\n");

  const int beg_ech = 3; // errors start at this channel
  const int num_ech = num_ch - beg_ech; // number of error channels
  vw::ImageViewRef<vw::Vector<double, num_ch>> point_disk_image
    = asp::form_point_cloud_composite<vw::Vector<double, num_ch>>
    (pc_files, ASP_MAX_SUBBLOCK_SIZE);
  vw::ImageViewRef<vw::Vector<double, num_ech>> error_channels =
    vw::select_channels<num_ech, num_ch, double>(point_disk_image, beg_ech);

  return vw::per_pixel_filter(error_channels, VectorNorm<vw::Vector<double, num_ech>>());
}

// Get a handle to the error image given a set of point clouds with 4 or 6 bands
vw::ImageViewRef<double> 
point_cloud_error_image(std::vector<std::string> const& pointcloud_files);

} // End namespace asp

#endif
