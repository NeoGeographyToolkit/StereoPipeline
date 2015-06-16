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
///

//  Note: A LAS file is a standard format for storing airborne LIDAR data.

#ifndef __ASP_CORE_POINT_UTILS_H__
#define __ASP_CORE_POINT_UTILS_H__

#include <string>
#include <vw/Core/Functors.h>
#include <vw/Image/PerPixelViews.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/ImageViewRef.h>

namespace vw{
  namespace cartography{
    class Datum;
    class GeoReference;
  }
}

namespace asp {

  class BaseOptions;

  // Utilities for processing csv files
  enum CsvFormat{
    XYZ, HEIGHT_LAT_LON, LAT_LON_RADIUS_M,
    LAT_LON_RADIUS_KM, EASTING_HEIGHT_NORTHING};

  struct CsvConv{
    std::map<std::string,int> name2col;
    std::map<int, std::string> col2name;
    std::map<int, int> col2sort;
    int lon_index, lat_index;
    std::string csv_format_str;
    std::string csv_proj4_str;
    CsvFormat format;
    int utm_zone;
    bool utm_north;
    CsvConv():lon_index(-1), lat_index(-1), format(XYZ), utm_zone(-1),
              utm_north(false){}
  };

  bool is_las       (std::string const& file); ///< Return true if this is a LAS file
  bool is_csv       (std::string const& file); ///< Return true if this is a CSV file
  bool is_las_or_csv(std::string const& file); ///< Return true if this file is LAS or CSV format

  /// Builds a GeoReference from a LAS file
  bool georef_from_las(std::string const& las_file,
                       vw::cartography::GeoReference & georef);

  /// Builds a GeoReference from the first LAS file found in the file list
  bool georef_from_las(std::vector<std::string> const& files,
                       vw::cartography::GeoReference & georef);

  /// Returns the number of points stored in a LAS
  boost::uint64_t las_file_size(std::string const& las_file);

  /// Builds a datum object out of the input arguments
  bool read_user_datum(double semi_major, double semi_minor,
                       std::string const& reference_spheroid,
                       vw::cartography::Datum& datum );

  /// If the user passed in a csv file containing easting, northing, height
  /// above datum, and either a utm zone or a custom proj4 string,
  /// pass that info into the georeference for the purpose of converting
  /// later from easting and northing to lon and lat.
  void handle_easting_northing(asp::CsvConv const& csv_conv,
                               vw::cartography::GeoReference & georef);

  /// Parse a UTM string such as "58N"
  void parse_utm_str(std::string const& utm, int & zone, bool & north);

  ///
  inline std::string csv_separator(){ return ", \t"; }

  /// Need this for pc_align and point2dem
  inline std::string csv_opt_caption(){
    return "Specify the format of input CSV files as a list of entries column_index:column_type (indices start from 1). Examples: '1:x 2:y 3:z', '5:lon 6:lat 7:radius_m', '3:lat 2:lon 1:height_above_datum', '1:easting 2:northing 3:height_above_datum' (need to set --csv-proj4). Can also use radius_km for column_type.";
  }

  /// Fetch a chunk of the las file of area tile_len x
  /// tile_len, split it into bins of spatially close points, and write
  /// it to disk as a tile in a vector tif image.
  void las_or_csv_to_tif(std::string const& in_file,
                         std::string const& out_file,
                         int num_rows, int block_size,
                         asp::BaseOptions * opt,
                         vw::cartography::GeoReference const& csv_georef,
                         asp::CsvConv const& csv_conv);

  /// Parse the CSV format string and build the data structure which
  /// will enable to convert from CSV to Cartesian and vice-versa.
  void parse_csv_format(std::string const& csv_format_str,
                        std::string const& csv_proj4_str,
                        CsvConv & C);

  /// Parse a CSV file line in given format
  vw::Vector3 parse_csv_line(bool & is_first_line, bool & success,
                             std::string const& line,
                             asp::CsvConv const& csv_conv);

  /// Convert values read from a csv file (in the same order as in the file)
  /// to a Cartesian point. If return_point_height is true, and the csv point is not
  /// in xyz format, return instead the projected point and height above datum.
  vw::Vector3 csv_to_cartesian_or_point_height(vw::Vector3 const& csv,
                                               vw::cartography::GeoReference const& geo,
                                               CsvConv const& C,
                                               bool return_point_height);

  /// Convert an xyz point to the fields we can write in a CSV file, in
  /// the same order as in the input CSV file.
  vw::Vector3 cartesian_to_csv(vw::Vector3 const& xyz,
                               vw::cartography::GeoReference const& geo,
                               double mean_longitude,
                               CsvConv const& C);

  /// A valid line is not empty and does not start with '#'.
  bool is_valid_csv_line(std::string const& line);

  /// Returns the number of points contained in a CSV file
  boost::uint64_t csv_file_size(std::string const& file);

  /// Erases a file suffix if one exists and returns the base string
  std::string prefix_from_pointcloud_filename(std::string const& filename);


  // Apply an offset to the points in the PointImage
  class PointOffsetFunc : public vw::UnaryReturnSameType {
    vw::Vector3 m_offset;

  public:
    PointOffsetFunc(vw::Vector3 const& offset) : m_offset(offset) {}

    template <class T>
    T operator()(T const& p) const {
      if (p == T()) return p;
      return p + m_offset;
    }
  }; // End class PointOffsetFunc

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, PointOffsetFunc>
  inline point_image_offset( vw::ImageViewBase<ImageT> const& image, vw::Vector3 const& offset) {
    return vw::UnaryPerPixelView<ImageT,PointOffsetFunc>( image.impl(), PointOffsetFunc(offset) );
  }


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


  /// Imageview operation that applies a transform matrix to every point in the image.
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


}

#endif
