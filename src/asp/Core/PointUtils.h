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

#ifndef __ASP_CORE_POINT_UTILS_H__
#define __ASP_CORE_POINT_UTILS_H__

#include <string>
#include <vw/Core/Functors.h>
#include <vw/Image/PerPixelViews.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

namespace vw{
  namespace cartography{
    class Datum;
    class GeoReference;
  }
}

namespace asp {

  class BaseOptions;
  
  bool is_las(std::string const& file);
  bool is_csv(std::string const& file);
  bool is_las_or_csv(std::string const& file);
  
  bool georef_from_las(std::string const& las_file,
                       vw::cartography::GeoReference & georef);

  bool georef_from_las(std::vector<std::string> const& las_files,
                       vw::cartography::GeoReference & georef);

  boost::uint64_t las_file_size(std::string const& las_file);
    
  bool read_user_datum(double semi_major, double semi_minor,
                       std::string const& reference_spheroid,
                       vw::cartography::Datum& datum );

  void parse_utm_str(std::string const& utm, int & zone, bool & north);

  inline std::string csv_separator(){ return ", \t"; }

  // Need this for pc_align and point2dem
  inline std::string csv_opt_caption(){
    return "Specify the format of input CSV files as a list of entries column_index:column_type (indices start from 1). Examples: '1:x 2:y 3:z', '5:lon 6:lat 7:radius_m', '3:lat 2:lon 1:height_above_datum', 'utm:47N 1:easting 2:northing 3:height_above_datum'. Can also use radius_km for column_type.";
  }
  
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
    CsvFormat format;
    int utm_zone;
    bool utm_north;
    CsvConv():lon_index(-1), lat_index(-1), format(XYZ), utm_zone(-1),
              utm_north(false){}
  };

  void las_or_csv_to_tif(std::string const& in_file,
                         std::string const& out_file,
                         int num_rows, int block_size,
                         asp::BaseOptions * opt, 
                         vw::cartography::GeoReference const& csv_georef,
                         asp::CsvConv const& csv_conv);

  void parse_csv_format(std::string const& csv_format_str, CsvConv & C);
  
  vw::Vector3 parse_csv_line(bool & is_first_line, bool & success,
                             std::string const& line,
                             asp::CsvConv const& csv_conv);

  vw::Vector3 csv_to_cartesian_or_point_height(vw::Vector3 const& csv,
                                               vw::cartography::GeoReference const& geo,
                                               CsvConv const& C,
                                               bool return_point_height);

  vw::Vector3 cartesian_to_csv(vw::Vector3 const& xyz,
                               vw::cartography::GeoReference const& geo,
                               double mean_longitude,
                               CsvConv const& C);
  
  bool is_valid_csv_line(std::string const& line);

  boost::uint64_t csv_file_size(std::string const& file);
  
  // Erases a file suffix if one exists and returns the base string
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
  };

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
  };

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, CenterLongitudeFunc>
  inline recenter_longitude( vw::ImageViewBase<ImageT> const& image, double center ) {
    return vw::UnaryPerPixelView<ImageT, CenterLongitudeFunc>(image.impl(),
                                                              CenterLongitudeFunc(center));
  }

  // Imageview operation that applies a transform matrix to every point
  // in the image.
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

  // Helper function that extracts the bounding box of a point cloud. It
  // skips the point Vector3(), which, being at the center of the
  // planet, is an invalid point.

  template <class ViewT>
  vw::BBox3 pointcloud_bbox(vw::ImageViewBase<ViewT> const& point_image) {
    // Compute bounding box
    vw::BBox3 result;
    typename ViewT::pixel_accessor row_acc = point_image.impl().origin();
    vw::vw_out() << "Computing the point cloud bounding box.\n";
    vw::TerminalProgressCallback progress_bar("asp","");

    for (int row=0; row < point_image.impl().rows(); ++row ) {
      typename ViewT::pixel_accessor col_acc = row_acc;
      progress_bar.report_fractional_progress(row, point_image.impl().rows());
      for (int col=0; col < point_image.impl().cols(); ++col ) {
        if (*col_acc != vw::Vector3())
          result.grow(*col_acc);
        col_acc.next_col();
      }
      row_acc.next_row();
    }
    progress_bar.report_finished();
    
    return result;
  }
  
}  

#endif
