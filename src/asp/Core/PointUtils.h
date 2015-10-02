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
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>

#include <asp/Core/Common.h>

namespace vw{
  namespace cartography{
    class Datum;
    class GeoReference;
  }
}

namespace asp {

  class BaseOptions;

  /// A Data structure which converts from CSV to Cartesian and vice-versa.
  class CsvConv{

  public: // Definitions

    // Utilities for processing csv files
    enum CsvFormat{
      XYZ, HEIGHT_LAT_LON, LAT_LON_RADIUS_M,
      LAT_LON_RADIUS_KM, EASTING_HEIGHT_NORTHING};

  public: // Variables
    std::map<std::string,int>  name2col;
    std::map<int, std::string> col2name;
    std::map<int, int>         col2sort;
    int         lon_index, lat_index;
    std::string csv_format_str;
    std::string csv_proj4_str;
    CsvFormat   format;
    int         utm_zone;
    bool        utm_north;


  public: // Functions

    /// Default Constructor, the object is not ready to use.
    CsvConv() : lon_index(-1), lat_index(-1), format(XYZ), utm_zone(-1),
                utm_north(false){}

    /// Initialize using a pair of CSV format strings
    void parse_csv_format(std::string const& csv_format_str,
                          std::string const& csv_proj4_str);

    /// If the user passed in a csv file containing easting, northing, height
    /// above datum, and either a utm zone or a custom proj4 string,
    /// pass that info into the georeference for the purpose of converting
    /// later from easting and northing to lon and lat.
    bool parse_georef(vw::cartography::GeoReference & georef) const;

    /// Parse a CSV file line in given format
    vw::Vector3 parse_csv_line(bool & is_first_line, bool & success,
                               std::string const& line) const;

    /// Convert values read from a csv file (in the same order as in the file)
    /// to a Cartesian point. If return_point_height is true, and the csv point is not
    /// in xyz format, return instead the projected point and height above datum.
    vw::Vector3 csv_to_cartesian_or_point_height(vw::Vector3 const& csv,
                                                 vw::cartography::GeoReference const& geo,
                                                 bool return_point_height) const;

    /// Convert an xyz point to the fields we can write in a CSV file, in
    /// the same order as in the input CSV file.
    vw::Vector3 cartesian_to_csv(vw::Vector3 const& xyz,
                                 vw::cartography::GeoReference const& geo,
                                 double mean_longitude) const;

  }; // End class CsvConv


  /// Fetch a chunk of the las file of area TILE_LEN x TILE_LEN,
  /// split it into bins of spatially close points, and write
  /// it to disk as a tile in a vector tif image.
  void las_or_csv_to_tif(std::string const& in_file,
                         std::string const& out_file,
                         int num_rows, int block_size,
                         asp::BaseOptions * opt,
                         vw::cartography::GeoReference const& csv_georef,
                         asp::CsvConv const& csv_conv);


  bool is_las       (std::string const& file); ///< Return true if this is a LAS file
  bool is_csv       (std::string const& file); ///< Return true if this is a CSV file
  bool is_las_or_csv(std::string const& file); ///< Return true if this file is LAS or CSV format

  /// Builds a GeoReference from a LAS file
  bool georef_from_las(std::string const& las_file,
                       vw::cartography::GeoReference & georef);

  /// Builds a GeoReference from the first cloud having a georeference in the list
  bool georef_from_pc_files(std::vector<std::string> const& files,
			    vw::cartography::GeoReference & georef);

  /// Returns the number of points stored in a LAS
  boost::uint64_t las_file_size(std::string const& las_file);

  /// Builds a datum object out of the input arguments
  bool read_user_datum(double semi_major, double semi_minor,
                       std::string const& reference_spheroid,
                       vw::cartography::Datum& datum );



  /// Parse a UTM string such as "58N"
  void parse_utm_str(std::string const& utm, int & zone, bool & north);

  ///
  inline std::string csv_separator(){ return ", \t"; }

  /// Need this for pc_align and point2dem
  inline std::string csv_opt_caption(){
    return "Specify the format of input CSV files as a list of entries column_index:column_type (indices start from 1). Examples: '1:x 2:y 3:z', '5:lon 6:lat 7:radius_m', '3:lat 2:lon 1:height_above_datum', '1:easting 2:northing 3:height_above_datum' (need to set --csv-proj4). Can also use radius_km for column_type.";
  }



  /// A valid line is not empty and does not start with '#'.
  bool is_valid_csv_line(std::string const& line);

  /// Returns the number of points contained in a CSV file
  boost::uint64_t csv_file_size(std::string const& file);

  /// Erases a file suffix if one exists and returns the base string
  std::string prefix_from_pointcloud_filename(std::string const& filename);


  /// Read a point cloud file in the format written by ASP.
  /// Given a point cloud with n channels, return the first m channels.
  /// We must have 1 <= m <= n <= 6.
  /// If the image was written by subtracting a shift, put that shift back.
  template<int m>
  vw::ImageViewRef< vw::Vector<double, m> > read_asp_point_cloud(std::string const& filename);


  /// Hide these functions from external users
  namespace point_utils_private {

    // These two functions choose between two possible inputs for the form_point_cloud_composite function.

    /// Read a texture file
    template<class PixelT>
    typename boost::enable_if<boost::is_same<PixelT, vw::PixelGray<float> >, vw::ImageViewRef<PixelT> >::type
    read_point_cloud_compatible_file(std::string const& file){
      return vw::DiskImageView<PixelT>(file);
    }
    /// Read a point cloud file
    template<class PixelT>
    typename boost::disable_if<boost::is_same<PixelT, vw::PixelGray<float> >, vw::ImageViewRef<PixelT> >::type
    read_point_cloud_compatible_file(std::string const& file){
      return asp::read_asp_point_cloud< vw::math::VectorSize<PixelT>::value >(file);
    }

  } // end namespace point_utils_private

  /// Read multiple image files pack them into a single patchwork tiled image.
  /// - Relies on the vw::mosaic::ImageComposite class.
  template<class PixelT>
  inline vw::ImageViewRef<PixelT> form_point_cloud_composite(std::vector<std::string> const & files, int spacing=0);


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


//===================================================================================
// Template function definitions

template<int m>
vw::ImageViewRef< vw::Vector<double, m> > read_asp_point_cloud(std::string const& filename){

  vw::Vector3 shift;
  std::string shift_str;
  boost::shared_ptr<vw::DiskImageResource> rsrc
    ( new vw::DiskImageResourceGDAL(filename) );
  if (vw::cartography::read_header_string(*rsrc.get(), asp::ASP_POINT_OFFSET_TAG_STR, shift_str)){
    shift = str_to_vec<vw::Vector3>(shift_str);
  }

  // Read the first m channels
  vw::ImageViewRef< vw::Vector<double, m> > out_image
    = vw::read_channels<m, double>(filename, 0);

  // Add the shift back to the first several channels.
  if (shift != vw::Vector3())
    out_image = subtract_shift(out_image, -shift);

  return out_image;
}


/// Read given files and form an image composite.
template<class PixelT>
vw::ImageViewRef<PixelT> form_point_cloud_composite(std::vector<std::string> const & files,
                                                    int spacing){

  VW_ASSERT(files.size() >= 1, vw::ArgumentErr() << "Expecting at least one file.\n");

  vw::mosaic::ImageComposite<PixelT> composite_image;
  composite_image.set_draft_mode(true); // images will be disjoint, no need for fancy stuff

  for (int i = 0; i < (int)files.size(); i++){

    vw::ImageViewRef<PixelT> I = point_utils_private::read_point_cloud_compatible_file<PixelT>(files[i]);

    // We will stack the images in the composite side by side. Images which
    // are wider than tall will be transposed.
    // To do: A more efficient approach would be to also stack one
    // image on top of each other, if some images are not too tall.
    // --> Does this code ever get images where rows != cols??
    if (I.rows() < I.cols())
      I = transpose(I);

    int start = composite_image.cols();
    if (i > 0){
      // Insert the spacing
      start = spacing*(int)ceil(double(start)/spacing) + spacing;
    }
    composite_image.insert(I, start, 0);

  } // End loop through files

  return composite_image;
}

} // End namespace asp

#endif
