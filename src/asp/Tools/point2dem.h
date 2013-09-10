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


/// \file point2dem.h
///
/// This header represents the overflow of small objects and image
/// transforms that point2dem specifically applies.

// To do: Many of these utilities may need to go to a lower-level header file
// dealing with point cloud manipulations.

#include <stdlib.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>

namespace vw {

  // Erases a file suffix if one exists and returns the base string
  std::string prefix_from_pointcloud_filename(std::string const& filename) {
    std::string result = filename;

    // First case: filenames that match <prefix>-PC.<suffix>
    int index = result.rfind("-PC.");
    if (index != -1) {
      result.erase(index, result.size());
      return result;
    }

    // Second case: filenames that match <prefix>.<suffix>
    index = result.rfind(".");
    if (index != -1) {
      result.erase(index, result.size());
      return result;
    }

    // No match
    return result;
  }

  // Apply an offset to the points in the PointImage
  class PointOffsetFunc : public UnaryReturnSameType {
    Vector3 m_offset;

  public:
    PointOffsetFunc(Vector3 const& offset) : m_offset(offset) {}

    template <class T>
    T operator()(T const& p) const {
      if (p == T()) return p;
      return p + m_offset;
    }
  };

  template <class ImageT>
  UnaryPerPixelView<ImageT, PointOffsetFunc>
  inline point_image_offset( ImageViewBase<ImageT> const& image, Vector3 const& offset) {
    return UnaryPerPixelView<ImageT,PointOffsetFunc>( image.impl(), PointOffsetFunc(offset) );
  }

  // Center Longitudes
  class CenterLongitudeFunc : public UnaryReturnSameType {
    double center;
  public:
    CenterLongitudeFunc(double c = 0) : center(c) {}

    Vector3 operator()( Vector3 const& v ) const {
      if ( v[0] < center - 180 )
        return (*this)(v + Vector3(360,0,0));
      else if ( v[0] > center + 180 )
        return (*this)(v - Vector3(360,0,0));
      return v;
    }
  };

  template <class ImageT>
  UnaryPerPixelView<ImageT, CenterLongitudeFunc>
  inline recenter_longitude( ImageViewBase<ImageT> const& image, double center ) {
    return UnaryPerPixelView<ImageT, CenterLongitudeFunc>(image.impl(),
                                                          CenterLongitudeFunc(center));
  }

  // Imageview operation that applies a transform matrix to every point
  // in the image.
  class PointTransFunc : public ReturnFixedType<Vector3> {
    Matrix3x3 m_trans;
  public:
    PointTransFunc(Matrix3x3 const& trans) : m_trans(trans) {}
    Vector3 operator() (Vector3 const& pt) const { return m_trans*pt; }
  };

  template <class ImageT>
  UnaryPerPixelView<ImageT, PointTransFunc>
  inline point_transform( ImageViewBase<ImageT> const& image,
                          Matrix3x3 const& t ) {
    return UnaryPerPixelView<ImageT, PointTransFunc>(image.impl(),
                                                     PointTransFunc(t));
  }

  // Helper function that extracts the bounding box of a point cloud. It
  // skips the point Vector3(), which, being at the center of the
  // planet, is an invalid point.

  template <class ViewT>
  BBox3 pointcloud_bbox(ImageViewBase<ViewT> const& point_image) {
    // Compute bounding box
    BBox3 result;
    typename ViewT::pixel_accessor row_acc = point_image.impl().origin();
    for( int32 row=0; row < point_image.impl().rows(); ++row ) {
      typename ViewT::pixel_accessor col_acc = row_acc;
      for( int32 col=0; col < point_image.impl().cols(); ++col ) {
        if (*col_acc != Vector3())
          result.grow(*col_acc);
        col_acc.next_col();
      }
      row_acc.next_row();
    }
    return result;
  }



}
