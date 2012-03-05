// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file point2dem.h
///
/// This header represents the overflow of small objects and image
/// transforms that point2dem specifically applies.

#include <stdlib.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>

namespace vw {

  // Erases a file suffix if one exists and returns the base string
  static std::string prefix_from_pointcloud_filename(std::string const& filename) {
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

  // Imageview operator that extracts only the first 3 channels of the
  // point cloud. The forth channel is the point cloud error.
  struct SelectPoints : public ReturnFixedType<Vector3> {
    Vector3 operator() (Vector4 const& pt) const { return subvector(pt,0,3); }
  };

  template <class ImageT>
  UnaryPerPixelView<ImageT, SelectPoints>
  inline select_points( ImageViewBase<ImageT> const& image ) {
    return UnaryPerPixelView<ImageT, SelectPoints>( image.impl(),
                                                    SelectPoints() );
  }
}
