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


/// \file SurfaceNURBS.h
///

#ifndef __SURFACENURBS_H__
#define __SURFACENURBS_H__

#include <asp_config.h>

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Image/Interpolation.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Core/ProgressCallback.h>
#include <vw/Math/Functions.h>

#if defined(ASP_HAVE_PKG_MBA) && ASP_HAVE_PKG_MBA==1
#include <MBA.h>
#include <UCButils.h>
#include <boost/shared_ptr.hpp>

// These type computation routines help us to decide which pixels are
// considered "valid" when computing the NURBS fit.
template <class PixelT> struct is_valid_pixel {
  static bool value(PixelT const& pix) { return true; }
};


template <class ChannelT>  struct is_valid_pixel<vw::PixelDisparity<ChannelT> > {
  static bool value(vw::PixelDisparity<ChannelT> const& pix)  {
    return !pix.missing();
  }
};

template <class ImageT>
static void init_mba_xy(vw::ImageViewBase<ImageT> const& image,
                       std::vector<double> &x_array,
                       std::vector<double> &y_array) {

  int i = 0;
  for (int32 row = 0; row < image.impl().rows(); row++) {
    for (int32 col = 0; col < image.impl().cols(); col++) {
      if (is_valid_pixel<typename ImageT::pixel_type>::value(image.impl()(col, row))) {
        x_array.push_back(col);
        y_array.push_back(row);
        i++;
      }
    }
  }

  if ((image.impl().cols() * image.impl().rows() == 0) || (x_array.size() == 0))
    throw vw::ArgumentErr() << "MBA NURBS Adaptation failed. Image does not contain any points or image has zero valid pixels.";
}


// Copy the data into STL vectors; this is the format expected by
// the MBA NURBS code.  The View must have a PixelGray pixel type.
template <class ViewT>
static void init_mba_z(vw::ImageViewBase<ViewT> const& image,
                      std::vector<double> const& x_array,
                      std::vector<double> const& y_array,
                      std::vector<double> &z_array, int channel) {
  z_array.resize(x_array.size());
  for (unsigned i = 0; i < x_array.size(); ++i)
    z_array[i] = vw::compound_select_channel<typename vw::CompoundChannelType<typename ViewT::pixel_type>::type>(image.impl()(int(x_array[i]),int(y_array[i])), channel);
}

template <class PixelT>
class SurfaceEvaluator {

  std::vector<UCBspl::SplineSurface> m_fitted_surfaces;

public:
  SurfaceEvaluator(std::vector<UCBspl::SplineSurface> const& fitted_surfaces) :
    m_fitted_surfaces(fitted_surfaces) {}

  PixelT operator()(int i, int j) const {
    PixelT result;
    if (i > m_fitted_surfaces[0].umin() && i < m_fitted_surfaces[0].umax() &&
        j > m_fitted_surfaces[0].vmin() && j < m_fitted_surfaces[0].vmax()) {
      for (int c = 0; c < vw::PixelNumChannels<PixelT>::value; ++c)
        result[c] = m_fitted_surfaces[c].f((float)i,(float)j);
    }
    return result;
  }
};

template <class ChannelT>
class SurfaceEvaluator<vw::PixelDisparity<ChannelT> > {

  const std::vector<UCBspl::SplineSurface> &m_fitted_surfaces;

public:
  SurfaceEvaluator(std::vector<UCBspl::SplineSurface> const& fitted_surfaces) :
    m_fitted_surfaces(fitted_surfaces) {
    VW_ASSERT(m_fitted_surfaces.size() == 2,
              vw::ArgumentErr() << "SurfaceEvaluator: expecting two surfaces for pixels of PixelDisparty type.");
  }

  vw::PixelDisparity<ChannelT> operator()(int i, int j) const {
    vw::PixelDisparity<ChannelT> result;
    if (i > m_fitted_surfaces[0].umin() && i < m_fitted_surfaces[0].umax() &&
        j > m_fitted_surfaces[0].vmin() && j < m_fitted_surfaces[0].vmax()) {
      return vw::PixelDisparity<ChannelT> (m_fitted_surfaces[0].f((float)i,(float)j),
                                       m_fitted_surfaces[1].f((float)i,(float)j));
    } else {
      return vw::PixelDisparity<ChannelT>();
    }
  }
};

// These two classes are just a quick fix for now since the
// PixelDisparity pixel type really only has two channels that we want
// to fit NURBS data on, but three channels total as far as the VW
// pixel type system is concerned.
template <class PixelT>
struct NURBSChannelCount { static int value() { return vw::PixelNumChannels<PixelT>::value; } };
template <class ChannelT>
struct NURBSChannelCount<vw::PixelDisparity<ChannelT> > { static int value() { return 2; } };

/// Fit a 2D spline surface to a given image of data.  Each channel of
/// the image is handled as an independent 2D surface.
template <class ViewT>
vw::ImageView<typename ViewT::pixel_type> MBASurfaceNURBS(vw::ImageViewBase<ViewT> const& input,
                                                          int numIterations) {

  typedef typename ViewT::pixel_type pixel_type;

  // NOTE: we assume only one channel here!
  VW_ASSERT(input.impl().planes() == 1, vw::NoImplErr() <<
            "FindNURBSSurface() does not have support for images with more than one plane.");

  boost::shared_ptr<std::vector<double> > x_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > y_arr(new std::vector<double>);

  // Set the size ratio of the spline space.  This causes the
  // algorithm to create more control points in one dimenion than in
  // the other.
  int m0 = 1, n0 = 1;
  if (input.impl().cols() > input.impl().rows())
    m0 = input.impl().cols() / input.impl().rows();
  else
    n0 = input.impl().rows() / input.impl().cols();

  // Initialize the grid
  init_mba_xy(input.impl(), *x_arr, *y_arr);
  // Create a vector of spline surfaces
  std::vector<UCBspl::SplineSurface> fitted_surfaces(NURBSChannelCount<pixel_type>::value());

  // For each channel in the image, fit a spline surface
  for (int channel = 0; channel < NURBSChannelCount<pixel_type>::value(); ++channel) {
    boost::shared_ptr<std::vector<double> > z_arr(new std::vector<double>);
    init_mba_z(input.impl(), *x_arr, *y_arr, *z_arr, channel);
    std::cout << "\t    Fitting spline surface for channel " << channel << ".\n";
    MBA mba_z(x_arr, y_arr, z_arr);   // Initialize with scattered data
    mba_z.MBAalg(m0,n0,numIterations);            // Create spline surface
    fitted_surfaces[channel] = mba_z.getSplineSurface(); // Get the spline surface object
    std::cout << "\t    Range -- U: [" << fitted_surfaces[channel].umin() << " " << fitted_surfaces[channel].umax() << "]\n";
    std::cout << "\t             V: [" << fitted_surfaces[channel].vmin() << " " << fitted_surfaces[channel].vmax() << "]\n";
  }

  // Free up some memory.
  (*x_arr).clear();
  (*y_arr).clear();

  //  Rasterize the surface by iterating over all of the x and y values.
  vw::ImageView<pixel_type> output(input.impl().cols(), input.impl().rows());
  std::cout << "\t    Evaluating surfaces.\n";
  SurfaceEvaluator<pixel_type> evaluator(fitted_surfaces);
  for (int j = 0; j < input.impl().rows(); ++j) {
    for (int i = 0; i < input.impl().cols(); ++i) {
      output(i,j) = evaluator(i,j);
    }
  }
  //   // Sample surface and print to VRML file.
  //   UCBspl::printVRMLgrid("qwe.wrl", surface, 50, 50, true);

  return output;
}




// Rapid resample()
//
// Downsample a disparity map by averaging pixels.  In this version of
// the algorithm, missing pixels are "consumed" by valid pixels.
template <class ViewT>
ImageView<typename ViewT::pixel_type > disparity_rapid_downsample(ImageViewBase<ViewT> const& view_,
                                                                  int downsample_size) {

  EdgeExtensionView<ViewT, ConstantEdgeExtension> view(view_.impl(), ConstantEdgeExtension());
  typedef typename ViewT::pixel_type pixel_type;
  typedef Vector2 accumulator_type;
  typedef typename EdgeExtensionView<ViewT, ConstantEdgeExtension>::pixel_accessor pixel_accessor;

  int32 resized_rows = view.impl().rows() / downsample_size;
  int32 resized_cols = view.impl().cols() / downsample_size;
  ImageView<pixel_type> result(resized_cols, resized_rows);

  std::cout << "\t    Subsampling disparity map to output size: " << resized_cols << " x " << resized_rows << "\n";;

  TerminalProgressCallback progress_callback(ErrorMessage, "\t    Subsampling: ");
  progress_callback.report_progress(0);

  for (int32 j = 0; j < view.impl().rows(); j+=downsample_size) {
    progress_callback.report_progress((float)j / view.impl().rows());
    for (int32 i = 0; i < view.impl().cols(); i+= downsample_size) {
      accumulator_type sum = accumulator_type();
      int num_good_pixels = 0;

      pixel_accessor row_acc = view.origin().advance(i,j);
      for (int32 v = 0; v < downsample_size; ++v) {
        pixel_accessor col_acc = row_acc;
        for (int32 u = 0; u < downsample_size; ++u) {
          if ( !((*col_acc).missing()) ) {
            sum[0] += (*col_acc).h();
            sum[1] += (*col_acc).v();
            ++num_good_pixels;
          }
          col_acc.next_col();
        }
        row_acc.next_row();
      }

      // If there were any valid pixels, we count them towards the
      // average.  If there are none, we mark this location as a
      // missing pixel.
      pixel_type avg;
      if (num_good_pixels != 0) {
        avg = pixel_type(sum[0] / num_good_pixels,
                         sum[1] / num_good_pixels);
      } else {
        avg = pixel_type();
      }

      // This bounds checking keeps us from straying outside of the
      // result image.
      if (i/downsample_size < resized_cols && j/downsample_size < resized_rows)
        result(i/downsample_size, j/downsample_size) = avg;
    }
  }
  progress_callback.report_finished();
  return result;
}



class HoleFillView : public ImageViewBase<HoleFillView> {
  ImageViewRef<PixelDisparity<float> > m_original_disparity_map;
  ImageView<PixelDisparity<float> > m_lowres_disparity_map;
  int m_subsample_factor;

public:
  typedef PixelDisparity<float> pixel_type;
  typedef const pixel_type result_type;
  typedef ProceduralPixelAccessor<HoleFillView> pixel_accessor;

  template <class DisparityViewT>
  HoleFillView(DisparityViewT disparity_map, int subsample_factor = 16)
    : m_original_disparity_map(disparity_map), m_subsample_factor(subsample_factor) {

    // Compute the bounding box that encompasses all of the
    // available points.
    m_lowres_disparity_map = MBASurfaceNURBS(disparity_rapid_downsample(disparity_map, m_subsample_factor), 10);
  }

  inline int32 cols() const { return m_original_disparity_map.cols(); }
  inline int32 rows() const { return m_original_disparity_map.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( int i, int j, int p=0 ) const {

    // If we can, we take the pixels from the original disparity
    // image.
    if ( !(m_original_disparity_map(i,j).missing()) ) {
      return m_original_disparity_map(i,j);

    // Otherwise, we resort to using the hole fill values.  These
    // values are the result of a low resolution NURBS fit followed by
    // bicubic interpolation.
    } else {
      float ii = float(i)/m_subsample_factor;
      float jj = float(j)/m_subsample_factor;

      int32 x = vw::math::impl::_floor(ii), y = vw::math::impl::_floor(jj);
      double normx = ii-x, normy = jj-y;

      double s0 = ((2-normx)*normx-1)*normx;      double t0 = ((2-normy)*normy-1)*normy;
      double s1 = (3*normx-5)*normx*normx+2;      double t1 = (3*normy-5)*normy*normy+2;
      double s2 = ((4-3*normx)*normx+1)*normx;    double t2 = ((4-3*normy)*normy+1)*normy;
      double s3 = (normx-1)*normx*normx;          double t3 = (normy-1)*normy*normy;

      EdgeExtensionView<ImageView<PixelDisparity<float> >,ConstantEdgeExtension> extended_view = edge_extend(m_lowres_disparity_map,ConstantEdgeExtension());
      if (extended_view(x-1,y-1).missing() || extended_view(x-1,y).missing() || extended_view(x-1,y+1).missing() ||
          extended_view(x  ,y-1).missing() || extended_view(x  ,y).missing() || extended_view(x  ,y+1).missing() ||
          extended_view(x+1,y-1).missing() || extended_view(x+1,y).missing() || extended_view(x+1,y+1).missing() ) {
        return PixelDisparity<float>();
      } else {
        float hdisp = ( ( s0*extended_view(x-1,y-1,p).h() + s1*extended_view(x+0,y-1,p).h() + s2*extended_view(x+1,y-1,p).h() + s3*extended_view(x+2,y-1,p).h() ) * t0 +
                        ( s0*extended_view(x-1,y+0,p).h() + s1*extended_view(x+0,y+0,p).h() + s2*extended_view(x+1,y+0,p).h() + s3*extended_view(x+2,y+0,p).h() ) * t1 +
                        ( s0*extended_view(x-1,y+1,p).h() + s1*extended_view(x+0,y+1,p).h() + s2*extended_view(x+1,y+1,p).h() + s3*extended_view(x+2,y+1,p).h() ) * t2 +
                        ( s0*extended_view(x-1,y+2,p).h() + s1*extended_view(x+0,y+2,p).h() + s2*extended_view(x+1,y+2,p).h() + s3*extended_view(x+2,y+2,p).h() ) * t3 ) * 0.25;
        float vdisp = ( ( s0*extended_view(x-1,y-1,p).v() + s1*extended_view(x+0,y-1,p).v() + s2*extended_view(x+1,y-1,p).v() + s3*extended_view(x+2,y-1,p).v() ) * t0 +
                        ( s0*extended_view(x-1,y+0,p).v() + s1*extended_view(x+0,y+0,p).v() + s2*extended_view(x+1,y+0,p).v() + s3*extended_view(x+2,y+0,p).v() ) * t1 +
                        ( s0*extended_view(x-1,y+1,p).v() + s1*extended_view(x+0,y+1,p).v() + s2*extended_view(x+1,y+1,p).v() + s3*extended_view(x+2,y+1,p).v() ) * t2 +
                        ( s0*extended_view(x-1,y+2,p).v() + s1*extended_view(x+0,y+2,p).v() + s2*extended_view(x+1,y+2,p).v() + s3*extended_view(x+2,y+2,p).v() ) * t3 ) * 0.25;
        return PixelDisparity<float>(hdisp,vdisp);
      }
    }
  }

  /// \cond INTERNAL
  typedef HoleFillView prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const { return *this; }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
  /// \endcond

};

#endif  // have MBA nurbs

#endif  // __SURFACENURBS_H__
