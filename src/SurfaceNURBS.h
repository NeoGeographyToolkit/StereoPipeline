#ifndef __SURFACENURBS_H__
#define __SURFACENURBS_H__

#include <asp_config.h>

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Core/ProgressCallback.h>

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
static int init_mba_xy(vw::ImageViewBase<ImageT> const& image,
                       std::vector<double> &x_array,
                       std::vector<double> &y_array) {

  int i = 0;
  for (unsigned int row = 0; row < image.impl().rows(); row++) {
    for (unsigned int col = 0; col < image.impl().cols(); col++) {
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
static int init_mba_z(vw::ImageViewBase<ViewT> const& image,
                      std::vector<double> const& x_array, 
                      std::vector<double> const& y_array,
                      std::vector<double> &z_array, int channel) {
  z_array.resize(x_array.size());
  for (int i = 0; i < x_array.size(); ++i) 
    z_array[i] = vw::compound_select_channel<typename vw::CompoundChannelType<typename ViewT::pixel_type>::type>(image.impl()(int(x_array[i]),int(y_array[i])), channel);
}

template <class PixelT> 
class SurfaceEvaluator { 

  std::vector<UCBspl::SplineSurface> &m_fitted_surfaces;

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
    std::cout << "Fitting spline surface for channel " << channel << ".\n";
    MBA mba_z(x_arr, y_arr, z_arr);   // Initialize with scattered data
    mba_z.MBAalg(m0,n0,numIterations);            // Create spline surface
    fitted_surfaces[channel] = mba_z.getSplineSurface(); // Get the spline surface object
    std::cout << "\tRange -- U: [" << fitted_surfaces[channel].umin() << " " << fitted_surfaces[channel].umax() << "]\n";
    std::cout << "\t         V: [" << fitted_surfaces[channel].vmin() << " " << fitted_surfaces[channel].vmax() << "]\n";
  }

  // Free up some memory.
  (*x_arr).clear();
  (*y_arr).clear();

  //  Rasterize the surface by iterating over all of the x and y values.
  vw::ImageView<pixel_type> output(input.impl().cols(), input.impl().rows());
  std::cout << "\tEvaluating surfaces.\n";
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


#endif  // have MBA nurbs

#endif  // __SURFACENURBS_H__
