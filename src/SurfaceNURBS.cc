#include <asp_config.h>
#include <SurfaceNURBS.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include "stereo.h"
using namespace std;
using namespace vw;

#if defined(ASP_HAVE_PKG_MBA) && ASP_HAVE_PKG_MBA==1
#include <MBA.h>
#include <UCButils.h>
#include <boost/shared_ptr.hpp>

// These type computation routines help us to decide which pixels are
// considered "valid" when computing the NURBS fit.
template <class PixelT> struct is_valid_pixel { 
  static bool value(PixelT const& pix) { return true; }
};

template <class ChannelT>  struct is_valid_pixel<PixelDisparity<ChannelT> > {
  static bool value(PixelDisparity<ChannelT> const& pix)  {
    return !pix.missing();
  }
};

template <class ChannelT> struct is_valid_pixel<PixelGrayA<ChannelT> > {
  static bool value(PixelGrayA<ChannelT> const& pix) {
    return (pix.a() == 1);
  }
};
    
template <class ImageT>
static int init_mba_xy(ImageT const& image,
                       std::vector<double> &x_array,
                       std::vector<double> &y_array) {

  int i = 0;
  for (unsigned int row = 0; row < image.rows(); row++) {
    for (unsigned int col = 0; col < image.cols(); col++) {
      if (is_valid_pixel<typename ImageT::pixel_type>::value(image(col, row))) {
        x_array.push_back(col);
        y_array.push_back(row);      
        i++;
      }
    }
  }  

  if ((image.cols() * image.rows() == 0) || (x_array.size() == 0)) 
    throw vw::ArgumentErr() << "MBA NURBS Adaptation failed. Image does not contain any points or image has zero valid pixels.";
    
}


// Copy the data into STL vectors; this is the format expected by
// the MBA NURBS code.
static int init_mba_z(ImageView<PixelGrayA<float> > const& image,
                      std::vector<double> const& x_array, std::vector<double> const& y_array,
                      std::vector<double> &z_array) {
  z_array.resize(x_array.size());
  for (int i = 0; i < x_array.size(); ++i) 
    z_array[i] = image(int(x_array[i]),int(y_array[i])).v();
}

// Copy the data into STL vectors; this is the format expected by
// the MBA NURBS code.
static int init_mba_hv(ImageView<PixelDisparity<float> > const& disparity_map,
                       std::vector<double> const& x_array, std::vector<double> const& y_array,
                       std::vector<double> &h_array, std::vector<double> &v_array) {
  
  h_array.resize(x_array.size());
  v_array.resize(x_array.size());
  
  for (int i = 0; i < x_array.size(); ++i) {
    h_array[i] = disparity_map(int(x_array[i]),int(y_array[i])).h();
    v_array[i] = disparity_map(int(x_array[i]),int(y_array[i])).v();
  }
}

ImageView<PixelGrayA<float> > MBASurfaceNURBS(ImageView<PixelGrayA<float> > const& input, 
                                              int numIterations) {

  ImageView<PixelGrayA<float> > output(input.cols(), input.rows());

  // NOTE: we assume only one channel here!
  VW_ASSERT(input.planes() == 1, vw::NoImplErr() << 
            "FindNURBSSurface() does not have support for images with more than one plane.");  

  boost::shared_ptr<std::vector<double> > x_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > y_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > z_arr(new std::vector<double>);

  init_mba_xy(input, *x_arr, *y_arr);
  init_mba_z(input, *x_arr, *y_arr, *z_arr);

  // Set the size ratio of the spline space.  This causes the
  // algorithm to create more control points in one dimenion than in
  // the other.
  int m0, n0;
  if (input.cols() > input.rows()) {
    m0 = input.cols() / input.rows();
    n0 = 1;
  } else {
    m0 = 1;
    n0 = input.rows() / input.cols();
  }

  std::cout << "Generating spline surface using the MBA algorithm.\n";
  MBA mba_z(x_arr, y_arr, z_arr);   // Initialize with scattered data

  std::cout << "\tFitting NURBS\n";
  mba_z.MBAalg(m0,n0,numIterations);            // Create spline surface
  UCBspl::SplineSurface surface_z = mba_z.getSplineSurface(); // Get the spline surface object
  std::cout << "\tRange -- U: [" << surface_z.umin() << " " << surface_z.umax() << "]\n";
  std::cout << "\t         V: [" << surface_z.vmin() << " " << surface_z.vmax() << "]\n";

  //  Rasterize the surface by iterating over all of the x and y values.
  std::cout << "\tEvaluating surface.\n";
  for (int i = 0; i < input.cols(); i++) {
    for (int j = 0; j < input.rows(); j++) {
      if (i > surface_z.umin() && i < surface_z.umax() &&
          j > surface_z.vmin() && j < surface_z.vmax()) {
        output(i,j) = PixelGrayA<float>(surface_z.f((float)i,(float)j));
      } else {
        output(i,j) = PixelGrayA<float>(); // Missing pixel
      }
    }
  } 
  //   // Sample surface and print to VRML file.
  //   UCBspl::printVRMLgrid("qwe.wrl", surface, 50, 50, true);  
  return output;
}

ImageView<PixelDisparity<float> > MBASurfaceNURBS(ImageView<PixelDisparity<float> > const& input, 
                                                  int numIterations) {

  ImageView<PixelDisparity<float> > output(input.cols(), input.rows());

  // NOTE: we assume only one channel here!
  VW_ASSERT(input.planes() == 1, vw::NoImplErr() << 
            "FindNURBSSurface() does not have support for images with more than one plane.");  
  output.set_size(input.cols(), input.rows());

  boost::shared_ptr<std::vector<double> > x_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > y_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > h_arr(new std::vector<double>);
  boost::shared_ptr<std::vector<double> > v_arr(new std::vector<double>);

  init_mba_xy(input, *x_arr, *y_arr);
  init_mba_hv(input, *x_arr, *y_arr, *h_arr, *v_arr);

  // Set the size ratio of the spline space.  This causes the
  // algorithm to create more control points in one dimenion than in
  // the other.
  int m0, n0;
  if (input.cols() > input.rows()) {
    m0 = input.cols() / input.rows();
    n0 = 1;
  } else {
    m0 = 1;
    n0 = input.rows() / input.cols();
  }

  std::cout << "Generating spline surface using the MBA algorithm.\n";
  MBA mba_h(x_arr, y_arr, h_arr);   // Initialize with scattered data
  MBA mba_v(x_arr, y_arr, v_arr);   // Initialize with scattered data

  std::cout << "\tProcessing horizontal disparities\n";
  mba_h.MBAalg(m0,n0,numIterations);            // Create spline surface
  UCBspl::SplineSurface surface_h = mba_h.getSplineSurface(); // Get the spline surface object
  std::cout << "\tRange -- U: [" << surface_h.umin() << " " << surface_h.umax() << "]\n";
  std::cout << "\t         V: [" << surface_h.vmin() << " " << surface_h.vmax() << "]\n";

  std::cout << "\tProcessing vertical disparities\n";
  mba_v.MBAalg(m0,n0,numIterations);            // Create spline surface
  UCBspl::SplineSurface surface_v = mba_v.getSplineSurface(); // Get the spline surface object
  std::cout << "\tRange -- U: [" << surface_v.umin() << " " << surface_v.umax() << "]\n";
  std::cout << "\t         V: [" << surface_v.vmin() << " " << surface_v.vmax() << "]\n";

  //  Rasterize the surface by iterating over all of the x and y values.
  std::cout << "\tEvaluating surface.\n";
  for (int i = 0; i < input.cols(); i++) {
    for (int j = 0; j < input.rows(); j++) {
      if (i > surface_h.umin() && i < surface_h.umax() &&
          j > surface_h.vmin() && j < surface_h.vmax() &&
        i > surface_v.umin() && i < surface_v.umax() &&
        j > surface_v.vmin() && j < surface_v.vmax()) {
        output(i,j) = PixelDisparity<float>(surface_h.f((float)i,(float)j),
                                            surface_v.f((float)i,(float)j));
      } else {
        output(i,j) = PixelDisparity<float>(); // Missing pixel
      }
    }
  }  
  //   // Sample surface and print to VRML file.
  //   UCBspl::printVRMLgrid("qwe.wrl", surface, 50, 50, true);  
  return output;
}
#endif
