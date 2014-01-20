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


/// \file stereo_fltr.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>

#include <asp/Core/BlobIndexThreaded.h>
#include <asp/Core/InpaintView.h>
#include <asp/Core/ErodeView.h>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;
using namespace asp;





//------------------------------------------------------------------------------------------------------
// Code copied from visionworkbench/src/vw//stereo/DisparityMap.h
// PROTOTYPE TO COPY
/*



  ///  remove_outliers()
  ///
  /// Replacement for old erosion based filter.
  template <class PixelT>
  class RemoveOutliersFunc2 : public ReturnFixedType<PixelT>
  {

    // This small subclass gives us the wiggle room we need to update
    // the state of this object from within the PerPixelAccessorView.
    // By maintaining a smart pointer to this small status class, we
    // can change state that is shared between any copies of the
    // RemoveOutliersFunc object and the original.
    struct RemoveOutliersState {
      int32 rejected_points, total_points;
    };

    int32 m_half_h_kernel, m_half_v_kernel;
    float m_pixel_threshold;
    float m_rejection_threshold;
    boost::shared_ptr<RemoveOutliersState> m_state;

  public:
    RemoveOutliersFunc2(int32 half_h_kernel, int32 half_v_kernel, float pixel_threshold, float rejection_threshold) :
      m_half_h_kernel(half_h_kernel), m_half_v_kernel(half_v_kernel),
      m_pixel_threshold(pixel_threshold), m_rejection_threshold(rejection_threshold),
      m_state( new RemoveOutliersState() ) {
      m_state->rejected_points = m_state->total_points = 0;

      VW_ASSERT(half_h_kernel > 0 && half_v_kernel > 0,
                ArgumentErr() << "RemoveOutliersFunc: half kernel sizes must be non-zero.");
    }

    int32 half_h_kernel() const { return m_half_h_kernel; }
    int32 half_v_kernel() const { return m_half_v_kernel; }
    float rejection_threshold() const { return m_rejection_threshold; }
    float pixel_threshold() const { return m_pixel_threshold; }
    int32 rejected_points() const { return m_state->rejected_points; }
    int32 total_points() const { return m_state->total_points; }

    BBox2i work_area() const { return BBox2i(Vector2i(-m_half_h_kernel, -m_half_v_kernel),
                                             Vector2i(m_half_h_kernel, m_half_v_kernel)); }

    // This function will be called for each pixel in the image to be filtered
    template <class PixelAccessorT>
    typename PixelAccessorT::pixel_type operator() (PixelAccessorT const& acc) const 
    {
      m_state->total_points++; // Accumulate number of points operated on

      // Quit immediately if the current point is already invalid
      if (!is_valid(*acc))
        return *acc;

      int32 matched = 0, total = 0;
      PixelAccessorT row_acc = acc;
      row_acc.advance(-m_half_h_kernel,-m_half_v_kernel); // Move to top left kernel
      for(int32 yk = -m_half_v_kernel; yk <= m_half_v_kernel; ++yk) 
      {
        PixelAccessorT col_acc = row_acc; // Start column iterator at left of current row
        for(int32 xk = -m_half_h_kernel; xk <= m_half_h_kernel; ++xk) 
        {

          if( is_valid(*col_acc) &&
              fabs((*acc)[0]-(*col_acc)[0]) <= m_pixel_threshold &&
              fabs((*acc)[1]-(*col_acc)[1]) <= m_pixel_threshold) 
          {
            matched++; // Total number of matched pixels
          }
          col_acc.next_col(); // Advance to next column
          total++; // Total number of pixels evaluated
        }
        row_acc.next_row(); // Advance to next row
      }
      if( ((float)matched/(float)total) < m_rejection_threshold)
      {
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }

      return *acc; // Return valid pixel
    }
  }; // End class RemoveOutliersFunc2

  // Useful routine for printing how many points have been rejected
  // using a particular RemoveOutliersFunc.
  template <class PixelT>
  inline std::ostream&
  operator<<(std::ostream& os, RemoveOutliersFunc2<PixelT> const& u) {
    os << "\tKernel: [ " << u.half_h_kernel()*2 << ", " << u.half_v_kernel()*2 << "]\n";
    os << "   Rejected " << u.rejected_points() << "/" << u.total_points() << " vertices ("
       << double(u.rejected_points())/u.total_points()*100 << "%).\n";
    return os;
  }

  template <class ViewT>
  UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc2<typename ViewT::pixel_type> >
  remove_outliers2(ImageViewBase<ViewT> const& disparity_map,
                  int32 half_h_kernel, int32 half_v_kernel,
                  double pixel_threshold,
                  double rejection_threshold) {
    typedef RemoveOutliersFunc2<typename ViewT::pixel_type> func_type;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, func_type > view_type;
    return view_type(edge_extend(disparity_map.impl(), ConstantEdgeExtension()),
                     func_type(half_h_kernel, half_v_kernel,
                               pixel_threshold, rejection_threshold));
  }


  /// Clean up a disparity map.
  ///
  /// You supply the half dimensions of the kernel window.
  ///
  /// Next, you supply the threshold that determines whether a
  /// pixel is considered "close" to its neightbors (in units of
  /// pixels).
  ///
  /// Finally, you supply the percentage of the pixels within the kernel
  /// that must "match" the center pixel if that pixel is to be
  /// considered an inlier. ([0..1.0]).
  template <class ViewT>
  inline UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                                              RemoveOutliersFunc2<typename ViewT::pixel_type> >, 
                                    RemoveOutliersFunc<typename ViewT::pixel_type> >
  disparity_clean_up2(ImageViewBase<ViewT> const& disparity_map,
                     int32 h_half_kernel, int32 v_half_kernel,
                     double pixel_threshold, double rejection_threshold) {
    // Remove outliers first using user specified parameters, and then
    // using a heuristic that isolates single pixel outliers.
    typedef RemoveOutliersFunc<typename ViewT::pixel_type> func_typeOrig;
    typedef RemoveOutliersFunc2<typename ViewT::pixel_type> func_typeNew;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                      func_typeNew > inner_type;
    typedef UnaryPerPixelAccessorView<inner_type,
                                      func_typeOrig > outer_type;
    return outer_type(remove_outliers2(disparity_map.impl(),
                                       h_half_kernel, v_half_kernel,
                                       pixel_threshold, rejection_threshold),
                      func_type( 1, 1, 6.0, 0.30 ) ); // Constants set to find only very isolated pixels
  }

*/
//------------------------------------------------------------------------------------------------------
// REPLACEMENT 2: Just compare the central point to the mean of the neighbors


  ///  remove_outliers2()
  ///
  /// Replacement for old erosion based filter.
  template <class PixelT>
  class RemoveOutliersFunc2 : public ReturnFixedType<PixelT>
  {

    // This small subclass gives us the wiggle room we need to update
    // the state of this object from within the PerPixelAccessorView.
    // By maintaining a smart pointer to this small status class, we
    // can change state that is shared between any copies of the
    // RemoveOutliersFunc object and the original.
    struct RemoveOutliersState {
      int32 rejected_points, total_points;
    };

    int32 m_half_h_kernel, m_half_v_kernel;
    float m_pixel_thresholdSq;
    float m_rejection_threshold;
    boost::shared_ptr<RemoveOutliersState> m_state;

  public:
  
    // TODO: rejection_threshold is not used!!!
    RemoveOutliersFunc2(int32 half_h_kernel, int32 half_v_kernel, float pixel_threshold, float rejection_threshold) :
      m_half_h_kernel(half_h_kernel), m_half_v_kernel(half_v_kernel),
      m_pixel_thresholdSq(pixel_threshold*pixel_threshold), m_rejection_threshold(rejection_threshold),
      m_state( new RemoveOutliersState() ) {
      m_state->rejected_points = m_state->total_points = 0;

      VW_ASSERT(half_h_kernel > 0 && half_v_kernel > 0,
                ArgumentErr() << "RemoveOutliersFunc: half kernel sizes must be non-zero.");
    }

    int32 half_h_kernel() const { return m_half_h_kernel; }
    int32 half_v_kernel() const { return m_half_v_kernel; }
    float rejection_threshold() const { return m_rejection_threshold; }
    float pixel_threshold() const { return sqrt(m_pixel_thresholdSq); }
    int32 rejected_points() const { return m_state->rejected_points; }
    int32 total_points() const { return m_state->total_points; }

    BBox2i work_area() const { return BBox2i(Vector2i(-m_half_h_kernel, -m_half_v_kernel),
                                             Vector2i(m_half_h_kernel, m_half_v_kernel)); }

    // This function will be called for each pixel in the image to be filtered
    template <class PixelAccessorT>
    typename PixelAccessorT::pixel_type operator() (PixelAccessorT const& acc) const 
    {
      m_state->total_points++; // Accumulate number of points operated on

      // Quit immediately if the current point is already invalid
      if (!is_valid(*acc))
        return *acc;

      size_t matched = 0, total = 0;
      double meanX=0, meanY=0;
      PixelAccessorT row_acc = acc;
      row_acc.advance(-m_half_h_kernel,-m_half_v_kernel); // Move to top left kernel
      for(int32 yk = -m_half_v_kernel; yk <= m_half_v_kernel; ++yk) 
      {
        PixelAccessorT col_acc = row_acc; // Start column iterator at left of current row
        for(int32 xk = -m_half_h_kernel; xk <= m_half_h_kernel; ++xk) 
        {
          if(is_valid(*col_acc))
          {
            meanX += (*col_acc)[0];
            meanY += (*col_acc)[1];
            matched++; // Total number of valid pixels
          }
          col_acc.next_col(); // Advance to next column
          total++; // Total number of pixels evaluated
        }
        row_acc.next_row(); // Advance to next row
      }
      // Done accumulating valid pixels, now compute the means
      double errorSq = m_pixel_thresholdSq + 1.0; // Default value ensures error if there are no valid pixels
      if (matched > 0)
      {
        meanX = meanX / static_cast<double>(matched);
        meanY = meanY / static_cast<double>(matched);
        
        // Compute squared difference of this pixel from the mean disparity
        double thisX   = (*acc)[0];
        double thisY   = (*acc)[1];
               errorSq = (thisX - meanX)*(thisX - meanX) + (thisY - meanY)*(thisY - meanY);
      }
      
      //vw_out() << "errorSq = " << errorSq << " meanX = " << meanX << " meanY = " << meanY << " m_pixel_thresholdSq = " << m_pixel_thresholdSq <<"\n";
      
      if (errorSq > m_pixel_thresholdSq) // Reject pixels too far from the mean
      {
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }

      return *acc; // Return valid pixel
    }
  }; // End class RemoveOutliersFunc2

  // Useful routine for printing how many points have been rejected
  // using a particular RemoveOutliersFunc.
  template <class PixelT>
  inline std::ostream&
  operator<<(std::ostream& os, RemoveOutliersFunc2<PixelT> const& u) {
    os << "\tKernel: [ " << u.half_h_kernel()*2 << ", " << u.half_v_kernel()*2 << "]\n";
    os << "   Rejected " << u.rejected_points() << "/" << u.total_points() << " vertices ("
       << double(u.rejected_points())/u.total_points()*100 << "%).\n";
    return os;
  }

  template <class ViewT>
  UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc2<typename ViewT::pixel_type> >
  remove_outliers2(ImageViewBase<ViewT> const& disparity_map,
                  int32 half_h_kernel, int32 half_v_kernel,
                  double pixel_threshold,
                  double rejection_threshold) {
    typedef RemoveOutliersFunc2<typename ViewT::pixel_type> func_type;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, func_type > view_type;
    return view_type(edge_extend(disparity_map.impl(), ConstantEdgeExtension()),
                     func_type(half_h_kernel, half_v_kernel,
                               pixel_threshold, rejection_threshold));
  }


  /// Clean up a disparity map.
  ///
  /// You supply the half dimensions of the kernel window.
  ///
  /// Next, you supply the threshold that determines whether a
  /// pixel is considered "close" to its neightbors (in units of
  /// pixels).
  ///
  /// Finally, you supply the percentage of the pixels within the kernel
  /// that must "match" the center pixel if that pixel is to be
  /// considered an inlier. ([0..1.0]).
  template <class ViewT>
  inline UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                                              RemoveOutliersFunc2<typename ViewT::pixel_type> >, 
                                    stereo::RemoveOutliersFunc<typename ViewT::pixel_type> >
  disparity_clean_up2(ImageViewBase<ViewT> const& disparity_map,
                     int32 h_half_kernel, int32 v_half_kernel,
                     double pixel_threshold, double rejection_threshold) {
    // Remove outliers first using user specified parameters, and then
    // using a heuristic that isolates single pixel outliers.
    typedef stereo::RemoveOutliersFunc<typename ViewT::pixel_type> func_typeOrig;
    typedef RemoveOutliersFunc2<typename ViewT::pixel_type> func_typeNew;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                      func_typeNew > inner_type;
    typedef UnaryPerPixelAccessorView<inner_type,
                                      func_typeOrig > outer_type;
    return outer_type(remove_outliers2(disparity_map.impl(),
                                       h_half_kernel, v_half_kernel,
                                       pixel_threshold, rejection_threshold),
                      func_typeOrig( 1, 1, 5.0, 0.12 ) ); // Constants set to find only very isolated pixels
                      //func_typeOrig( 1, 1, 600.0, 0.10 ) ); // Constants set to find only very isolated pixels
  }

//------------------------------------------------------------------------------------------------------
// REPLACEMENT 3: Use the standard deviation of the kernel pixels to determine valid range

  ///  remove_outliers3()
  ///
  /// Replacement for old erosion based filter.
  template <class PixelT>
  class RemoveOutliersFunc3 : public ReturnFixedType<PixelT>
  {

    // This small subclass gives us the wiggle room we need to update
    // the state of this object from within the PerPixelAccessorView.
    // By maintaining a smart pointer to this small status class, we
    // can change state that is shared between any copies of the
    // RemoveOutliersFunc object and the original.
    struct RemoveOutliersState {
      int32 rejected_points, total_points;
    };

    int32 m_half_h_kernel, m_half_v_kernel;
    float m_pixel_threshold;
    float m_rejection_threshold;
    boost::shared_ptr<RemoveOutliersState> m_state;

  public:
  
    // TODO: rejection_threshold is not used!!!
    RemoveOutliersFunc3(int32 half_h_kernel, int32 half_v_kernel, float pixel_threshold, float rejection_threshold) :
      m_half_h_kernel(half_h_kernel), m_half_v_kernel(half_v_kernel),
      m_pixel_threshold(pixel_threshold), m_rejection_threshold(rejection_threshold),
      m_state( new RemoveOutliersState() ) {
      m_state->rejected_points = m_state->total_points = 0;

      VW_ASSERT(half_h_kernel > 0 && half_v_kernel > 0,
                ArgumentErr() << "RemoveOutliersFunc: half kernel sizes must be non-zero.");
    }

    int32 half_h_kernel() const { return m_half_h_kernel; }
    int32 half_v_kernel() const { return m_half_v_kernel; }
    float rejection_threshold() const { return m_rejection_threshold; }
    float pixel_threshold() const { return m_pixel_threshold; }
    int32 rejected_points() const { return m_state->rejected_points; }
    int32 total_points() const { return m_state->total_points; }

    BBox2i work_area() const { return BBox2i(Vector2i(-m_half_h_kernel, -m_half_v_kernel),
                                             Vector2i(m_half_h_kernel, m_half_v_kernel)); }

    // This function will be called for each pixel in the image to be filtered
    template <class PixelAccessorT>
    typename PixelAccessorT::pixel_type operator() (PixelAccessorT const& acc) const 
    {
      m_state->total_points++; // Accumulate number of points operated on

      // Quit immediately if the current point is already invalid
      if (!is_valid(*acc))
        return *acc;

      // Allocate storage for recording pixel values
      const size_t numPixels = (2*m_half_v_kernel + 1) * (2*m_half_v_kernel + 1);
      std::vector<double> xVals(numPixels), yVals(numPixels);

      size_t matched = 0, total = 0;
      double meanX=0, meanY=0;
      PixelAccessorT row_acc = acc;
      row_acc.advance(-m_half_h_kernel,-m_half_v_kernel); // Move to top left kernel
      for(int32 yk = -m_half_v_kernel; yk <= m_half_v_kernel; ++yk) 
      {
        PixelAccessorT col_acc = row_acc; // Start column iterator at left of current row
        for(int32 xk = -m_half_h_kernel; xk <= m_half_h_kernel; ++xk) 
        {
          if(is_valid(*col_acc))
          {
            // Record the X and Y values
            xVals[matched] = (*col_acc)[0];
            yVals[matched] = (*col_acc)[1];           
            meanX += (*col_acc)[0];
            meanY += (*col_acc)[1];
            matched++; // Total number of valid pixels
          }
          col_acc.next_col(); // Advance to next column
          total++; // Total number of pixels evaluated
        }
        row_acc.next_row(); // Advance to next row
      }
      if (matched == 0) // Reject pixel if there are no valid neighbors
      {
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }
      // Done accumulating valid pixels, compute means
      meanX = meanX / static_cast<double>(matched);
      meanY = meanY / static_cast<double>(matched);
      
      // Now go through the pixels again to compute the standard deviation
      // - Since we recorded the pixel values in vectors this is easier
      double sumDiff=0;
      for (size_t i=0; i<matched; ++i)
      {
        double diffX  = xVals[i] - meanX;
        double diffY  = yVals[i] - meanY;
        double diffSq = diffX*diffX + diffY*diffY;
        sumDiff += diffSq;
      }
      double stdDev = sqrt(sumDiff / static_cast<double>(matched));
      
      // Compute difference of this pixel from the mean disparity
      double thisX = (*acc)[0];
      double thisY = (*acc)[1];
      double error = sqrt((thisX - meanX)*(thisX - meanX) + (thisY - meanY)*(thisY - meanY));
            
      if (error > m_pixel_threshold*stdDev) // Reject pixels greater than N deviations from the mean
      {
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }

      return *acc; // Return valid pixel
    }
  }; // End class RemoveOutliersFunc3

  // Useful routine for printing how many points have been rejected
  // using a particular RemoveOutliersFunc.
  template <class PixelT>
  inline std::ostream&
  operator<<(std::ostream& os, RemoveOutliersFunc3<PixelT> const& u) {
    os << "\tKernel: [ " << u.half_h_kernel()*2 << ", " << u.half_v_kernel()*2 << "]\n";
    os << "   Rejected " << u.rejected_points() << "/" << u.total_points() << " vertices ("
       << double(u.rejected_points())/u.total_points()*100 << "%).\n";
    return os;
  }

  template <class ViewT>
  UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc3<typename ViewT::pixel_type> >
  remove_outliers3(ImageViewBase<ViewT> const& disparity_map,
                  int32 half_h_kernel, int32 half_v_kernel,
                  double pixel_threshold,
                  double rejection_threshold) {
    typedef RemoveOutliersFunc3<typename ViewT::pixel_type> func_type;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, func_type > view_type;
    return view_type(edge_extend(disparity_map.impl(), ConstantEdgeExtension()),
                     func_type(half_h_kernel, half_v_kernel,
                               pixel_threshold, rejection_threshold));
  }


  /// Clean up a disparity map.
  ///
  /// You supply the half dimensions of the kernel window.
  ///
  /// Next, you supply the threshold that determines whether a
  /// pixel is considered "close" to its neightbors (in units of
  /// pixels).
  ///
  /// Finally, you supply the percentage of the pixels within the kernel
  /// that must "match" the center pixel if that pixel is to be
  /// considered an inlier. ([0..1.0]).
  template <class ViewT>
  inline UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                                              RemoveOutliersFunc3<typename ViewT::pixel_type> >, 
                                    stereo::RemoveOutliersFunc<typename ViewT::pixel_type> >
  disparity_clean_up3(ImageViewBase<ViewT> const& disparity_map,
                     int32 h_half_kernel, int32 v_half_kernel,
                     double pixel_threshold, double rejection_threshold) {
    // Remove outliers first using user specified parameters, and then
    // using a heuristic that isolates single pixel outliers.
    typedef stereo::RemoveOutliersFunc<typename ViewT::pixel_type> func_typeOrig;
    typedef RemoveOutliersFunc3<typename ViewT::pixel_type> func_typeNew;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                      func_typeNew > inner_type;
    typedef UnaryPerPixelAccessorView<inner_type,
                                      func_typeOrig > outer_type;
    return outer_type(remove_outliers3(disparity_map.impl(),
                                       h_half_kernel, v_half_kernel,
                                       pixel_threshold, rejection_threshold),
                      func_typeOrig( 1, 1, 6.0, 0.30 ) ); // Constants set to find only very isolated pixels
  }


//------------------------------------------------------------------------------------------------------
// REPLACEMENT 4: Fit a plane to the other pixels and see how well the test pixel fits the plane

  /// Compute the plane that best fits a set of 3D points
  /// - The plane is described as z = ax + by + c (the output vector contains [a, b, c]
  bool fitPlaneToPoints(const std::vector<Vector3> &points, Vector3 &planeDesc)
  {
    const size_t X = 0; // Convenience constants
    const size_t Y = 1;
    const size_t Z = 2;
    const size_t numPoints = points.size();
    
    // Compute values in a matrix A and vector b
    //A: [xx  xy  x]     B: [xz]
    //   [xy  yy  y]        [yz]
    //   [ x   y  n]        [ z]
    Matrix3x3 matA(0, 0, 0, 0, 0, 0, 0, 0, 0); // A symmetric matrix, A' = A
    Vector3   vecB(0, 0, 0);
    for (size_t i=0; i<numPoints; ++i)
    {
      matA[0][0] += points[i][X] * points[i][X]; // sum xx
      matA[0][1] += points[i][X] * points[i][Y]; // sum xy
      matA[0][2] += points[i][X];                // sum x
      matA[1][0] += points[i][X] * points[i][Y]; // sum xy
      matA[1][1] += points[i][Y] * points[i][Y]; // sum yy
      matA[1][2] += points[i][Y];                // sum y
      matA[2][0] += points[i][X];                // sum x
      matA[2][1] += points[i][Y];                // sum y
      
      vecB[0]    += points[i][X] * points[i][Z]; // sum xz
      vecB[1]    += points[i][Y] * points[i][Z]; // sum yz
      vecB[2]    += points[i][Z];                // sum z
    }
    matA[2][2] = numPoints; // n
        
    // Now solve Ax = b (3x3)*(3x1) = (3x1)
    planeDesc = vw::math::solve(matA, vecB); // Throws!
    /*
    try
    {
      planeDesc = vw::math::solve(matA, vecB); // Throws!
    }
    catch (...)
    {
      
      vw_out() << "Failed to solve matrix: \n";
      for (int r=0; r<3; ++r)
      {
        for (int c=0; c<3; ++c)
        {
          vw_out() << matA[r][c] << " ";
        }
        vw_out() << "         " << vecB[r] << "\n";
      }
      vw_out() << "With points:\n";
      for (size_t i=0; i<numPoints; ++i)
      {
        vw_out() << points[i] << "\n";
      }
    }
    for (int r=0; r<3; ++r)
    { 
      vw_out() << planeDesc[r] << " ";
    }
    vw_out() << "\n";
*/    
    return true;
  }

  /// Computes the distance from a point to a plane
  double pointToPlaneDist(const Vector3 &point, const Vector3 &planeDesc)
  {
    // Convert plane from format z = ax + by + c to format 0 = ax + by + cz + d = 0
    double a = planeDesc[0];
    double b = planeDesc[1];
    double c = -1.0;
    double d = planeDesc[2];
   
    double numerator   = fabs(a*point[0] + b*point[1] + c*point[2] + d);
    double denominator = sqrt(a*a + b*b + c*c);
   
    //vw_out() << "planeDesc = " << planeDesc << " point = " << point << " numerator = " << numerator << " numerator = " << numerator << " denominator = " << denominator<<"\n";
   
    //TODO: This will crash if an invalid plane is given! 
    return numerator / denominator;
  }

  /// Compute statistics for how well points fit a plane 
  bool checkPointToPlaneFit(const std::vector<Vector3> &points, const Vector3 &planeDesc,
                            double &meanError, double &stdDevError)
  {
    const size_t numPoints = points.size();
    std::vector<double> dists(numPoints);
    
    // Compute the mean
    meanError = 0;
    for (size_t i=0; i<numPoints; ++i)
    {
      dists[i] = pointToPlaneDist(points[i], planeDesc);
      meanError += dists[i];
    }
    meanError /= static_cast<double>(numPoints);
    
    // Compute the standard deviation
    double sumDiff = 0;
    for (size_t i=0; i<numPoints; ++i)
    {
      //double diffSq = (dists[i] - meanError) * (dists[i] - meanError);
      double diffSq = dists[i]*dists[i];
      sumDiff += diffSq;
    }
    stdDevError = sqrt(sumDiff / static_cast<double>(numPoints));
    
    return true;
  }
  


  ///  remove_outliers4()
  ///
  /// Replacement for old erosion based filter.
  template <class PixelT>
  class RemoveOutliersFunc4 : public ReturnFixedType<PixelT>
  {

    // This small subclass gives us the wiggle room we need to update
    // the state of this object from within the PerPixelAccessorView.
    // By maintaining a smart pointer to this small status class, we
    // can change state that is shared between any copies of the
    // RemoveOutliersFunc object and the original.
    struct RemoveOutliersState {
      int32 rejected_points, total_points;
    };

    int32 m_half_h_kernel, m_half_v_kernel;
    float m_pixel_threshold;
    float m_rejection_threshold;
    boost::shared_ptr<RemoveOutliersState> m_state;

  public:
  
    // TODO: rejection_threshold is not used!!!
    RemoveOutliersFunc4(int32 half_h_kernel, int32 half_v_kernel, float pixel_threshold, float rejection_threshold) :
      m_half_h_kernel(half_h_kernel), m_half_v_kernel(half_v_kernel),
      m_pixel_threshold(pixel_threshold), m_rejection_threshold(rejection_threshold),
      m_state( new RemoveOutliersState() ) {
      m_state->rejected_points = m_state->total_points = 0;

      VW_ASSERT(half_h_kernel > 0 && half_v_kernel > 0,
                ArgumentErr() << "RemoveOutliersFunc: half kernel sizes must be non-zero.");
    }

    int32 half_h_kernel() const { return m_half_h_kernel; }
    int32 half_v_kernel() const { return m_half_v_kernel; }
    float rejection_threshold() const { return m_rejection_threshold; }
    float pixel_threshold() const { return m_pixel_threshold; }
    int32 rejected_points() const { return m_state->rejected_points; }
    int32 total_points() const { return m_state->total_points; }

    BBox2i work_area() const { return BBox2i(Vector2i(-m_half_h_kernel, -m_half_v_kernel),
                                             Vector2i(m_half_h_kernel, m_half_v_kernel)); }

    // This function will be called for each pixel in the image to be filtered
    template <class PixelAccessorT>
    typename PixelAccessorT::pixel_type operator() (PixelAccessorT const& acc) const 
    {
      m_state->total_points++; // Accumulate number of points operated on

      // Quit immediately if the current point is already invalid
      if (!is_valid(*acc))
        return *acc;

      // Allocate storage for recording pixel values
      const size_t numPixels = (2*m_half_v_kernel + 1) * (2*m_half_v_kernel + 1);
      std::vector<double> xVals(numPixels), yVals(numPixels);

      // Record all valid points as x/y/z pairs (one set for dX, one set for dY)
      size_t matched = 0, total = 0;
      std::vector<Vector3> xVec, yVec;
      xVec.reserve(numPixels);
      yVec.reserve(numPixels);
      PixelAccessorT row_acc = acc;
      row_acc.advance(-m_half_h_kernel,-m_half_v_kernel); // Move to top left kernel
      for(int32 yk = -m_half_v_kernel; yk <= m_half_v_kernel; ++yk) 
      {
        PixelAccessorT col_acc = row_acc; // Start column iterator at left of current row
        for(int32 xk = -m_half_h_kernel; xk <= m_half_h_kernel; ++xk) 
        {
          if(is_valid(*col_acc))
          {
            // Record the X and Y values
            xVec.push_back(Vector3(xk, yk, (*col_acc)[0]));
            yVec.push_back(Vector3(xk, yk, (*col_acc)[1]));
            matched++; // Total number of valid pixels
          }
          col_acc.next_col(); // Advance to next column
          total++; // Total number of pixels evaluated
        }
        row_acc.next_row(); // Advance to next row
      }
      if (matched == 0) // Reject pixel if there are no valid neighbors
      {
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }

      //TODO: eliminate outliers here

      // Compute a best fit plane for the dX and dY values
      Vector3 xPlane, yPlane;
      try
      {
        fitPlaneToPoints(xVec, xPlane);
        fitPlaneToPoints(yVec, yPlane);
      }
      catch(...) // Failed to solve, probably because points were in a line
      {
        //TODO: Have a fallback check (maybe a line fit) in this case!
        return *acc; // Return valid pixel        
      }

      // Now determine the quality of the plane fits
      double xMean, yMean, xStdDev, yStdDev;
      checkPointToPlaneFit(xVec, xPlane, xMean, xStdDev);
      checkPointToPlaneFit(yVec, yPlane, yMean, yStdDev);
      
      // Determine if the test pixel is too far from either plane
      Vector3 thisLocX(0,0,(*acc)[0]);
      Vector3 thisLocY(0,0,(*acc)[1]);
      double errorX = pointToPlaneDist(thisLocX, xPlane);
      double errorY = pointToPlaneDist(thisLocY, yPlane);
            
      //vw_out() << "errorX = " << errorX << " errorY = " << errorY << " meanX = " << xMean << " meanY = " << yMean << " xStdDev = " << xStdDev << " yStdDev = " << yStdDev << " m_pixel_threshold = " << m_pixel_threshold <<"\n";
            
      if ( (errorX > m_pixel_threshold*xStdDev) ||  // Reject pixels greater than N deviations from the mean
           (errorY > m_pixel_threshold*yStdDev) )
      {
        //vw_out() << "errorX = " << errorX << " errorY = " << errorY << " meanX = " << xMean << " meanY = " << yMean << " xStdDev = " << xStdDev << " yStdDev = " << yStdDev << " m_pixel_threshold = " << m_pixel_threshold <<"\n";
      
        m_state->rejected_points++;
        return typename PixelAccessorT::pixel_type();  // Return invalid pixel
      }

      return *acc; // Return valid pixel
    }
  }; // End class RemoveOutliersFunc3

  // Useful routine for printing how many points have been rejected
  // using a particular RemoveOutliersFunc.
  template <class PixelT>
  inline std::ostream&
  operator<<(std::ostream& os, RemoveOutliersFunc4<PixelT> const& u) {
    os << "\tKernel: [ " << u.half_h_kernel()*2 << ", " << u.half_v_kernel()*2 << "]\n";
    os << "   Rejected " << u.rejected_points() << "/" << u.total_points() << " vertices ("
       << double(u.rejected_points())/u.total_points()*100 << "%).\n";
    return os;
  }

  template <class ViewT>
  UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc4<typename ViewT::pixel_type> >
  remove_outliers4(ImageViewBase<ViewT> const& disparity_map,
                  int32 half_h_kernel, int32 half_v_kernel,
                  double pixel_threshold,
                  double rejection_threshold) {
    typedef RemoveOutliersFunc4<typename ViewT::pixel_type> func_type;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, func_type > view_type;
    return view_type(edge_extend(disparity_map.impl(), ConstantEdgeExtension()),
                     func_type(half_h_kernel, half_v_kernel,
                               pixel_threshold, rejection_threshold));
  }


  /// Clean up a disparity map.
  ///
  /// You supply the half dimensions of the kernel window.
  ///
  /// Next, you supply the threshold that determines whether a
  /// pixel is considered "close" to its neightbors (in units of
  /// pixels).
  ///
  /// Finally, you supply the percentage of the pixels within the kernel
  /// that must "match" the center pixel if that pixel is to be
  /// considered an inlier. ([0..1.0]).
  template <class ViewT>
  inline UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                                              RemoveOutliersFunc4<typename ViewT::pixel_type> >, 
                                    stereo::RemoveOutliersFunc<typename ViewT::pixel_type> >
  disparity_clean_up4(ImageViewBase<ViewT> const& disparity_map,
                     int32 h_half_kernel, int32 v_half_kernel,
                     double pixel_threshold, double rejection_threshold) {
    // Remove outliers first using user specified parameters, and then
    // using a heuristic that isolates single pixel outliers.
    typedef stereo::RemoveOutliersFunc<typename ViewT::pixel_type> func_typeOrig;
    typedef RemoveOutliersFunc4<typename ViewT::pixel_type> func_typeNew;
    typedef UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>,
                                      func_typeNew > inner_type;
    typedef UnaryPerPixelAccessorView<inner_type,
                                      func_typeOrig > outer_type;
    return outer_type(remove_outliers4(disparity_map.impl(),
                                       h_half_kernel, v_half_kernel,
                                       pixel_threshold, rejection_threshold),
                      func_typeOrig( 1, 1, 6.0, 0.30 ) ); // Constants set to find only very isolated pixels
  }


//------------------------------------------------------------------------------------------------------



namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

// This uses a struct as partial template specialization is not
// allowed for functions in C++.
template <class ViewT, int N>
struct MultipleDisparityCleanUp {

  MultipleDisparityCleanUp<ViewT,N-1> inner_func;

  typedef typename MultipleDisparityCleanUp<ViewT,N-1>::result_type inner_type;
  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<inner_type,ConstantEdgeExtension>, stereo::RemoveOutliersFunc<typename inner_type::pixel_type> >, stereo::RemoveOutliersFunc<typename inner_type::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres )  {
    return stereo::disparity_clean_up(
                                      inner_func(input.impl(), h_half_kern,
                                                 v_half_kern, pixel_thres,
                                                 rej_thres), h_half_kern,
                                      v_half_kern, pixel_thres, rej_thres);
  }
};

//TODO: The N template value has been hijacked to try out different versions without modifying any of the config loading code!

// Current version (compare each pixel)
template <class ViewT>
struct MultipleDisparityCleanUp<ViewT,1> {

  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> >, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres ) {
    vw_out() <<"\nRUNNING ORIGINAL VERSION\n";
    return stereo::disparity_clean_up (input.impl(), h_half_kern, v_half_kern, pixel_thres, rej_thres); 
  }
};


// Arra's version (compare to mean of pixels)
template <class ViewT>
struct MultipleDisparityCleanUp<ViewT,2> {
  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc2<typename ViewT::pixel_type> >, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres ) {
    vw_out() <<"\nRUNNING MEAN VERSION\n";
    return disparity_clean_up2(input.impl(), h_half_kern, v_half_kern, pixel_thres, rej_thres); 
  }
};

// Compare against mean with standard deviation
template <class ViewT>
struct MultipleDisparityCleanUp<ViewT,3> {
  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc3<typename ViewT::pixel_type> >, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres ) {
    vw_out() <<"\nRUNNING STD VERSION\n";
    return disparity_clean_up3(input.impl(), h_half_kern, v_half_kern, pixel_thres, rej_thres); 
  }
};  
  
// Compare against plane fits to dX/dY values
template <class ViewT>
struct MultipleDisparityCleanUp<ViewT,4> {
  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, RemoveOutliersFunc4<typename ViewT::pixel_type> >, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres ) {
    vw_out() <<"\nRUNNING PLANE FIT VERSION\n";
    return disparity_clean_up4(input.impl(), h_half_kern, v_half_kern, pixel_thres, rej_thres); 
  }   
};

template <class ImageT>
void write_good_pixel_and_filtered( ImageViewBase<ImageT> const& inputview,
                                    Options const& opt ) {
  { // Write Good Pixel Map
    // Sub-sampling so that the user can actually view it.
    float sub_scale =
      float( std::min( inputview.impl().cols(),
                       inputview.impl().rows() ) ) / 2048.0;

    asp::block_write_gdal_image( opt.out_prefix + "-GoodPixelMap.tif",
                                 subsample(
                                   apply_mask(
                                     copy_mask(stereo::missing_pixel_image(inputview.impl()),
                                               create_mask(DiskImageView<vw::uint8>(opt.out_prefix+"-lMask.tif"),
                                                           0))),
                                   sub_scale < 1 ? 1 : sub_scale ),
                                 opt, TerminalProgressCallback("asp", "\t--> Good Pxl Map: ") );
  }

  // Fill holes
  if(!stereo_settings().disable_fill_holes) {
    vw_out() << "\t--> Filling holes with Inpainting method.\n";
    BlobIndexThreaded bindex( invert_mask( inputview.impl() ),
                              stereo_settings().fill_hole_max_size );
    vw_out() << "\t    * Identified " << bindex.num_blobs() << " holes\n";
    bool use_grassfire = true;
    typename ImageT::pixel_type default_inpaint_val;
    asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                 inpaint(inputview.impl(), bindex,
                                         use_grassfire, default_inpaint_val),
                                 opt, TerminalProgressCallback("asp","\t--> Filtering: ") );

  } else {
    asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                 inputview.impl(), opt,
                                 TerminalProgressCallback("asp", "\t--> Filtering: ") );
  }
}

void stereo_filtering( Options& opt ) {
  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 3 --> FILTERING \n";

  std::string post_correlation_fname;
  opt.session->pre_filtering_hook(opt.out_prefix+"-RD.tif",
                                  post_correlation_fname);

  try {

    // Rasterize the results so far to a temporary file on disk.
    // This file is deleted once we complete the second half of the
    // disparity map filtering process.
    {
      // Apply filtering for high frequencies
      typedef DiskImageView<PixelMask<Vector2f> > input_type;
      input_type disparity_disk_image(post_correlation_fname);

      // Applying additional clipping from the edge. We make new
      // mask files to avoid a weird and tricky segfault due to
      // ownership issues.
      DiskImageView<vw::uint8> left_mask( opt.out_prefix+"-lMask.tif" );
      DiskImageView<vw::uint8> right_mask( opt.out_prefix+"-rMask.tif" );
      int32 mask_buffer = max( stereo_settings().subpixel_kernel );

      vw_out() << "\t--> Cleaning up disparity map prior to filtering processes ("
               << stereo_settings().rm_cleanup_passes << " pass).\n";
      if ( stereo_settings().mask_flatfield ) 
      {
        vw_out() << "Mask flatfield\n";
        ImageViewRef<PixelMask<Vector2f> > filtered_disparity;
        if ( stereo_settings().rm_cleanup_passes == 1 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,1>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
                             
        else if ( stereo_settings().rm_cleanup_passes == 2 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,2>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else if ( stereo_settings().rm_cleanup_passes == 3 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,3>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else if ( stereo_settings().rm_cleanup_passes >= 4 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,4>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else // stereo_settings().rm_cleanup_passes == 0
          filtered_disparity =
            stereo::disparity_mask(disparity_disk_image,
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));

        // This is only turned on for apollo. Blob detection doesn't
        // work to great when tracking a whole lot of spots. HiRISE
        // seems to keep breaking this so I've keep it turned off.
        //
        // The crash happens inside Boost Graph when dealing with
        // large number of blobs.
        BlobIndexThreaded bindex( filtered_disparity,
                                  stereo_settings().erode_max_size );
        vw_out() << "\t    * Eroding " << bindex.num_blobs() << " islands\n";
        write_good_pixel_and_filtered( ErodeView<ImageViewRef<PixelMask<Vector2f> > >(filtered_disparity, bindex ),
                                       opt );
      } else {
        // No Erosion step
        if ( stereo_settings().rm_cleanup_passes == 1 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,1>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes == 2 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,2>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes == 3 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,3>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes >= 4 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,4>()(
                                               disparity_disk_image,
                                               stereo_settings().rm_half_kernel.x(),
                                               stereo_settings().rm_half_kernel.y(),
                                               stereo_settings().rm_threshold,
                                               stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else
          write_good_pixel_and_filtered(
            stereo::disparity_mask(disparity_disk_image,
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt );
      }
    }
  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      FilteringDescription() );

    // Internal Processes
    //---------------------------------------------------------
    stereo_filtering( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : FILTERING FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
