// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#ifndef __ASP_SURFACE_FIT_VIEW_H__
#define __ASP_SURFACE_FIT_VIEW_H__

#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/ImageMath.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/Manipulation.h>

namespace asp {

  struct PolynomialSurfaceFit {
    PolynomialSurfaceFit(double observed, double x, double y) :
      observed(observed), x(x), y(y) {}

    template <typename T>
    bool operator()(const T* const polynomial,
                    T* residuals) const {
      residuals[0] = T(observed) -
        (polynomial[0] +
         polynomial[1] * T(x) +
         polynomial[2] * T(x) * T(x) +
         polynomial[3] * T(y) +
         polynomial[4] * T(y) * T(x) +
         polynomial[5] * T(y) * T(x) * T(x) +
         polynomial[6] * T(y) * T(y) +
         polynomial[7] * T(y) * T(y) * T(x) +
         polynomial[8] * T(y) * T(y) * T(x) * T(x)
         );
      return true;
    }

    double observed, x, y;
  };

  class SurfaceFitViewBase {
  protected:
    void fit_2d_polynomial_surface( vw::ImageView<vw::PixelMask<vw::Vector2i> > const& input,
                                    vw::Matrix3x3* output_h, vw::Matrix3x3* output_v,
                                    vw::Vector2* xscaling, vw::Vector2* yscaling) const;

    // The size of output image will define how big of a render we
    // perform. This function will automatically scale the indices of the
    // output image to be between 0 -> 1 for polynomial evaluation.
    void render_polynomial_surface(vw::Matrix3x3 const& polynomial_coeff,
                                   vw::ImageView<float>* output ) const;
  };

  template <class ImageT>
  class SurfaceFitView : public vw::ImageViewBase<SurfaceFitView<ImageT> >, SurfaceFitViewBase {
    ImageT m_input;

  public:
    typedef vw::PixelMask<vw::Vector2f> pixel_type;
    typedef pixel_type result_type;
    typedef vw::ProceduralPixelAccessor<SurfaceFitView> pixel_accessor;

    SurfaceFitView( vw::ImageViewBase<ImageT> const& input ) : m_input(input.impl()) {}

    inline vw::int32 cols() const { return m_input.cols(); }
    inline vw::int32 rows() const { return m_input.rows(); }
    inline vw::int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }
    inline pixel_type operator()( vw::int32 /*i*/, vw::int32 /*j*/, vw::int32 /*p*/ = 0) const {
      using namespace vw;
      vw_throw( NoImplErr() << "PatchMatchView::operator()(....) has not been implemented." );
      return pixel_type();
    }

    // Block rasterization section that does actual work
    typedef vw::CropView<vw::ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {
      using namespace vw;

      BBox2i exp_bbox = bbox;
      exp_bbox.expand(16);

      Matrix3x3 polynomial_h, polynomial_v;
      Vector2 xscaling, yscaling;
      ImageView<PixelMask<Vector2i> > copy =
        crop(edge_extend(m_input), exp_bbox);
      fit_2d_polynomial_surface(copy,
                                &polynomial_h, &polynomial_v,
                                &xscaling, &yscaling);

      ImageView<float> fitted_h(exp_bbox.width(), exp_bbox.height()),
        fitted_v(exp_bbox.width(), exp_bbox.height());
      render_polynomial_surface(polynomial_h, &fitted_h);
      render_polynomial_surface(polynomial_v, &fitted_v);

      ImageView<pixel_type> smoothed_disparity(exp_bbox.width(),exp_bbox.height());
      fill(smoothed_disparity, pixel_type(Vector2f()));
      select_channel(smoothed_disparity, 0) = fitted_h;
      select_channel(smoothed_disparity, 1) = fitted_v;

      return prerasterize_type(smoothed_disparity,
                               -exp_bbox.min().x(), -exp_bbox.min().y(),
                               cols(), rows());
    }

    template <class DestT>
    inline void rasterize(DestT const& dest, vw::BBox2i const& bbox) const {
      vw::rasterize(prerasterize(bbox), dest, bbox);
    }

  };


  template <class ImageT>
  SurfaceFitView<ImageT>
  surface_fit( vw::ImageViewBase<ImageT> const& input ) {
    typedef SurfaceFitView<ImageT> result_type;
    return result_type(input.impl());
  }
}

#endif // __ASP_SURFACE_FIT_VIEW_H__
