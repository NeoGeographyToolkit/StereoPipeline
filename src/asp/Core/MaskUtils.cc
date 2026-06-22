// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file MaskUtils.cc

#include <asp/Core/MaskUtils.h>

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/PixelAccessors.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Grassfire.h>
#include <vw/Core/Exception.h>

using namespace vw;

namespace {

// A lazy view that grows the valid region of a mask by 'radius' pixels. Each
// requested tile is computed on demand from the input over a collar of
// radius + 1 pixels. Grassfire then gives the Manhattan distance from each pixel
// to the nearest valid pixel (with the sub-image border treated as valid, as in
// vw::grassfire); a pixel is kept valid if that distance is within 'radius'. The
// collar makes each tile's result identical to grassfire run over the whole
// image: any valid pixel within 'radius' of a tile pixel is inside the collar,
// and the sub-image border sits radius + 1 away from the tile, so its phantom
// validity never reaches the tile. Where the collar meets the real image border,
// that border is treated as valid in both cases, so they agree there too.
class ExpandMaskView: public ImageViewBase<ExpandMaskView> {
  ImageViewRef<PixelMask<uint8>> m_mask;
  int m_radius;

public:
  typedef PixelMask<uint8> pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<ExpandMaskView> pixel_accessor;

  ExpandMaskView(ImageViewRef<PixelMask<uint8>> const& mask, int radius):
    m_mask(mask), m_radius(radius) {}

  inline int32 cols  () const { return m_mask.cols(); }
  inline int32 rows  () const { return m_mask.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()(int32 /*i*/, int32 /*j*/, int32 /*p*/ = 0) const {
    vw_throw(NoImplErr() << "ExpandMaskView::operator() is not implemented. "
             << "Access this view via rasterization.");
    return result_type();
  }

  typedef CropView<ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Expand the requested tile by a collar of radius + 1, clipped to the image.
    BBox2i biased = bbox;
    biased.expand(m_radius + 1);
    biased.crop(bounding_box(m_mask));

    // Rasterize the input over the collar.
    ImageView<pixel_type> sub = crop(m_mask, biased);

    // is_invalid is 1 where invalid, 0 where valid. Grassfire then gives the
    // Manhattan distance to the nearest valid pixel (0 at valid pixels).
    ImageView<uint8> is_invalid(sub.cols(), sub.rows());
    for (int row = 0; row < sub.rows(); row++)
      for (int col = 0; col < sub.cols(); col++)
        is_invalid(col, row) = is_valid(sub(col, row)) ? 0 : 1;

    ImageView<int32> dist = grassfire(is_invalid);

    // Keep a pixel valid if a valid pixel is within 'radius'.
    ImageView<pixel_type> out(sub.cols(), sub.rows());
    for (int row = 0; row < sub.rows(); row++) {
      for (int col = 0; col < sub.cols(); col++) {
        out(col, row) = pixel_type(255);
        if (dist(col, row) > m_radius)
          out(col, row).invalidate();
      }
    }

    // Wrap so the tile's coordinates look like the full image. The ImageView is
    // reference-counted, so it stays alive in the returned crop.
    return crop(out, -biased.min().x(), -biased.min().y(), cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

} // end anonymous namespace

namespace asp {

ImageViewRef<PixelMask<uint8>>
expandMask(ImageViewRef<PixelMask<uint8>> const& mask, int radius) {
  if (radius <= 0)
    return mask;
  return ExpandMaskView(mask, radius);
}

} // namespace asp
