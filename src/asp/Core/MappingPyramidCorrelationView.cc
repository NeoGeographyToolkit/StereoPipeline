#include <asp/Core/MappingPyramidCorrelationView.h>

using namespace vw;

void asp::MappingPyramidCorrelationViewBase::blur_disparity(vw::ImageView<PixelMask<Vector2f> >& sf_disparity,
                                                            BBox2i const& disparity_bounds) const {
  select_channel(sf_disparity,0) =
    clamp(gaussian_filter(select_channel(sf_disparity,0),5),
          disparity_bounds.min()[0],
          disparity_bounds.max()[0]);
  select_channel(sf_disparity,1) =
    clamp(gaussian_filter(select_channel(sf_disparity,1),5),
          disparity_bounds.min()[1],
          disparity_bounds.max()[1]);
}

void asp::MappingPyramidCorrelationViewBase::copy_valid(vw::ImageView<PixelMask<Vector2f> >& destination,
                                                        vw::ImageView<PixelMask<Vector2f> >& source) const {
  for (int j = 0; j < destination.rows(); j++ ) {
    for (int i = 0; i < destination.cols(); i++ ) {
      if (is_valid(source(i,j))) {
        destination(i,j) = source(i,j);
      }
    }
  }
}
