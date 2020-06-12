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

/// \file OrthoRasterizer.h
///
/// Given a point image and corresponding texture, this class
/// bins and averages the point cloud on a regular grid over the [x,y]
/// plane of the point image; producing an evenly sampled ortho-image
/// with interpolated z values.

#ifndef __ASP_CORE_ORTHORASTERIZER_H__
#define __ASP_CORE_ORTHORASTERIZER_H__

#include <vw/Core/Thread.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <asp/Core/Point2Grid.h>

namespace asp{

  using namespace vw;

  typedef std::pair<BBox3, BBox2i> BBoxPair;

  /// Given a point image and corresponding texture, this class
  /// bins and averages the point cloud on a regular grid over the [x,y]
  /// plane of the point image; producing an evenly sampled ortho-image
  /// with interpolated z values.
  class OrthoRasterizerView:
    public ImageViewBase<OrthoRasterizerView> {
    ImageViewRef<Vector3> m_point_image;
    ImageViewRef<float>   m_texture;
    BBox3   m_bbox, m_snapped_bbox; // bounding box of point cloud
    double  m_spacing;         // point cloud units (usually m or deg) per pixel
    double  m_default_spacing; // if user did not specify spacing
    double  m_default_spacing_x;
    double  m_default_spacing_y;
    double  m_search_radius_factor;
    double  m_sigma_factor;
    bool    m_use_surface_sampling;
    double  m_default_value;
    bool    m_minz_as_default;
    bool    m_use_alpha;
    int     m_block_size;
    BBox2   m_projwin;
    ImageViewRef<double> const& m_error_image;
    double  m_error_cutoff;
    Vector2 m_median_filter_params;
    int     m_erode_len;
    asp::FilterType m_filter;
    double m_percentile;
    double m_default_grid_size_multiplier;
    size_t     *m_num_invalid_pixels; ///< Keep a count of nodata output pixels, needs to be pointer due to VW weirdness.
    vw::Mutex  *m_count_mutex;        ///< A lock for m_num_invalid_pixels, needs to be pointer due to C++ weirdness.

    // We could actually use a quadtree here .. but this should be a
    // good enough improvement.
    std::vector<BBoxPair> m_point_image_boundaries;
    // These boundaries describe a point cloud 3D boundaries and then
    // their location in the the point cloud image. These boxes are
    // overlapping in the pc image X/Y domain to insure that
    // everything is triangulated.

    // Function to convert pixel coordinates to the point domain
    BBox3 pixel_to_point_bbox( BBox2 const& px ) const;

  public:
    typedef PixelGray<float> pixel_type;
    typedef const PixelGray<float> result_type;
    typedef ProceduralPixelAccessor<OrthoRasterizerView> pixel_accessor;
    static int max_subblock_size(){ return 128;} // is used in point2dem and below

    /// Constructor.  You must call initialize_spacing before using the object!!
    OrthoRasterizerView(ImageViewRef<Vector3> point_image,
                        ImageViewRef<double > texture,
                        double  search_radius_factor,
                        double  sigma_factor,
                        bool    use_surface_sampling,
                        int     pc_tile_size,
                        vw::BBox2 const& projwin,
                        bool    remove_outliers_with_pct,
                        Vector2 const& remove_outliers_params,
                        ImageViewRef<double> const& error_image,
                        double  estim_max_error,
                        vw::BBox3 const& estim_proj_box,
                        double  max_valid_triangulation_error,
                        Vector2 median_filter_params,
                        int     erode_len,
                        bool    has_las_or_csv,
                        std::string const& filter,
                        double  default_grid_size_multiplier,
                        size_t  *num_invalid_pixels,
                        vw::Mutex *count_mutex,
                        const ProgressCallback& progress);

    /// This must be called before the object can be used!
    void initialize_spacing(double spacing=0.0);

    /// You can change the texture after the class has been
    /// initialized.  The texture image must have the same dimensions
    /// as the point image, and texture pixels must correspond exactly
    /// to point image pixels.
    template <class TextureViewT>
    void set_texture(TextureViewT texture) {
      VW_ASSERT(texture.impl().cols() == m_point_image.cols() &&
                texture.impl().rows() == m_point_image.rows(),
      ArgumentErr() << "Orthorasterizer: set_texture() failed."
                    << " Texture dimensions must match point image dimensions.");
      m_texture = channel_cast<float>(channels_to_planes(texture.impl()));
    }

    inline int32 cols() const {return (int)round((fabs(m_snapped_bbox.max().x() - m_snapped_bbox.min().x()) / m_spacing)) + 1;}
    inline int32 rows() const {return (int)round((fabs(m_snapped_bbox.max().y() - m_snapped_bbox.min().y()) / m_spacing)) + 1;}

    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( int /*i*/, int /*j*/, int /*p*/=0 ) const {
      vw_throw(NoImplErr() << "OrthoRasterizersView::operator()(double i, double j, int32 p) has not been implemented.");
      return pixel_type();
    }

    /// \cond INTERNAL
    typedef CropView<ImageView<pixel_type> > prerasterize_type;
    prerasterize_type prerasterize( BBox2i const& bbox ) const;

    template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }
    /// \endcond

    void set_use_alpha          (bool   val) { m_use_alpha       = val; }
    void set_use_minz_as_default(bool   val) { m_minz_as_default = val; }
    void set_default_value      (double val) { m_default_value   = val; }
    double default_value() {
      if (m_minz_as_default) return m_bbox.min().z();
      else return m_default_value;
    }

    /// If the DEM spacing is set to zero, we compute a DEM with
    /// approximately the same pixel dimensions as the input image.
    /// Note, however, that this could lead to a loss in DEM
    /// resolution if the DEM is rotated from the orientation of the
    /// original image.
    void set_spacing(double val) {
      if (val == 0.0) {
        m_spacing = m_default_spacing * m_default_grid_size_multiplier;
      } else {
        m_spacing = val;
      }

    }

    double spacing() const { return m_spacing; }

    // Convert the hole fill length from output image pixels to point cloud pixels.
    int pc_hole_fill_len(int hole_fill_len){

      if (hole_fill_len ==0) return 0;
        
      VW_ASSERT(m_spacing > 0 && m_default_spacing > 0,
                ArgumentErr() << "Expecting positive DEM spacing.");
      return (int)round((m_spacing/m_default_spacing)*hole_fill_len);
    }

    BBox3 bounding_box() const { return m_snapped_bbox; }

    // Return the affine georeferencing transform.
    vw::Matrix<double,3,3> geo_transform();

    ImageViewRef<Vector3> get_point_image() { return m_point_image; }
    
    void set_point_image(ImageViewRef<Vector3> point_image) {m_point_image = point_image;}
    
  };

  // TODO: Make this a BBox class function!!!
  /// Snaps the coordinates of a BBox to a grid spacing
  template <size_t N>
  void snap_bbox(const double spacing, BBox<double, N> &bbox ) {
    bbox.min() = spacing*floor(bbox.min()/spacing);
    bbox.max() = spacing*ceil (bbox.max()/spacing);
  }

} // namespace asp

#endif
