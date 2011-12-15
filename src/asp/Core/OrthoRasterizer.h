// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file OrthoRasterizer.h
///
/// Given a point image and corresponding texture, this class
/// resamples the point cloud on a regular grid over the [x,y] plane
/// of the point image; producing an evenly sampled ortho-image with
/// interpolated z values.

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/BlockRasterize.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

// The SoftwareRenderer actual "renders" the 3D scene, textures it,
// and then returns a 2D orthographic view.
#include <asp/Core/SoftwareRenderer.h>
#include <boost/math/special_functions/next.hpp>

namespace vw {
namespace cartography {

  template <class PixelT, class ImageT>
  class OrthoRasterizerView : public ImageViewBase<OrthoRasterizerView<PixelT, ImageT> > {
    ImageT m_point_image;
    ImageViewRef<float> m_texture;
    BBox3 m_bbox;
    double m_spacing;
    double m_default_value;
    bool m_minz_as_default;
    bool m_use_alpha;

    // We could actually use a quadtree here .. but this should be a
    // good enough improvement.
    typedef std::pair<BBox3, BBox2i> BBoxPair;
    std::vector<BBoxPair > m_point_image_boundaries;

    struct GrowBBoxAccumulator {
      BBox3 bbox;
      void operator()( Vector3 const& v ) { if (v != Vector3()) bbox.grow(v); }
    };

    struct RemoveSoftInvalid : ReturnFixedType<PixelT> {
      template <class T>
      PixelT operator()( T const& v ) const {
        if ( v == -32000 )
          return PixelT();
        return v;
      }
    };

  public:
    typedef PixelT pixel_type;
    typedef const PixelT result_type;
    typedef ProceduralPixelAccessor<OrthoRasterizerView> pixel_accessor;

    template <class TextureViewT>
    OrthoRasterizerView(ImageT point_cloud, TextureViewT texture, double spacing = 0.0,
                        const ProgressCallback& progress = ProgressCallback::dummy_instance()) :
      m_point_image(point_cloud), m_texture(ImageView<float>(1,1)), // dummy value
      m_default_value(0), m_minz_as_default(true), m_use_alpha(false) {

      set_texture(texture.impl());

      // Compute the bounding box that encompasses tiles within the image
      vw_out(DebugMessage) << "Computing raster bounding box...\n";

      static const int32 BBOX_SPACING = 256;
      int32 x_divisions = (m_point_image.cols() - 1)/BBOX_SPACING + 1;
      int32 y_divisions = (m_point_image.rows() - 1)/BBOX_SPACING + 1;
      BBox2i image_bbox(0,0,m_point_image.cols(),m_point_image.rows());

      for ( int32 yi = 0; yi < y_divisions; ++yi ) {
        progress.report_fractional_progress(yi,y_divisions);
        for ( int32 xi = 0; xi < x_divisions; ++xi ) {
          BBox2i local_spot( xi * BBOX_SPACING, yi * BBOX_SPACING,
                             BBOX_SPACING, BBOX_SPACING );
          local_spot.crop( image_bbox );

          // The tiles need to overlap so we extend max by 1 px.
          if ( local_spot.max()[0] != image_bbox.max()[0] )
            ++local_spot.max()[0];
          if ( local_spot.max()[1] != image_bbox.max()[1] )
            ++local_spot.max()[1];

          GrowBBoxAccumulator accum;
          for_each_pixel( crop(m_point_image,local_spot), accum );
          if ( !accum.bbox.empty() ) {
            // HACK UNTIL BBOX is fixed!
            accum.bbox.max()[0] = boost::math::float_next(accum.bbox.max()[0]);
            accum.bbox.max()[1] = boost::math::float_next(accum.bbox.max()[1]);
            m_point_image_boundaries.push_back( std::make_pair( accum.bbox, local_spot ) );
            m_bbox.grow( accum.bbox );
          }
        }
      }
      progress.report_finished();

      if ( m_bbox.empty() )
        vw_throw( ArgumentErr() << "OrthoRasterize: Input point cloud is empty!\n" );

      // Set the sampling rate (i.e. spacing between pixels)
      this->set_spacing(spacing);
    }

    /// You can change the texture after the class has been
    /// initialized.  The texture image must have the same dimensions
    /// as the point image, and texture pixels must correspond exactly
    /// to point image pixels.
    template <class TextureViewT>
    void set_texture(TextureViewT texture) {
      VW_ASSERT(texture.impl().cols() == m_point_image.cols() && texture.impl().rows() == m_point_image.rows(),
                ArgumentErr() << "Orthorasterizer: set_texture() failed."
                << " Texture dimensions must match point image dimensions.");
      m_texture = channel_cast<float>(channels_to_planes(texture.impl()));
    }

    inline int32 cols() const { return (int) (fabs(m_bbox.max().x() - m_bbox.min().x()) / m_spacing) + 1; }
    inline int32 rows() const { return (int) (fabs(m_bbox.max().y() - m_bbox.min().y()) / m_spacing) + 1; }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( int /*i*/, int /*j*/, int /*p*/=0 ) const {
      vw_throw(NoImplErr() << "OrthoRasterizersView::operator()(double i, double j, int32 p) has not been implemented.");
      return pixel_type();
    }

    /// \cond INTERNAL
    typedef CropView<ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize( BBox2i const& bbox ) const {

      // Buffer the bounding box slightly so that we render a few
      // extra triangles that would have fallen off the border
      // otherwise.
      //
      // This is a quality option, if this buffer is not large enough
      // we might not rasterize faces which are large, due to slope
      // from the camera perspective. Realize this data is still
      // gridded in the camera perspective.
      BBox2i buffered_bbox = bbox;
      buffered_bbox.expand(std::max(bbox.width(),bbox.height())/16);

      // This ensures that our bounding box is properly sized.
      BBox3 buf_local_3d_bbox = m_bbox;
      buf_local_3d_bbox.min().x() = m_bbox.min().x() + (buffered_bbox.min().x() * m_spacing);
      buf_local_3d_bbox.min().y() = m_bbox.max().y() - (buffered_bbox.max().y() * m_spacing);
      buf_local_3d_bbox.max().x() = buf_local_3d_bbox.min().x() + (buffered_bbox.width() * m_spacing);
      buf_local_3d_bbox.max().y() = buf_local_3d_bbox.min().y() + (buffered_bbox.height() * m_spacing);
      BBox3 local_3d_bbox = m_bbox;
      local_3d_bbox.min().x() = m_bbox.min().x() + (bbox.min().x() * m_spacing);
      local_3d_bbox.min().y() = m_bbox.max().y() - (bbox.max().y() * m_spacing);
      local_3d_bbox.max().x() = local_3d_bbox.min().x() + (bbox.width() * m_spacing);
      local_3d_bbox.max().y() = local_3d_bbox.min().y() + (bbox.height() * m_spacing);

      ImageView<float> render_buffer(buffered_bbox.width(), buffered_bbox.height());
      float *render_buffer_ptr = &(render_buffer(0,0));

      // Setup a software renderer and the orthographic view matrix
      vw::stereo::SoftwareRenderer renderer(buffered_bbox.width(),
                                            buffered_bbox.height(),
                                            render_buffer_ptr);
      renderer.Ortho2D(buf_local_3d_bbox.min().x(), buf_local_3d_bbox.max().x(),
                       buf_local_3d_bbox.min().y(), buf_local_3d_bbox.max().y());

      // Set up the default color value
      if (m_use_alpha) {
        renderer.Clear(-32000);  // use this dummy value to denote transparency
      } else if (m_minz_as_default) {
        renderer.Clear(m_bbox.min().z());
      } else {
        renderer.Clear(m_default_value);
      }

      static const int NUM_COLOR_COMPONENTS = 1;  // We only need gray scale
      static const int NUM_VERTEX_COMPONENTS = 2; // DEMs are 2D

      float vertices[10], intensities[5];
      renderer.SetVertexPointer(NUM_VERTEX_COMPONENTS, vertices);
      renderer.SetColorPointer(NUM_COLOR_COMPONENTS, intensities);

      BOOST_FOREACH( BBoxPair const& boundary,
                     m_point_image_boundaries ) {
        if ( local_3d_bbox.intersects( boundary.first ) ) {
          // Pull a copy of the input image
          ImageView<Vector3> point_copy =
            crop(m_point_image, boundary.second );
          ImageView<float> texture_copy =
            crop(m_texture, BBox2i( boundary.second.min(),
                                    boundary.second.max()+Vector2i(1,1) ) );
          typedef typename ImageView<Vector3>::pixel_accessor PointAcc;
          PointAcc row_acc = point_copy.origin();
          for ( int32 row = 0; row < point_copy.rows()-1; ++row ) {
            PointAcc point_ul = row_acc;
            for ( int32 col = 0; col < point_copy.cols()-1; ++col ) {
              if ( *point_ul != Vector3() ) {

                PointAcc point_ur = point_ul; point_ur.next_col();
                PointAcc point_ll = point_ul; point_ll.next_row();
                PointAcc point_lr = point_ul; point_lr.advance(1,1);

                // Verify the point UL and LR are in the rasterable
                // area. Then just verify that at least one vertice is
                // placed in the viewable area.
                if ( *point_lr == Vector3() ||
                     !buf_local_3d_bbox.contains(*point_ul) ||
                     !buf_local_3d_bbox.contains(*point_lr) ||
                     !(local_3d_bbox.contains(*point_ul) ||
                       local_3d_bbox.contains(*point_ur) ||
                       local_3d_bbox.contains(*point_ll) ||
                       local_3d_bbox.contains(*point_lr) ) ) {
                  point_ul.next_col();
                  continue;
                }

                vertices[0] = (*point_ll).x(); // LL
                vertices[1] = (*point_ll).y();
                vertices[2] = (*point_lr).x(); // LR
                vertices[3] = (*point_lr).y();
                vertices[4] = (*point_ul).x(); // UL
                vertices[5] = (*point_ul).y();
                vertices[6] = (*point_lr).x(); // LR
                vertices[7] = (*point_lr).y();
                vertices[8] = (*point_ur).x(); // UR
                vertices[9] = (*point_ur).y();

                intensities[0] = texture_copy(col,  row+1);
                intensities[1] = texture_copy(col+1,row+1);
                intensities[2] = texture_copy(col,  row);
                intensities[3] = texture_copy(col+1,row+1);
                intensities[4] = texture_copy(col+1,row);

                // triangle 1 is: LL, LR, UL
                if ( *point_ll != Vector3() &&
                     buf_local_3d_bbox.contains(*point_lr) )
                  renderer.DrawPolygon(0, 3);

                // triangle 2 is: UL, LR, UR
                if ( *point_ur != Vector3() &&
                     buf_local_3d_bbox.contains(*point_ur) )
                  renderer.DrawPolygon(2, 3);
              }
              point_ul.next_col();
            }
            row_acc.next_row();
          }
        } // end cond
      }   // end foreach

      // The software renderer returns an image which will render
      // upside down in most image formats, so we correct that here.
      // We also introduce transparent pixels into the result where
      // necessary.
      ImageView<PixelT> result =
        flip_vertical(per_pixel_filter(render_buffer, RemoveSoftInvalid()));

      // This may seem confusing, but we must crop here so that the
      // good pixel data is placed into the coordinates specified by
      // the bbox.  This allows rasterize to touch those pixels
      // using the coordinates inside the bbox.  The pixels outside
      // those coordinates are invalid, but they never get accessed.
      return CropView<ImageView<pixel_type> > (result, BBox2i(-buffered_bbox.min().x(),
                                                              -buffered_bbox.min().y(),
                                                              cols(), rows()));
    }
    template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }
    /// \endcond

    void set_use_alpha(bool val) { m_use_alpha = val; }
    void set_use_minz_as_default(bool val) { m_minz_as_default = val; }
    void set_default_value(double val) { m_default_value = val; }
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
        BBox3 bbox = bounding_box();
        double bbox_width = fabs(bbox.max().x() - bbox.min().x());
        double bbox_height = fabs(bbox.max().y() - bbox.min().y());
        double input_image_width = m_point_image.cols();
        double input_image_height = m_point_image.rows();
        m_spacing = std::max(bbox_width, bbox_height) / std::max(input_image_width, input_image_height);
      } else {
        m_spacing = val;
      }
    }
    double spacing() { return m_spacing; }

    BBox3 bounding_box() { return m_bbox; }

    // Return the affine georeferencing transform.
    vw::Matrix<double,3,3> geo_transform() {
      vw::Matrix<double,3,3> geo_transform;
      geo_transform.set_identity();
      geo_transform(0,0) = m_spacing;
      geo_transform(1,1) = -1 * m_spacing;
      geo_transform(0,2) = m_bbox.min().x();
      geo_transform(1,2) = m_bbox.max().y();
      return geo_transform;
    }

  };

  template <class PixelT, class ImageT, class TextureT>
  OrthoRasterizerView<PixelT, ImageT>
  ortho_rasterizer( ImageViewBase<ImageT> const& point_cloud,
                    ImageViewBase<TextureT> const& texture, double spacing = 0.0,
                    const ProgressCallback& progress = ProgressCallback::dummy_instance()) {
    return OrthoRasterizerView<PixelT,ImageT>(point_cloud.impl(),texture.impl(),
                                              spacing, progress );
  }

}} // namespace vw::cartography
