// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
    BBox3 m_bbox;           // Bounding box of point cloud
    double m_spacing;       // pointcloud units (usually m or deg) per pxel
    double m_point_spacing; // pointcloud samples per pointcloud units
    double m_default_value;
    bool m_minz_as_default;
    bool m_use_alpha;

    // We could actually use a quadtree here .. but this should be a
    // good enough improvement.
    typedef std::pair<BBox3, BBox2i> BBoxPair;
    std::vector<BBoxPair > m_point_image_boundaries;
    // These boundaries describe a point cloud 3D boundaries and then
    // their location in the the point cloud image. These boxes are
    // overlapping in the pc image X/Y domain to insure that
    // everything is triangulated.

    // This is growing a bbox of points in point projection and Z
    // values which are altitude.
    struct GrowBBoxAccumulator {
      BBox3 bbox;
      void operator()( Vector3 const& v ) {
        if ( !boost::math::isnan(v.z()) )
          bbox.grow(v);
      }
    };

    struct RemoveSoftInvalid : ReturnFixedType<PixelT> {
      template <class T>
      PixelT operator()( T const& v ) const {
        if ( v == -32000 )
          return PixelT();
        return v;
      }
    };

    // Function to convert pixel coordinates to the point domain
    BBox3 pixel_to_point_bbox( BBox2 const& px ) const {
      BBox3 output = m_bbox;
      output.min().x() = m_bbox.min().x() + ((double(px.min().x()) - 0.5 ) * m_spacing);
      output.max().x() = boost::math::float_next(m_bbox.min().x() + ((double(px.max().x()) - 1 + 0.5) * m_spacing));
      output.min().y() = m_bbox.min().y() + ((double(rows() - px.max().y() + 1) - 0.5) * m_spacing);
      output.max().y() = boost::math::float_next(m_bbox.min().y() + ((double(rows() - px.min().y()) + 0.5) * m_spacing));
      return output;
    }

    // Task to parallelize the generation of bounding boxes
    template <class ViewT>
    class SubBlockBoundaryTask : public Task, private boost::noncopyable {
      ViewT m_view;
      BBox2i m_image_bbox;
      BBox3& m_global_bbox;
      std::vector<BBoxPair>& m_point_image_boundaries;
      Mutex& m_mutex;
    public:
      SubBlockBoundaryTask( ImageViewBase<ViewT> const& view, BBox2i const& image_bbox,
                            BBox3& global_bbox, std::vector<BBoxPair>& boundaries,
                            Mutex& mutex ) : m_view(view.impl()), m_image_bbox(image_bbox),
                                             m_global_bbox(global_bbox), m_point_image_boundaries( boundaries ),
                                             m_mutex( mutex ) {}
      void operator()() {
        GrowBBoxAccumulator accum;
        ImageView< typename ViewT::pixel_type > local_copy =
          crop( m_view, m_image_bbox );
        for_each_pixel( local_copy, accum );
        if ( !accum.bbox.empty() ) {
          accum.bbox.max()[0] = boost::math::float_next(accum.bbox.max()[0]);
          accum.bbox.max()[1] = boost::math::float_next(accum.bbox.max()[1]);
          {
            Mutex::Lock lock( m_mutex );
            m_point_image_boundaries.push_back( std::make_pair( accum.bbox, m_image_bbox ) );
            m_global_bbox.grow( accum.bbox );
          }
        }
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
      VW_OUT(DebugMessage,"asp") << "Computing raster bounding box...\n";

      static const int32 BBOX_SPACING = 256;
      int32 x_divisions = (m_point_image.cols() - 1)/BBOX_SPACING + 1;
      int32 y_divisions = (m_point_image.rows() - 1)/BBOX_SPACING + 1;
      BBox2i pc_image_bbox(0,0,m_point_image.cols(),m_point_image.rows());

      FifoWorkQueue queue( vw_settings().default_num_threads() );
      Mutex mutex;
      typedef SubBlockBoundaryTask<ImageT> task_type;
      for ( int32 yi = 0; yi < y_divisions; ++yi ) {
        progress.report_fractional_progress(yi,y_divisions);
        for ( int32 xi = 0; xi < x_divisions; ++xi ) {
          BBox2i local_spot( xi * BBOX_SPACING, yi * BBOX_SPACING,
                             BBOX_SPACING, BBOX_SPACING );
          local_spot.expand(1);
          local_spot.max() += Vector2i(1,1);
          local_spot.crop( pc_image_bbox );

          boost::shared_ptr<task_type> task( new task_type( m_point_image, local_spot,
                                                            m_bbox, m_point_image_boundaries, mutex ) );
          queue.add_task( task );
        }
        queue.join_all(); // We do this a little early just so there's a progress bar.
      }
      progress.report_finished();

      if ( m_bbox.empty() )
        vw_throw( ArgumentErr() << "OrthoRasterize: Input point cloud is empty!\n" );

      m_point_spacing =
        double(std::min( m_point_image.rows(), m_point_image.cols() )) /
        max( subvector(m_bbox.size(),0,2) );
      VW_OUT(DebugMessage,"asp") << "Point cloud boundary is " << m_bbox << "\n";
      VW_OUT(DebugMessage,"asp") << "Point spacing is " << m_point_spacing << " samples/pnt\n";

      // Set the sampling rate (i.e. spacing between pixels)
      this->set_spacing(spacing);
      VW_OUT(DebugMessage,"asp") << "Pixel spacing is " << m_spacing << " pnt/px\n";
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

      // Used to find which polygons are actually in the draw space.
      BBox2i bbox_1 = bbox;
      bbox_1.expand(1);
      BBox3 local_3d_bbox     = pixel_to_point_bbox(bbox_1);

      ImageView<float> render_buffer(bbox_1.width(), bbox_1.height());

      // Setup a software renderer and the orthographic view matrix
      vw::stereo::SoftwareRenderer renderer(bbox_1.width(),
                                            bbox_1.height(),
                                            &render_buffer(0,0) );
      renderer.Ortho2D(local_3d_bbox.min().x(), local_3d_bbox.max().x(),
                       local_3d_bbox.min().y(), local_3d_bbox.max().y());

      // Set up the default color value
      if (m_use_alpha) {
        renderer.Clear(std::numeric_limits<float>::min());  // use this dummy value to denote transparency
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
            crop(m_texture, boundary.second );
          typedef typename ImageView<Vector3>::pixel_accessor PointAcc;
          PointAcc row_acc = point_copy.origin();
          for ( int32 row = 0; row < point_copy.rows()-1; ++row ) {
            PointAcc point_ul = row_acc;
            for ( int32 col = 0; col < point_copy.cols()-1; ++col ) {
              // This loop rasterizes a quad indexed by the upper left.
              if ( !boost::math::isnan((*point_ul).z()) ) {
                PointAcc point_ur = point_ul; point_ur.next_col();
                PointAcc point_ll = point_ul; point_ll.next_row();
                PointAcc point_lr = point_ul; point_lr.advance(1,1);

                // Verify that at least one vertice is
                // placed in the viewable area.
                if ( boost::math::isnan((*point_lr).z()) ||
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
                if ( !boost::math::isnan((*point_ll).z()) ) {
                  renderer.DrawPolygon(0, 3);
                }

                // triangle 2 is: UL, LR, UR
                if ( !boost::math::isnan((*point_ur).z()) ) {
                  renderer.DrawPolygon(2, 3);
                }
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
      // those coordinates are invalid, but they never get accessed
      return CropView<ImageView<pixel_type> > (result, BBox2i(-bbox_1.min().x(),
                                                              -bbox_1.min().y(),
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
      geo_transform(1,1) = -m_spacing;
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
