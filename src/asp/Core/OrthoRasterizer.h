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

#include <boost/foreach.hpp>
#include <boost/math/special_functions/next.hpp>

namespace vw {
namespace cartography {

  template <class PixelT, class ImageT>
  class OrthoRasterizerView : public ImageViewBase<OrthoRasterizerView<PixelT, ImageT> > {
    ImageT m_point_image;
    ImageViewRef<float> m_texture;
    BBox3 m_bbox;           // Bounding box of point cloud
    double m_spacing;       // pointcloud units (usually m or deg) per pxel
    double m_default_value;
    bool m_minz_as_default;
    bool m_use_alpha;
    int m_block_size;
    
    // We could actually use a quadtree here .. but this should be a
    // good enough improvement.
    typedef std::pair<BBox3, BBox2i> BBoxPair;
    std::vector<BBoxPair > m_point_image_boundaries;
    // These boundaries describe a point cloud 3D boundaries and then
    // their location in the the point cloud image. These boxes are
    // overlapping in the pc image X/Y domain to insure that
    // everything is triangulated.

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
      int m_sub_block_size;
      BBox2i m_image_bbox;
      BBox3& m_global_bbox;
      std::vector<BBoxPair>& m_point_image_boundaries;
      Mutex& m_mutex;
      const ProgressCallback& m_progress;
      float m_inc_amt;

      // This is growing a bbox of points in point projection and Z
      // values which are altitude.
      struct GrowBBoxAccumulator {
        BBox3 bbox;
        void operator()( Vector3 const& v ) {
          if ( !boost::math::isnan(v.z()) )
            bbox.grow(v);
        }
      };

    public:
      SubBlockBoundaryTask( ImageViewBase<ViewT> const& view,
                            int sub_block_size,
                            BBox2i const& image_bbox,
                            BBox3& global_bbox, std::vector<BBoxPair>& boundaries,
                            Mutex& mutex, const ProgressCallback& progress, float inc_amt ) :
        m_view(view.impl()), m_sub_block_size(sub_block_size),
        m_image_bbox(image_bbox),
        m_global_bbox(global_bbox), m_point_image_boundaries( boundaries ),
        m_mutex( mutex ), m_progress( progress ), m_inc_amt( inc_amt ) {}
      void operator()() {
        ImageView< typename ViewT::pixel_type > local_copy =
          crop( m_view, m_image_bbox );

        // Further subdivide into boundaries so
        // that prerasterize will only query what it needs.
        std::vector<BBox2i> blocks =
          image_blocks( m_image_bbox, m_sub_block_size, m_sub_block_size );
        BBox3 local_union;
        std::list<BBoxPair> solutions;
        for ( size_t i = 0; i < blocks.size(); i++ ) {
          GrowBBoxAccumulator accum;
          for_each_pixel( crop( local_copy, blocks[i] - m_image_bbox.min() ),
                          accum );
          if ( !accum.bbox.empty() ) {
            accum.bbox.max()[0] = boost::math::float_next(accum.bbox.max()[0]);
            accum.bbox.max()[1] = boost::math::float_next(accum.bbox.max()[1]);
            local_union.grow( accum.bbox );
            solutions.push_back( std::make_pair( accum.bbox, blocks[i] ) );
          }
        }

        // Append to the global list of boxes and expand the point
        // cloud bounding box.
        if ( local_union != BBox3() ) {
          Mutex::Lock lock( m_mutex );
          for ( std::list<BBoxPair>::const_iterator it = solutions.begin();
                it != solutions.end(); it++ ) {
            m_point_image_boundaries.push_back( *it );
          }
          m_global_bbox.grow( local_union );
          m_progress.report_incremental_progress( m_inc_amt );
        }
      }
    };

  public:
    typedef PixelT pixel_type;
    typedef const PixelT result_type;
    typedef ProceduralPixelAccessor<OrthoRasterizerView> pixel_accessor;

    // Estimated size of each point cloud pixel in geodetic units
    double m_pix2point;
    
    template <class TextureViewT>
    OrthoRasterizerView(ImageT point_image, TextureViewT texture, double spacing = 0.0,
                        const ProgressCallback& progress = ProgressCallback::dummy_instance()) :
      m_point_image(point_image), m_texture(ImageView<float>(1,1)), // dummy value
      m_default_value(0), m_minz_as_default(true), m_use_alpha(false),
      m_block_size(256) /* To do: query block size from input point cloud */ {

      set_texture(texture.impl());

      // Compute the bounding box that encompasses tiles within the image
      //
      // They're used for querying what part of the image we need
      VW_OUT(DebugMessage,"asp") << "Computing raster bounding box...\n";

      // Subdivide each block into smaller chunks. Note: small chunks
      // greatly increase the memory usage and run-time for very large
      // images (because they are very many). As such, make the chunks
      // bigger for bigger images.
      double s = 10000.0; 
      int sub_block_size
        = int(double(point_image.cols())*double(point_image.rows())/(s*s));
      sub_block_size = std::max(1, sub_block_size);
      sub_block_size = int(round(pow(2.0, floor(log(sub_block_size)/log(2.0)))));
      sub_block_size = std::max(16, sub_block_size);
      sub_block_size = std::min(128, sub_block_size);
      std::vector<BBox2i> blocks =
        image_blocks( m_point_image, m_block_size, m_block_size );

      FifoWorkQueue queue( vw_settings().default_num_threads() );
      typedef SubBlockBoundaryTask<ImageT> task_type;
      Mutex mutex;
      float inc_amt = 1.0 / float(blocks.size());
      for ( size_t i = 0; i < blocks.size(); i++ ) {
        boost::shared_ptr<task_type>
          task( new task_type( m_point_image, sub_block_size, blocks[i],
                               m_bbox, m_point_image_boundaries, mutex,
                               progress, inc_amt ) );
        queue.add_task( task );
      }
      queue.join_all();
      progress.report_finished();

      if ( m_bbox.empty() )
        vw_throw( ArgumentErr() << "OrthoRasterize: Input point cloud is empty!\n" );

      VW_OUT(DebugMessage,"asp") << "Point cloud boundary is " << m_bbox << "\n";

      // Set the sampling rate (i.e. spacing between pixels)
      this->set_spacing(spacing);
      VW_OUT(DebugMessage,"asp") << "Pixel spacing is " << m_spacing << " pnt/px\n";

      // Estimate the size of each point cloud pixel
      int len = m_point_image_boundaries.size();
      std::vector<double> vx, vy;
      vx.reserve(len); vx.clear();
      vy.reserve(len); vy.clear();
      BOOST_FOREACH( BBoxPair const& boundary,
                     m_point_image_boundaries ) {
        if (boundary.first.empty()) continue;
        vx.push_back(boundary.first.max().x() - boundary.first.min().x());
        vy.push_back(boundary.first.max().y() - boundary.first.min().y());
      }
      std::sort(vx.begin(), vx.end());
      std::sort(vy.begin(), vy.end());
      m_pix2point = 0;
      if (len > 0){
        // Get the median
        m_pix2point = (vx[(int)(0.5*len)] + vy[(int)(0.5*len)])/2.0;
      }
      // Divide by the subblock size
      m_pix2point /= double(sub_block_size); 
      
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

      BBox2i bbox_1 = bbox;
      bbox_1.expand(5); // bugfix, ensure we see enough beyond current tile
      
      // Used to find which polygons are actually in the draw space.
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

      std::valarray<float> vertices(10), intensities(5);
      renderer.SetVertexPointer(NUM_VERTEX_COMPONENTS, &vertices[0]);
      renderer.SetColorPointer(NUM_COLOR_COMPONENTS, &intensities[0]);

      // From the box in the DEM pixel space, get the box in the point
      // cloud pixel space.
      BBox2i point_image_boundary;
      std::vector<double> cx, cy; // box centers in the point cloud pixel space
      BOOST_FOREACH( BBoxPair const& boundary,
                     m_point_image_boundaries ) {
        if (! local_3d_bbox.intersects(boundary.first) ) continue;
        point_image_boundary.grow( boundary.second );
        cx.push_back((boundary.second.min().x()+boundary.second.max().x())/2.0);
        cy.push_back((boundary.second.min().y()+boundary.second.max().y())/2.0);
      }

      // In some cases, the memory usage blows up. Then, roughly
      // estimate how much of the input point cloud region we need to
      // see for the given DEM tile, and multiply that region by a
      // factor of FACTOR. Place that region at the median of the
      // centers of the boxes used to grow the image boundary earlier,
      // and intersect that with the current box. This will not kick
      // in unless point_image_boundary estimated above is grossly
      // larger than what it should be.
      int FACTOR = 4;
      Vector3 estim = (local_3d_bbox.max() - local_3d_bbox.min())/m_pix2point;
      int max_len = (int)ceil(FACTOR*std::max(estim[0], estim[1]));
      if (!cx.empty() &&
          (point_image_boundary.width()  > max_len ||
           point_image_boundary.height() > max_len)
          ){
        std::sort(cx.begin(), cx.end());
        std::sort(cy.begin(), cy.end());
        int midx = (int)round(cx[cx.size()/2]);
        int midy = (int)round(cy[cy.size()/2]);
        BBox2i smaller_boundary(midx - max_len/2, midy - max_len/2,
                                max_len, max_len );
        smaller_boundary.expand(1);
        point_image_boundary.crop(smaller_boundary);
      }
      
      if ( point_image_boundary.empty() )
        return CropView<ImageView<pixel_type> >( render_buffer,
                                                 BBox2i(-bbox_1.min().x(),
                                                        -bbox_1.min().y(),
                                                        cols(), rows()) );

      // Bugfix, ensure we see enough beyond current tile
      point_image_boundary.expand(5);
      point_image_boundary.crop(vw::bounding_box(m_point_image));

      // If point_image_boundary is big, subdivide it into blocks
      // to save on memory.
      int max_tile_size = 512; 
      std::vector<BBox2i> blocks;
      if (point_image_boundary.width()  >= 2*max_tile_size ||
          point_image_boundary.height() >= 2*max_tile_size
          ){
        BBox2i tile_box;
        tile_box.min()
          = m_block_size*floor(point_image_boundary.min()/double(m_block_size));
        tile_box.max()
          = m_block_size*ceil(point_image_boundary.max()/double(m_block_size));
        blocks =
          image_blocks( tile_box, max_tile_size, max_tile_size);
      }else{
        blocks.push_back(point_image_boundary);
      }
      
      for (int i = 0; i < (int)blocks.size(); i++){

        // Need to grow the block, as for rendering below we
        // need each pixel and its neighbors
        blocks[i].max() += Vector2i(1, 1);
        blocks[i].crop(point_image_boundary);
        
        // Pull a copy of the input image in memory
        ImageView<typename ImageT::pixel_type> point_copy =
          crop(m_point_image, blocks[i] );
        ImageView<float> texture_copy =
          crop(m_texture, blocks[i] );
        typedef typename ImageView<Vector3>::pixel_accessor PointAcc;
        PointAcc row_acc = point_copy.origin();
        for ( int32 row = 0; row < point_copy.rows()-1; ++row ) {
          PointAcc point_ul = row_acc;
          
          for ( int32 col = 0; col < point_copy.cols()-1; ++col ) {
            PointAcc point_ur = point_ul; point_ur.next_col();
            PointAcc point_ll = point_ul; point_ll.next_row();
            PointAcc point_lr = point_ul; point_lr.advance(1,1);
            
            // This loop rasterizes a quad indexed by the upper left.
            if ( !boost::math::isnan((*point_ul).z()) &&
                 !boost::math::isnan((*point_lr).z()) ) {
              
              vertices[0] = (*point_ul).x(); // UL
              vertices[1] = (*point_ul).y();
              vertices[2] = (*point_ll).x(); // LL
              vertices[3] = (*point_ll).y();
              vertices[4] = (*point_lr).x(); // LR
              vertices[5] = (*point_lr).y();
              vertices[6] = (*point_ur).x(); // UR
              vertices[7] = (*point_ur).y();
              vertices[8] = (*point_ul).x(); // UL
              vertices[9] = (*point_ul).y();
              
              intensities[0] = texture_copy(col,  row);
              intensities[1] = texture_copy(col,row+1);
              intensities[2] = texture_copy(col+1,  row+1);
              intensities[3] = texture_copy(col+1,row);
              intensities[4] = texture_copy(col,row);
              
              if ( !boost::math::isnan((*point_ll).z()) ) {
                // triangle 1 is: UL LL LR
                renderer.DrawPolygon(0, 3);
              }
              if ( !boost::math::isnan((*point_ur).z()) ) {
                // triangle 2 is: LR, UR, UL
                renderer.DrawPolygon(2, 3);
              }
            }
            point_ul.next_col();
          }
          row_acc.next_row();
        }
        
      }
      
      // The software renderer returns an image which will render
      // upside down in most image formats, so we correct that here.
      // We also introduce transparent pixels into the result where
      // necessary.
      ImageView<PixelT> result = flip_vertical(render_buffer);

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
  ortho_rasterizer( ImageViewBase<ImageT> const& point_image,
                    ImageViewBase<TextureT> const& texture, double spacing = 0.0,
                    const ProgressCallback& progress = ProgressCallback::dummy_instance()) {
    return OrthoRasterizerView<PixelT,ImageT>(point_image.impl(),texture.impl(),
                                              spacing, progress );
  }

}} // namespace vw::cartography
