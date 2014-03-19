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

#include <asp/Core/Point2Grid.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/next.hpp>

namespace vw { namespace cartography {

  template <class PixelT, class ImageT>
  class OrthoRasterizerView : public ImageViewBase<OrthoRasterizerView<PixelT, ImageT> > {
    ImageT m_point_image;
    ImageViewRef<float> m_texture;
    BBox3 m_bbox;             // bounding box of point cloud
    double m_spacing;         // point cloud units (usually m or deg) per pixel
    double m_default_spacing; // if user did not specify spacing
    double m_default_spacing_x;
    double m_default_spacing_y;
    double m_default_value;
    bool m_minz_as_default;
    bool m_use_alpha;
    int m_block_size;
    double m_search_radius_factor;
    bool m_use_surface_sampling;
    
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
#if 1
      int d = (int)m_use_surface_sampling;
      output.min().x() = m_bbox.min().x() + ((double(px.min().x() - d)) * m_spacing);
      output.max().x() = m_bbox.min().x() + ((double(px.max().x() - d)) * m_spacing);
      output.min().y() = m_bbox.min().y() + ((double(rows() - px.max().y() - d)) * m_spacing);
      output.max().y() = m_bbox.min().y() + ((double(rows() - px.min().y() - d)) * m_spacing);

#else
      // Old buggy temporary code
      output.min().x() = m_bbox.min().x() + ((double(px.min().x()) - 0.5 ) * m_spacing);
      output.max().x() = boost::math::float_next(m_bbox.min().x() + ((double(px.max().x()) - 1 + 0.5) * m_spacing));
      output.min().y() = m_bbox.min().y() + ((double(rows() - px.max().y() + 1) - 0.5) * m_spacing);
      output.max().y() = boost::math::float_next(m_bbox.min().y() + ((double(rows() - px.min().y()) + 0.5) * m_spacing));
#endif
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
            // Note: for local_union, which will end up contributing
            // to the global bounding box, we don't use the float_next
            // gimmick, as we need the precise box. That is useful
            // though for the individual boxes, to better do
            // intersections later.
            local_union.grow( accum.bbox );
            accum.bbox.max()[0] = boost::math::float_next(accum.bbox.max()[0]);
            accum.bbox.max()[1] = boost::math::float_next(accum.bbox.max()[1]);
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

    template <class TextureViewT>
    OrthoRasterizerView(ImageT point_image, TextureViewT texture, double spacing,
                        double search_radius_factor, bool use_surface_sampling,
                        const ProgressCallback& progress
                        = ProgressCallback::dummy_instance()) :
      m_point_image(point_image), m_texture(ImageView<float>(1,1)), // dummy temp value
      m_default_value(0), m_minz_as_default(true), m_use_alpha(false),
      m_block_size(256), /* To do: query block size from input point cloud */
      m_search_radius_factor(search_radius_factor),
      m_use_surface_sampling(use_surface_sampling){

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
        vw_throw( ArgumentErr() <<
                  "OrthoRasterize: Input point cloud is empty!\n" );
      VW_OUT(DebugMessage,"asp") << "Point cloud boundary is " << m_bbox << "\n";
      
      // Find the width and height of the median point cloud
      // pixel in projected coordinates.
      int len = m_point_image_boundaries.size();
      {
        // This vectors can be large, so don't keep them for too long
        std::vector<double> vx, vy;
        vx.reserve(len); vx.clear();
        vy.reserve(len); vy.clear();
        BOOST_FOREACH( BBoxPair const& boundary, m_point_image_boundaries ) {
          if (boundary.first.empty()) continue;
          vx.push_back(boundary.first.width() /sub_block_size);
          vy.push_back(boundary.first.height()/sub_block_size);
        }
        std::sort(vx.begin(), vx.end());
        std::sort(vy.begin(), vy.end());
        m_default_spacing_x = m_default_spacing_y = 0.0;
        if (len > 0){
          // Get the median
          m_default_spacing_x = vx[(int)(0.5*len)];
          m_default_spacing_y = vy[(int)(0.5*len)];
        }
      }

      // This must happen after the bounding box was computed, but
      // before setting the spacing.
      if (m_use_surface_sampling){
        // Old way
        BBox3 bbox = bounding_box();
        double bbox_width = fabs(bbox.max().x() - bbox.min().x());
        double bbox_height = fabs(bbox.max().y() - bbox.min().y());
        double input_image_width = m_point_image.cols();
        double input_image_height = m_point_image.rows();
        // The formula below is not so good, its output depends strongly
        // on how many rows and columns are in the point cloud.
        m_default_spacing
          = std::max(bbox_width, bbox_height) / std::max(input_image_width, input_image_height);
      }else{
        // We choose the coarsest of the two spacings
        m_default_spacing = std::max(m_default_spacing_x, m_default_spacing_y);
      }
      
      // Set the sampling rate (i.e. spacing between pixels)
      this->set_spacing(spacing);
      VW_OUT(DebugMessage,"asp") << "Pixel spacing is " << m_spacing << " pnt/px\n";
      return;
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

    inline int32 cols() const { return (int) round((fabs(m_bbox.max().x() - m_bbox.min().x()) / m_spacing)) + 1; }
    inline int32 rows() const { return (int) round((fabs(m_bbox.max().y() - m_bbox.min().y()) / m_spacing)) + 1; }
    
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
      BBox3 local_3d_bbox = pixel_to_point_bbox(bbox_1);

      ImageView<float> render_buffer;
      ImageView<double> d_buffer, weights;
      if (m_use_surface_sampling){
        render_buffer.set_size(bbox_1.width(), bbox_1.height());
      }
      
      // Setup a software renderer and the orthographic view matrix
      vw::stereo::SoftwareRenderer renderer(bbox_1.width(),
                                            bbox_1.height(),
                                            &render_buffer(0,0) );
      renderer.Ortho2D(local_3d_bbox.min().x(), local_3d_bbox.max().x(),
                       local_3d_bbox.min().y(), local_3d_bbox.max().y());

      // Given a DEM grid point, search for cloud points within the
      // circular region of radius equal to grid size. As such, a
      // given cloud point may contribute to multiple DEM points, but
      // with different weights (set by Gaussian). We make this radius
      // no smaller than the default DEM spacing. Search radius can be
      // over-ridden by user.
      double search_radius;
      if (m_search_radius_factor <= 0.0)
        search_radius = std::max(m_spacing, m_default_spacing);
      else
        search_radius = m_spacing*m_search_radius_factor;
      vw::stereo::Point2Grid point2grid(bbox_1.width(),
                                        bbox_1.height(),
                                        d_buffer, weights,
                                        local_3d_bbox.min().x(),
                                        local_3d_bbox.min().y(),
                                        m_spacing, search_radius);
      
      // Set up the default color value
      double min_val = 0.0;
      if (m_use_alpha) {
        // use this dummy value to denote transparency
        min_val = std::numeric_limits<float>::min();
      } else if (m_minz_as_default) {
        min_val = m_bbox.min().z();
      } else {
        min_val = m_default_value;
      }
      
      std::valarray<float> vertices(10), intensities(5);

      if (m_use_surface_sampling){
        static const int NUM_COLOR_COMPONENTS = 1;  // We only need gray scale
        static const int NUM_VERTEX_COMPONENTS = 2; // DEMs are 2D
        renderer.Clear(min_val);
        renderer.SetVertexPointer(NUM_VERTEX_COMPONENTS, &vertices[0]);
        renderer.SetColorPointer(NUM_COLOR_COMPONENTS, &intensities[0]);
      }else{
        point2grid.Clear(min_val);
      }
      
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

      // In some cases, the memory usage blows up due to noisy points
      // in the cloud. Then, roughly estimate how much of the input
      // point cloud region we need to see for the given DEM tile, and
      // multiply that region by a factor of FACTOR. Place that region
      // at the median of the centers of the boxes used to grow the
      // image boundary earlier, and intersect that with the current
      // box. This will not kick in unless point_image_boundary
      // estimated above is grossly larger than what it should be.
      int FACTOR = 3;
      double min_spacing = std::min(m_default_spacing_x, m_default_spacing_y);
      Vector2 estim = FACTOR*Vector2(local_3d_bbox.width()/min_spacing,
                                     local_3d_bbox.height()/min_spacing);
      if (!cx.empty() &&
          (point_image_boundary.width()  > estim[0] ||
           point_image_boundary.height() > estim[1])
          ){
        std::sort(cx.begin(), cx.end());
        std::sort(cy.begin(), cy.end());
        int midx = (int)round(cx[cx.size()/2]);
        int midy = (int)round(cy[cy.size()/2]);
        BBox2i smaller_boundary(midx - estim[0]/2, midy - estim[1]/2,
                                estim[0], estim[1] );
        smaller_boundary.expand(1); // to compensate for casting to int above
        point_image_boundary.crop(smaller_boundary);
      }
      
      if ( point_image_boundary.empty() ){
        if (m_use_surface_sampling){
          return CropView<ImageView<pixel_type> >( render_buffer,
                                                   BBox2i(-bbox_1.min().x(),
                                                          -bbox_1.min().y(),
                                                          cols(), rows()) );
        }else{
          return CropView<ImageView<pixel_type> >( d_buffer,
                                                   BBox2i(-bbox_1.min().x(),
                                                          -bbox_1.min().y(),
                                                          cols(), rows()) );
        }
      }

      int bias = 5; // This minimum bias is a bugfix, to see enough data
      if (!m_use_surface_sampling){
        // Bias to ensure we see enough points to average
        bias = std::max(bias, (int)ceil(2.0*search_radius/min_spacing));
      }
      point_image_boundary.expand(bias);
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

      // This is very important. When doing surface sampling, for each
      // pixel we need to see its next up and right neighbors.
      int d = (int)m_use_surface_sampling;
      
      for (int i = 0; i < (int)blocks.size(); i++){

        blocks[i].max() += Vector2i(d, d);
        blocks[i].crop(point_image_boundary);
        
        // Pull a copy of the input image in memory
        ImageView<typename ImageT::pixel_type> point_copy =
          crop(m_point_image, blocks[i] );

        ImageView<float> texture_copy =
          crop(m_texture, blocks[i] );
        typedef typename ImageView<Vector3>::pixel_accessor PointAcc;
        PointAcc row_acc = point_copy.origin();
        for ( int32 row = 0; row < point_copy.rows()-d; ++row ) {
          PointAcc point_ul = row_acc;
          
          for ( int32 col = 0; col < point_copy.cols()-d; ++col ) {
            
            PointAcc point_ur = point_ul; point_ur.next_col();
            PointAcc point_ll = point_ul; point_ll.next_row();
            PointAcc point_lr = point_ul; point_lr.advance(1,1);

            if (m_use_surface_sampling){

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
              
            }else{
              // The new engine
              if ( !boost::math::isnan(point_copy(col, row).z()) ){
                point2grid.AddPoint(point_copy(col, row).x(),
                                    point_copy(col, row).y(),
                                    texture_copy(col,  row));
              }
            }
            point_ul.next_col();
          }
          row_acc.next_row();
        }
        
      }

      if (!m_use_surface_sampling)
        point2grid.normalize();

      // The software renderer returns an image which will render
      // upside down in most image formats, so we correct that here.
      // We also introduce transparent pixels into the result where
      // necessary.
      // To do: Here can do flipping in place.
      ImageView<PixelT> result;
      if (m_use_surface_sampling)
        result = flip_vertical(render_buffer);
      else
        result = flip_vertical(d_buffer);
      
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
        m_spacing = m_default_spacing;
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

    void find_bdbox_robust_to_outliers(std::vector<BBoxPair >
                                       const& point_image_boundaries,
                                       BBox3 & bbox
                                       ){

      // To do: This code is not enabled yet.
      
      using namespace vw::math;

      double pct = 0.1; 
      double outlier_factor = 1.5;
      double ctrx_min, ctrx_max, ctry_min, ctry_max;
      double widx_min, widx_max, widy_min, widy_max;

      int len = point_image_boundaries.size();
      std::vector<double> vx, vy;

      // Find the reasonable box widths
      vx.reserve(len); vx.clear();
      vy.reserve(len); vy.clear();
      BOOST_FOREACH( BBoxPair const& boundary, point_image_boundaries ) {
        if (boundary.first.empty()) continue;
        vx.push_back(boundary.first.width());
        vy.push_back(boundary.first.height());
      }
      find_outlier_brackets(vx, pct, outlier_factor, widx_min, widx_max);
      find_outlier_brackets(vy, pct, outlier_factor, widy_min, widy_max);

      // Find the reasonable box centers. Note: We reuse the same
      // vectors vx and vy to save on memory, as they can be very
      // large.
      vx.reserve(len); vx.clear();
      vy.reserve(len); vy.clear();
      BOOST_FOREACH( BBoxPair const& boundary, point_image_boundaries ) {
        if (boundary.first.empty()) continue;
        vx.push_back(boundary.first.center().x());
        vy.push_back(boundary.first.center().y());
      }
      find_outlier_brackets(vx, pct, outlier_factor, ctrx_min, ctrx_max);
      find_outlier_brackets(vy, pct, outlier_factor, ctry_min, ctry_max);

      // Redo the bounding box computation, excluding outliers
      bbox = BBox3();
      BOOST_FOREACH( BBoxPair const& boundary, point_image_boundaries ) {
        BBox3 b = boundary.first;
        if (b.empty()) continue;
        if ( b.width()  < widx_min || b.width()  > widx_max ) continue;
        if ( b.height() < widy_min || b.height() > widy_max ) continue;
        if ( b.center().x()  < ctrx_min || b.center().x() > ctrx_max ) continue;
        if ( b.center().y()  < ctry_min || b.center().y() > ctry_max ) continue;
        bbox.grow(b);
      }
      
    }
    
  };
  
}} // namespace vw::cartography
