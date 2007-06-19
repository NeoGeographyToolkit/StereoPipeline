
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/BlockCacheView.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

#include "SoftwareRenderer.h"
//#include "HardwareRenderer.h"

namespace vw {
namespace cartography {

  template <class PixelT>
  class OrthoRasterizerView : public ImageViewBase<OrthoRasterizerView<PixelT> > {
    ImageViewRef<Vector3> m_point_image;
    ImageViewRef<float> m_texture;
    BBox<float,3> m_bbox;
    float m_spacing;
    double m_default_value;
    bool m_minz_as_default;
    int* m_row_start;

  public:
    typedef PixelT pixel_type;
    typedef const PixelT result_type;
    typedef ProceduralPixelAccessor<OrthoRasterizerView> pixel_accessor;

    template <class PointViewT, class TextureViewT>
    OrthoRasterizerView(PointViewT point_cloud, TextureViewT texture, float spacing = 0.0) : 
      m_point_image(point_cloud), m_texture(channel_cast<float>(channels_to_planes(texture))), 
      m_default_value(0), m_minz_as_default(true) {

      set_texture(texture.impl());

      // Compute the bounding box that encompasses all of the
      // available points.
      vw_out(InfoMessage) << "Computing raster bounding box...\n";
      TerminalProgressCallback progress_callback;
      progress_callback.report_progress(0);

      for (unsigned int j = 0; j < m_point_image.rows(); j++) {
        progress_callback.report_progress(float(j)/m_point_image.rows());
        for (unsigned int i= 0; i < m_point_image.cols(); i++) 
          if (m_point_image(i,j) != Vector3())
            m_bbox.grow(m_point_image(i,j));
      }
      progress_callback.report_finished();
      this->set_spacing(spacing);

      m_row_start = new int(0);
    }

    // You can change the texture after
    template <class TextureViewT>
    void set_texture(TextureViewT texture) { 
      VW_ASSERT(texture.impl().cols() == m_point_image.cols() && texture.impl().rows() == m_point_image.rows(),
                ArgumentErr() << "Orthorasterizer: texture dimensions must match point image dimensions."); 
      m_texture = channel_cast<float>(channels_to_planes(texture.impl()));
    }

    inline int32 cols() const { return (int) (fabs(m_bbox.max().x() - m_bbox.min().x()) / m_spacing) + 1; }
    inline int32 rows() const { return (int) (fabs(m_bbox.max().y() - m_bbox.min().y()) / m_spacing) + 1; }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( int i, int j, int p=0 ) const { 
      vw_throw(NoImplErr() << "OrthoRasterizersView::operator()(double i, double j, int32 p) has not been implemented.");
      return pixel_type();
    }

    /// \cond INTERNAL
    typedef CropView<ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize( BBox2i const& bbox ) const { 

      // Buffer the bounding box slightly so that we render a few
      // extra triangles that would have fallen off the border
      // otherwise.
      BBox2i buffered_bbox = bbox;
      buffered_bbox.min() -= Vector2i(4,4);
      buffered_bbox.max() += Vector2i(4,4);

      // This ensures that our bounding box is properly sized.
      BBox<float,3> local_bbox = m_bbox;
      local_bbox.min().x() = m_bbox.min().x() + (buffered_bbox.min().x() * m_spacing);
      local_bbox.min().y() = m_bbox.max().y() - (buffered_bbox.max().y() * m_spacing);
      local_bbox.max().x() = local_bbox.min().x() + (buffered_bbox.width() * m_spacing);
      local_bbox.max().y() = local_bbox.min().y() + (buffered_bbox.height() * m_spacing);
                   
      ImageView<float> render_buffer(buffered_bbox.width(), buffered_bbox.height());
      float *render_buffer_ptr = &(render_buffer(0,0));
      
      const int numColorComponents = 1;	     // We only need gray scale
      const int numVertexComponents = 2;	   // DEMs are 2D

      // Setup a software renderer and the orthographic view matrix
      vw::stereo::SoftwareRenderer renderer = vw::stereo::SoftwareRenderer(buffered_bbox.width(), buffered_bbox.height(), render_buffer_ptr);
      //vw::stereo::HardwareRenderer renderer = vw::stereo::HardwareRenderer(buffered_bbox.width(), buffered_bbox.height(), render_buffer_ptr);
      renderer.Ortho2D(local_bbox.min().x(), local_bbox.max().x(), local_bbox.min().y(), local_bbox.max().y());

      // Set up the default color value
      if (m_minz_as_default) {
        renderer.ClearColor(m_bbox.min().z(), m_bbox.min().z(), m_bbox.min().z(), 1.0);        
        renderer.Clear(vw::stereo::eColorBufferBit);
      } else {
        renderer.ClearColor(m_default_value, m_default_value, m_default_value, 1.0);
        renderer.Clear(vw::stereo::eColorBufferBit);
      } 

      bool row_started = false;
      for (unsigned int row = *m_row_start; row < m_point_image.rows(); ++row) {
        bool row_finished = true;
        for (unsigned int col = 0; col < m_point_image.cols(); ++col) {
          if (local_bbox.contains(m_point_image(col,row)) ) {
            // If this is the first row with valid data, we record it
            // and start the search here next time.  However, this
            // optimization is only valid if we are rasterizing the
            // entire width of the image at once.
            if (row_started == false && bbox.width() == cols()) 
              *m_row_start = row;
            row_started = true;
            row_finished = false;

            float vertices[12], intensities[6];
            int triangle_count;
            
            // Create the two triangles covering this "pixel"
            this->create_triangles(row, col, triangle_count, vertices, intensities);
            renderer.SetVertexPointer(numVertexComponents, vw::stereo::ePackedArray, vertices);
            
            // Draw elevations into DEM buffer
            renderer.SetColorPointer(numColorComponents, vw::stereo::ePackedArray, intensities);
            for (int i = 0; i < triangle_count; i++)
              renderer.DrawPolygon(i * 3, 3);
          }
        }
        if (row_started && row_finished) break;
      }

      // The software renderer returns an image which will render
      // upside down in most image formats, so we correct that here as
      // well.
      ImageView<PixelT> result(render_buffer.cols(), render_buffer.rows());
      for (int j = 0; j < render_buffer.rows(); ++j) {
        for (int i = 0; i < render_buffer.cols(); ++i) {
          if (render_buffer(i,render_buffer.rows()-1-j) == 0) {
            result(i,j) = PixelT();
          } else {
            result(i,j) = PixelT(render_buffer(i,render_buffer.rows()-1-j));
          }
        }
      }

      // This may seem confusing, but we must crop here so that the
      // good pixel data is placed into the coordinates specified by
      // the bbox.  This allows rasterize to touch those pixels
      // using the coordinates inside the bbox.  The pixels outside
      // those coordinates are invalid, but they never get accessed.
      return CropView<ImageView<pixel_type> > (result, BBox2i(-(buffered_bbox.min().x()), 
                                                              -(buffered_bbox.min().y()), 
                                                              cols(), rows()));
    }
    template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
    /// \endcond

    void set_default_value(double val) { m_default_value = val; }
    double default_value() { 
      if (m_minz_as_default) return m_bbox.min().z();
      else return m_default_value; 
    }

    /// If no DEM spacing is set to zero, we generate a DEM with the same
    /// pixel dimensions as the input image.  Note, however, that this
    /// could lead to a loss in DEM resolution if the DEM is rotated
    /// from the orientation of the original image.
    void set_spacing(float val) {
      if (val == 0.0) {
        BBox<float,3> bbox = bounding_box();
        float bbox_width = fabs(bbox.max().x() - bbox.min().x());
        float bbox_height = fabs(bbox.max().y() - bbox.min().y());
        float input_image_width = m_point_image.cols();
        float input_image_height = m_point_image.rows();
        m_spacing = std::max(bbox_width, bbox_height) / 
                        std::max(input_image_width, input_image_height);
        std::cout << "\tAutomatically setting spacing to " << m_spacing << " units/pixel.\n";
      } else {
        m_spacing = val;
      }
    }
    double spacing() { return m_spacing; }

    BBox<float,3> bounding_box() { return m_bbox; }

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

    inline void create_triangles(int row, int col, 
                                int &triangleCount,  
                                float vertices[12],
                                float intensities[6]) const
    {
      triangleCount = 0;
      
      // triangle 1 is: LL, LR, UL
      if (m_point_image(col,row+1)   != Vector3() &&   // LL
          m_point_image(col+1,row+1) != Vector3() &&   // LR
          m_point_image(col,row)     != Vector3() )    // UL
        {
          vertices[0] = m_point_image(col, row+1).x();  // LL
          vertices[1] = m_point_image(col, row+1).y();
          vertices[2] = m_point_image(col+1,row+1).x(); // LR
          vertices[3] = m_point_image(col+1,row+1).y();
          vertices[4] = m_point_image(col,row).x();     // UL
          vertices[5] = m_point_image(col,row).y(); 

          intensities[0] = m_texture(col, row+1); // LL
          intensities[1] = m_texture(col+1,row+1);// LR
          intensities[2] = m_texture(col,row);    // UL
          
          triangleCount++;
        }
      
      // triangle 2 is: UL, LR, UR
      if (m_point_image(col,row)     != Vector3() &&   // UL
          m_point_image(col+1,row+1) != Vector3() &&   // LR
          m_point_image(col+1,row)   != Vector3() )    // UR
        {
          int vertex0 = triangleCount * 6;
          int elev0 = triangleCount * 3;
          
          vertices[vertex0] = m_point_image(col,row).x(); // UL
          vertices[vertex0 + 1] = m_point_image(col,row).y();
          vertices[vertex0 + 2] = m_point_image(col+1,row+1).x(); // LR
          vertices[vertex0 + 3] = m_point_image(col+1,row+1).y();
          vertices[vertex0 + 4] = m_point_image(col+1,row).x(); // UR
          vertices[vertex0 + 5] = m_point_image(col+1,row).y();

          intensities[elev0] = m_texture(col,row);
          intensities[elev0 + 1] = m_texture(col+1,row+1);
          intensities[elev0 + 2] = m_texture(col+1,row);
          
          triangleCount++;
        }
    }
  };

}} // namespace vw::cartography
