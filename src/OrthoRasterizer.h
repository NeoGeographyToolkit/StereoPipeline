
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <vw/Stereo/SoftwareRenderer.h>

namespace vw {
namespace cartography {

  template <class PositionT>
  class OrthoRasterizer {
    
    ImageViewRef<PositionT> m_point_cloud;
    float m_dem_spacing;
    BBox<float,3> m_bbox;
    double m_default_value;
    
  public:
    bool use_minz_as_default;

    template <class ViewT>
    OrthoRasterizer(ImageViewBase<ViewT> &point_cloud, float dem_spacing = 0.0) : 
      m_point_cloud(point_cloud.impl()), m_default_value(0), use_minz_as_default(true) {      

      for (unsigned int j = 0; j < m_point_cloud.rows(); j++) 
        for (unsigned int i= 0; i < m_point_cloud.cols(); i++) 
          if (m_point_cloud(i,j) != PositionT())   // If the position is not missing
            m_bbox.grow(m_point_cloud(i,j));

      this->set_dem_spacing(dem_spacing);
    }

    void set_default_value(double val) { m_default_value = val; }

    double default_value() { 
      if (use_minz_as_default) 
        return m_bbox.min().z();
      else 
        return m_default_value; 
    }

    /// If no DEM spacing is set to zero, we generate a DEM with the same
    /// pixel dimensions as the input image.  Note, however, that this
    /// could lead to a loss in DEM resolution if the DEM is rotated
    /// from the orientation of the original image.
    void set_dem_spacing(float val) {
      if (val == 0.0) {
        BBox<float,3> bbox = bounding_box();
        float bbox_width = fabs(bbox.max().x() - bbox.min().x());
        float bbox_height = fabs(bbox.max().y() - bbox.min().y());
        float input_image_width = m_point_cloud.cols();
        float input_image_height = m_point_cloud.rows();
        m_dem_spacing = std::max(bbox_width, bbox_height) / 
                        std::max(input_image_width, input_image_height);
        std::cout << "\tAutomatically setting spacing to " << m_dem_spacing << " units/pixel.\n";
      } else {
        m_dem_spacing = val;
      }
    }

    double dem_spacing() { return m_dem_spacing; }

    BBox<float,3> bounding_box() { return m_bbox; }

    // Return the affine georeferencing transform.
    vw::Matrix<double,3,3> geo_transform() {
      vw::Matrix<double,3,3> geo_transform;
      geo_transform.set_identity();
      geo_transform(0,0) = m_dem_spacing;
      geo_transform(1,1) = -1 * m_dem_spacing;
      geo_transform(0,2) = m_bbox.min().x();
      geo_transform(1,2) = m_bbox.max().y();
      return geo_transform;
    }

    template <class ViewT>
    ImageView<PixelGrayA<float> > operator()(ImageViewBase<ViewT> const& texture) {

      VW_ASSERT(texture.impl().channels() == 1 && texture.impl().planes() == 1,
                ArgumentErr() << "Orthorasterizer: texture must be a single channel, single plane image."); 

      unsigned int width = m_point_cloud.cols();
      unsigned int height = m_point_cloud.rows();
      POS3D *coords;
      double* textures;
      

      // Start by printing out the bounding box and resulting orthoimage dimensions
      int orthoimage_width  = (int) (fabs(m_bbox.max().x() - m_bbox.min().x()) / m_dem_spacing) + 1;
      int orthoimage_height = (int) (fabs(m_bbox.max().y() - m_bbox.min().y()) / m_dem_spacing) + 1;
      std::cout << "\tDimensions = [" << orthoimage_width << ", " << orthoimage_height << "]\n";

      // This ensures that our bounding box is properly sized, though
      // I'm not sure why this is necessary... -mbroxton
      m_bbox.max().x() = m_bbox.min().x() + (orthoimage_width * m_dem_spacing);
      m_bbox.max().y() = m_bbox.min().y() + (orthoimage_height * m_dem_spacing);
      std::cout << "\tDEM BBox = " << m_bbox << "\n";

      // Allocate memory and copy over data
      try {
        coords = new POS3D[width * height];
        textures = new double[width * height];
      } catch (std::bad_alloc &e) {
        throw vw::NullPtrErr() << "FATAL ERROR in OrthoRasterizer: cannot allocate DEM buffers.";
      }
      
      ImageView<float> ortho_image(orthoimage_width, orthoimage_height);
      float *ortho_image_ptr = &(ortho_image(0,0));
      
      const int numColorComponents = 1;	   // We only need gray scale
      const int numVertexComponents = 2;	   // DEMs are 2D

      // Setup a software renderer
      vw::stereo::SoftwareRenderer renderer = vw::stereo::SoftwareRenderer(orthoimage_width, orthoimage_height, ortho_image_ptr);
      renderer.Ortho2D(m_bbox.min().x(), m_bbox.max().x(), m_bbox.min().y(), m_bbox.max().y());
      if (use_minz_as_default) {
        renderer.ClearColor(m_bbox.min().z(), m_bbox.min().z(), m_bbox.min().z(), 1.0);        
      } else {
        renderer.ClearColor(m_default_value, m_default_value, m_default_value, 1.0);
      } 
      renderer.Clear(vw::stereo::eColorBufferBit);

      std::cout << "\tOrtho-projecting image..." << std::flush;

      int num_triangles = 0;
      for (unsigned int row = 0; row < (height-1); row++) {
        for (unsigned int col = 0; col < (width-1); col++) {
          float vertices[12], intensities[6];
          int triangle_count;
          
          // Create the two triangles covering this "pixel"
          this->create_triangles(row, col, texture, triangle_count, vertices, intensities);
          
          renderer.SetVertexPointer(numVertexComponents, vw::stereo::ePackedArray, vertices);
          
          // Draw elevations into DEM buffer
          renderer.SetColorPointer(numColorComponents, vw::stereo::ePackedArray, intensities);
          for (int i = 0; i < triangle_count; i++)
            renderer.DrawPolygon(i * 3, 3);
          
          num_triangles += triangle_count;
        }
      }
      
      std::cout << "done." << std::endl;
      std::cout << "\tNumber of planet surface triangles = " << num_triangles << std::endl;

      // Free resources
      delete [] coords;
      delete [] textures;
  
      // Create a DEM with alpha
      //
      // The software renderer returns an image which will render
      // upside down in most image formats, so we correct that here as
      // well.
      ImageView<PixelGrayA<float> > ortho_alpha_image(ortho_image.cols(), ortho_image.rows());
      for (int j = 0; j < ortho_image.rows(); ++j) {
        for (int i = 0; i < ortho_image.cols(); ++i) {
          if (ortho_image(i,ortho_image.rows()-1-j) == 0) {
            ortho_alpha_image(i,j) = PixelGrayA<float>();
          } else {
            ortho_alpha_image(i,j) = PixelGrayA<float>(ortho_image(i,ortho_image.rows()-1-j));
            //            ortho_alpha_image(i,j) = PixelGrayA<float>(0.5);
          }
        }
      }

      return ortho_alpha_image;
    }
  private:
    template <class ViewT>
    inline void create_triangles(int row, int col, 
                                ImageViewBase<ViewT> const& texture,
                                int &triangleCount,  
                                float vertices[12],
                                float intensities[6])
    {
      triangleCount = 0;
      
      // triangle 1 is: LL, LR, UL
      if (m_point_cloud(col,row+1)   != PositionT() &&   // LL
          m_point_cloud(col+1,row+1) != PositionT() &&   // LR
          m_point_cloud(col,row)     != PositionT() )    // UL
        {
          vertices[0] = m_point_cloud(col, row+1).x();  // LL
          vertices[1] = m_point_cloud(col, row+1).y();
          vertices[2] = m_point_cloud(col+1,row+1).x(); // LR
          vertices[3] = m_point_cloud(col+1,row+1).y();
          vertices[4] = m_point_cloud(col,row).x();     // UL
          vertices[5] = m_point_cloud(col,row).y(); 

          intensities[0] = texture.impl()(col, row+1); // LL
          intensities[1] = texture.impl()(col+1,row+1);// LR
          intensities[2] = texture.impl()(col,row);    // UL
          
          triangleCount++;
        }
      
      // triangle 2 is: UL, LR, UR
      if (m_point_cloud(col,row)     != PositionT() &&   // UL
          m_point_cloud(col+1,row+1) != PositionT() &&   // LR
          m_point_cloud(col+1,row)   != PositionT() )    // UR
        {
          int vertex0 = triangleCount * 6;
          int elev0 = triangleCount * 3;
          
          vertices[vertex0] = m_point_cloud(col,row).x(); // UL
          vertices[vertex0 + 1] = m_point_cloud(col,row).y();
          vertices[vertex0 + 2] = m_point_cloud(col+1,row+1).x(); // LR
          vertices[vertex0 + 3] = m_point_cloud(col+1,row+1).y();
          vertices[vertex0 + 4] = m_point_cloud(col+1,row).x(); // UR
          vertices[vertex0 + 5] = m_point_cloud(col+1,row).y();

          intensities[elev0] = texture.impl()(col,row);
          intensities[elev0 + 1] = texture.impl()(col+1,row+1);
          intensities[elev0 + 2] = texture.impl()(col+1,row);
          
          triangleCount++;
        }
    }
    
  };
  

}} // namespace vw::cartography
