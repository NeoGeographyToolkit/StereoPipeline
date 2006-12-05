
#include <vw/Image/ImageView.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <vw/Stereo/SoftwareRenderer.h>

namespace vw {
namespace cartography {

  template <class PositionT, class ChannelT>
  class OrthoRasterizer {
    
    ImageView<PositionT> m_point_cloud;
    ImageView<ChannelT> m_texture;
    float m_dem_spacing;
    BBox<float,3> m_bbox;
    bool m_swap_xy;
    
  public:
    ChannelT default_value;
    bool use_minz_as_default;

    OrthoRasterizer(ImageView<PositionT> point_cloud, 
                    ImageView<double> texture,
                    bool swap_xy = false,
                    float spacing = 0.0) : 
      default_value(0), use_minz_as_default(true), m_swap_xy(swap_xy) {
      
      m_point_cloud = point_cloud;
      m_texture = texture;

      for (unsigned int i= 0; i < m_point_cloud.cols(); i++) 
        for (unsigned int j = 0; j < m_point_cloud.rows(); j++) 
          if (m_point_cloud(i,j) != PositionT())   // If the position is not missing
            if (m_swap_xy) 
              m_bbox.grow(Vector3(m_point_cloud(i,j).y(), 
                                  m_point_cloud(i,j).x(),
                                  m_point_cloud(i,j).z()));
            else 
              m_bbox.grow(m_point_cloud(i,j));

      set_dem_spacing(spacing);
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

    BBox<float,3> bounding_box() { 
      return m_bbox;
    }

    vw::Matrix<double,3,3> geo_transform() {
      // Set up the georeferencing transform 
      vw::Matrix<double,3,3> geo_transform;
      geo_transform.set_identity();
      geo_transform(0,0) = m_dem_spacing;
      geo_transform(1,1) = -1 * m_dem_spacing;
      geo_transform(0,2) = m_bbox.min().x();
      geo_transform(1,2) = m_bbox.max().y(); 
    }

    ImageView<ChannelT> rasterize() {

      unsigned int width = m_point_cloud.cols();
      unsigned int height = m_point_cloud.rows();
      POS3D *coords;
      double* textures;

      try {
        coords = new POS3D[width * height];
        textures = new double[width * height];
      } catch (std::bad_alloc &e) {
        throw vw::NullPtrErr() << "FATAL ERROR in OrthoRasterizer: cannot allocate DEM buffers.";
      }
      
      // The software renderer is expecting an array of POS3D's 
      for (unsigned int i = 0; i < m_point_cloud.cols(); i++) {
        for (unsigned int j = 0; j < m_point_cloud.rows(); j++) {
          if (m_swap_xy) {
            coords[j * width + i].x = m_point_cloud(i,j).y();
            coords[j * width + i].y = m_point_cloud(i,j).x();
          } else {
            coords[j * width + i].x = m_point_cloud(i,j).x();
            coords[j * width + i].y = m_point_cloud(i,j).y();
          }
          
          // Copy over the z coordinate if there is one to copy.
          // Otherwise, substitute in a zero.
          if (m_point_cloud.channels() > 2) 
            coords[j * width + i].z = m_point_cloud(i,j)[2];
          else 
            coords[j * width + i].z = 0;

          textures[j * width + i] = m_texture(i,j);      
        }
      }
      
      std::cout << "\tDEM BBox found = " << m_bbox << "\n";
            
      int orthoimage_width  = (int) (fabs(m_bbox.max().x() - m_bbox.min().x()) / m_dem_spacing) + 1;
      int orthoimage_height = (int) (fabs(m_bbox.max().y() - m_bbox.min().y()) / m_dem_spacing) + 1;
      std::cout << "\tDimensions = [" << orthoimage_width << ", " << orthoimage_height << "]\n";
      
      m_bbox.max().x() = m_bbox.min().x() + (orthoimage_width * m_dem_spacing);
      m_bbox.max().y() = m_bbox.min().y() + (orthoimage_height * m_dem_spacing);
      
      ImageView<float> ortho_image(orthoimage_width, orthoimage_height);
      float *ortho_image_ptr = &(ortho_image(0,0));
      
      const int numColorComponents = 1;	   // We only need gray scale
      const int numVertexComponents = 2;	   // DEMs are 2D

      // Setup a software renderer
      SoftwareRenderer renderer = SoftwareRenderer(orthoimage_width, orthoimage_height, ortho_image_ptr);
      renderer.Ortho2D(m_bbox.min().x(), m_bbox.max().x(), m_bbox.min().y(), m_bbox.max().y());
      if (use_minz_as_default) {
        renderer.ClearColor(m_bbox.min().z(), m_bbox.min().z(), m_bbox.min().z(), 1.0);        
      } else {
        renderer.ClearColor(default_value, default_value, default_value, 1.0);
      } 
      renderer.Clear(eColorBufferBit);

      std::cout << "\tOrtho-projecting image..." << std::flush;

      int num_triangles = 0;
      for (unsigned int row = 0; row < (height-1); row++) {
        for (unsigned int col = 0; col < (width-1); col++) {
          float vertices[12], intensities[6];
          int triangle_count;
          
          // Create the two triangles covering this "pixel"
          CreateTriangles(row, col, width, coords, textures,
                          triangle_count, vertices, intensities);
          
          renderer.SetVertexPointer(numVertexComponents, ePackedArray, vertices);
          
          // Draw elevations into DEM buffer
          renderer.SetColorPointer(numColorComponents, ePackedArray, intensities);
          for (int i = 0; i < triangle_count; i++)
            renderer.DrawPolygon(i * 3, 3);
          
          num_triangles += triangle_count;
        }
      }
      
      std::cout << "done." << std::endl;
      std::cout << "\tNumber of planet surface triangles = " << num_triangles << std::endl;
  
      // Clean up 
      delete [] coords;
      delete [] textures;

      // The software renderer returns an image which will render upside
      // down in most image formats
      return vw::flip_vertical(ortho_image);
    }
  private:
    void CreateTriangles(int row, int col, int width, 
                         POS3D *coords,
                         double* textures,
                         int &triangleCount, float vertices[12],
                         float intensities[6])
    {
      PixelCoords upperLeft(col, row);
      PixelCoords upperRight(col + 1, row);
      PixelCoords lowerRight(col + 1, row + 1);
      PixelCoords lowerLeft(col, row + 1);
      int indexUL = upperLeft.PixelIndex(width);
      int indexUR = upperRight.PixelIndex(width);
      int indexLR = lowerRight.PixelIndex(width);
      int indexLL = lowerLeft.PixelIndex(width);
      
      triangleCount = 0;
      
      // triangle 1 is: LL, LR, UL
      if ((coords[indexLL].x != 0) && (coords[indexLL].y != 0) && (coords[indexLL].z != 0) &&
          (coords[indexLR].x != 0) && (coords[indexLR].y != 0) && (coords[indexLR].z != 0) &&
          (coords[indexUL].x != 0) && (coords[indexUL].y != 0) && (coords[indexUL].z != 0))
        {
          vertices[0] = coords[indexLL].x;
          vertices[1] = coords[indexLL].y;
          
          vertices[2] = coords[indexLR].x;
          vertices[3] = coords[indexLR].y;
          
          vertices[4] = coords[indexUL].x;
          vertices[5] = coords[indexUL].y;
          
          intensities[0] = textures[indexLL];
          intensities[1] = textures[indexLR];
          intensities[2] = textures[indexUL];
          
          triangleCount++;
        }
      
      // triangle 2 is: UL, LR, UR
      if ((coords[indexUL].x != 0) && (coords[indexUL].y != 0) && (coords[indexUL].z != 0) &&
          (coords[indexLR].x != 0) && (coords[indexLR].y != 0) && (coords[indexLR].z != 0) &&
          (coords[indexUR].x != 0) && (coords[indexUR].y != 0) && (coords[indexUR].z != 0))
        {
          int vertex0 = triangleCount * 6;
          int elev0 = triangleCount * 3;
          
          vertices[vertex0] = coords[indexUL].x;
          vertices[vertex0 + 1] = coords[indexUL].y;
          
          vertices[vertex0 + 2] = coords[indexLR].x;
          vertices[vertex0 + 3] = coords[indexLR].y;
          
          vertices[vertex0 + 4] = coords[indexUR].x;
          vertices[vertex0 + 5] = coords[indexUR].y;
          
          intensities[elev0] = textures[indexUL];
          intensities[elev0 + 1] = textures[indexLR];
          intensities[elev0 + 2] = textures[indexUR];
          
          triangleCount++;
        }
    }
    
  };
  

}} // namespace vw::cartography
