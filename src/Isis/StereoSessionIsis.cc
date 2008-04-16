#include <boost/shared_ptr.hpp>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/CameraBBox.h>
//#include <vw/Cartography/OrthoImageView.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Image/UtilityViews.h>
#include <vw/Image/Transform.h>
#include <vw/Image/ImageViewRef.h>
#include "Isis/StereoSessionIsis.h"
#include "Isis/IsisCameraModel.h"

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

// /// Erases a file suffix if one exists and returns the base string
// static std::string prefix_from_filename(std::string const& filename) {
//   std::string result = filename;
//   int index = result.rfind(".");
//   if (index != -1) 
//     result.erase(index, result.size());
//   return result;
// }

/// Cam to cam image transform functor
//
// TODO : Add in support for terrain
class CamToCamTransform : public TransformHelper<CamToCamTransform,ConvexFunction,ConvexFunction> {
  boost::shared_ptr<CameraModel> m_src_cam;
  boost::shared_ptr<CameraModel> m_dst_cam;
  GeoReference m_georef;
  
  double semi_major_axis;
  double semi_minor_axis;
  double z_scale;
  double radius;
  double radius_sqr;

public:    
  CamToCamTransform( boost::shared_ptr<CameraModel> src_cam,
                     boost::shared_ptr<CameraModel> dst_cam,
                     GeoReference georef):
    m_src_cam(src_cam), m_dst_cam(dst_cam), m_georef(georef) {
    semi_major_axis = georef.datum().semi_major_axis();
    semi_minor_axis = georef.datum().semi_minor_axis();
    z_scale = semi_major_axis / semi_minor_axis;
    radius = semi_major_axis;
    radius_sqr = radius*radius;
  }

  Vector3 compute_intersection(boost::shared_ptr<CameraModel> camera_model,
                               Vector2 pix) const {
    
    Vector3 center = camera_model->camera_center( pix );
    Vector3 vector = camera_model->pixel_to_vector( pix );
    center.z() *= z_scale;
    vector.z() *= z_scale;
    vector = normalize( vector );

    double alpha = - dot_prod(center,vector);
    Vector3 projection = center + alpha * vector;
    if( norm_2_sqr(projection) > radius_sqr )
      vw_throw(LogicErr() << "Error in CamToCamTransform: Ray does not intersect geoid.");
    alpha -= sqrt( radius_sqr - norm_2_sqr(projection) );
    Vector3 intersection = center + alpha * vector;
    intersection.z() /= z_scale;
    return intersection;
  }

  inline Vector2 reverse( Vector2 const& p ) const {
    Vector3 xyz = compute_intersection(m_dst_cam, p);
    return m_src_cam->point_to_pixel(xyz);
    }
  
  inline Vector2 forward( Vector2 const& p ) const {
    Vector3 xyz = compute_intersection(m_src_cam, p);
    return m_dst_cam->point_to_pixel(xyz);
  }
};

void StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                                   std::string & output_file1, std::string & output_file2) {

  DiskImageView<PixelGray<uint8> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<uint8> > right_disk_image(m_right_image_file);

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  boost::shared_ptr<camera::CameraModel> cam1, cam2;
  this->camera_models(cam1, cam2);

  const double MOLA_PEDR_EQUATORIAL_RADIUS =  3396000.0;
  vw::cartography::Datum mars_datum;
  mars_datum.name() = "IAU2000 Mars Spheroid";
  mars_datum.spheroid_name() = "IAU2000 Mars Spheroid";
  mars_datum.meridian_name() = "Mars Prime Meridian";
  mars_datum.set_semi_major_axis(MOLA_PEDR_EQUATORIAL_RADIUS);
  mars_datum.set_semi_minor_axis(MOLA_PEDR_EQUATORIAL_RADIUS);
  GeoReference terrain_ref(mars_datum);

//   BBox2 bbox1 = camera_bbox(terrain_ref, cam1,left_disk_image.cols(), left_disk_image.rows());
//   std::cout << "Bounding Box 1: " << bbox1 << "\n";
//   BBox2 bbox2 = camera_bbox(terrain_ref, cam2,right_disk_image.cols(), right_disk_image.rows());
//   std::cout << "Bounding Box 2: " << bbox2 << "\n";

//   BBox2 joint_bbox = bbox1;
//   joint_bbox.grow(bbox2);
//   std::cout << "Combined Bounding Box: " << joint_bbox << "\n";
  
//   Matrix3x3 affine;
//   affine.set_identity();
//   affine(0,0) = joint_bbox.width() / left_disk_image.cols()*2;
//   affine(1,1) = joint_bbox.width() / left_disk_image.cols()*2;
//   affine(0,2) = joint_bbox.min().x();
//   affine(1,2) = joint_bbox.min().y();
//   terrain_ref.set_transform(affine);
//   std::cout << "AFFINE: " << affine << "\n";
  
//   ConstantView<float> terrain(0, joint_bbox.width()/affine(0,0), joint_bbox.height()/affine(1,1));
  
//   ImageViewRef<PixelGray<uint8> > ortho1 = orthoproject(terrain ,terrain_ref,
//                                                         left_disk_image,cam1,
//                                                         BilinearInterpolation(),
//                                                         ZeroEdgeExtension());
                                                        
//   ImageViewRef<PixelGray<uint8> > ortho2 = orthoproject(terrain ,terrain_ref,
//                                                         right_disk_image,cam2,
//                                                         BilinearInterpolation(),
//                                                         ZeroEdgeExtension());
                                                        
//   write_image(output_file1, ortho1, TerminalProgressCallback());
//   write_image(output_file2, ortho2, TerminalProgressCallback());

  ImageViewRef<PixelGray<uint8> > reproj2 = crop(transform(right_disk_image, 
                                                           CamToCamTransform(cam2, cam1, terrain_ref), ZeroEdgeExtension()),
                                                 0,0,left_disk_image.cols(),left_disk_image.rows());
  write_image(output_file1, left_disk_image, TerminalProgressCallback());
  write_image(output_file2, reproj2, TerminalProgressCallback());

  exit(0);
}


void StereoSessionIsis::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                      boost::shared_ptr<camera::CameraModel> &cam2) {
  cam1 = boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(m_left_image_file));
  cam2 = boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(m_right_image_file));
}

