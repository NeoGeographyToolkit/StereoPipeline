#ifndef __STEREO_SESSION_H__
#define __STEREO_SESSION_H__

#include <vw/Camera/CameraModel.h>

#include <boost/shared_ptr.hpp>

class StereoSession {
  
protected:
  std::string m_left_image_file;
  std::string m_right_image_file;
  std::string m_left_camera_file;
  std::string m_right_camera_file;
  std::string m_out_prefix;
  std::string m_extra_argument1;
  std::string m_extra_argument2;
  std::string m_extra_argument3;
  std::string m_extra_argument4;
  
public:

  void initialize (std::string const& left_image_file, std::string const& right_image_file,
                   std::string const& left_camera_file, std::string const& right_camera_file,
                   std::string const& out_prefix,
                   std::string const& extra_argument1, std::string const& extra_argument2,
                   std::string const& extra_argument3, std::string const& extra_argument4) { 
    m_left_image_file = left_image_file;
    m_right_image_file = right_image_file;
    m_left_camera_file = left_camera_file;
    m_right_camera_file = right_camera_file;
    m_out_prefix = out_prefix;
    m_extra_argument1 = extra_argument1;
    m_extra_argument2 = extra_argument2;
    m_extra_argument3 = extra_argument3;
    m_extra_argument4 = extra_argument4;
  }

  virtual ~StereoSession() {}

  // Methods (mostly static) for registering and creating stereo sessions.
  static StereoSession* create( std::string const& session_type );
  typedef StereoSession* (*construct_func)();
  static void register_session_type( std::string const& id, construct_func func);
  

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2) {
    cam1 = camera_model(m_left_image_file, m_left_camera_file);
    cam2 = camera_model(m_right_image_file, m_right_camera_file);
  }
  
  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "") = 0;

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string &output_file1, std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  virtual void post_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string &output_file1, std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  // Stage 2: Correlation
  //
  // Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
  // Post file is a disparity map.            ( ImageView<PixelDisparity<float> > )
  virtual void pre_correlation_hook(std::string const& input_file1, std::string const& input_file2,
                                    std::string &output_file1, std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  virtual void post_correlation_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  // Stage 3: Filtering
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  // Post file is a disparity map. ( ImageView<PixelDisparity<float> > )
  virtual void pre_filtering_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  virtual void post_filtering_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  // Stage 4: Point cloud generation
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  // Post file is point image.     ( ImageView<Vector3> )
  virtual void pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  virtual void post_pointcloud_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  // Stage 4: Wire mesh/dtm generation
  //
  // Pre and post file is a point image.  ( ImageView<Vector3> )
  virtual void pre_wiremesh_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }

  virtual void post_wiremesh_hook(std::string const& input_file, std::string & output_file) {
    output_file = input_file;
  }
  
};

#endif // __STEREO_SESSION_H__
