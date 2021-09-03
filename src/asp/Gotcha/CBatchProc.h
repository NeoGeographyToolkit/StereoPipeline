#ifndef CBATCHPROC_H
#define CBATCHPROC_H

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <CTiePt.h>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

class CBatchProc {
public:
  CBatchProc(std::string          const & strMetaFile,
             vw::ImageView<float> const & imgL,
             vw::ImageView<float> const & imgR, 
             vw::ImageView<float> const & input_dispX,
             vw::ImageView<float> const & input_dispY);

  ~CBatchProc();
  
  void doBatchProcessing(vw::ImageView<float> & output_dispX,
                         vw::ImageView<float> & output_dispY);
  
private:
    void setProjParameter();
    bool validateProjParam();
    bool validateProjInputs();
    void generateMask();
    void generateTPFile(std::vector<CTiePt> & vecTPs);

    Point3f rotate(Point3f ptIn, cv::Mat matQ, bool bInverse);
    void quaternionMultiplication(const float* p, const float* q, float* pfOut);

protected:
    // processing
  void refinement(std::vector<CTiePt> const& vecTPs,
                  vw::ImageView<float> & output_dispX,
                  vw::ImageView<float> & output_dispY);

protected:
  std::string m_strMetaFile;   // file path to the Metadata file
#if 0
  std::string m_strImgL;
  std::string m_strImgR;
  std::string m_strDispX;
  std::string m_strDispY;
  string m_strMask;
  std::string m_strTPFile;
  std::string m_strOutPath;    // a user-supplied file path for the output directory
#endif
  
  cv::Mat m_imgL, m_imgR;
  cv::Mat m_input_dispX, m_input_dispY;
  cv::Mat m_Mask;
};

// Apply Gotcha refinement to each padded tile
class GotchaPerBlockView: public vw::ImageViewBase<GotchaPerBlockView>{
  vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> m_input_disp;
  vw::ImageViewRef<float> m_left_img, m_right_img;
  int m_padding;
  std::string m_casp_go_param_file;
  
  typedef vw::PixelMask<vw::Vector2f> PixelT;

public:
  GotchaPerBlockView(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> input_disp,
                     vw::ImageViewRef<float> left_img,
                     vw::ImageViewRef<float> right_img,
                     int padding, std::string const& casp_go_param_file):
    m_input_disp(input_disp), m_left_img(left_img), m_right_img(right_img),
    m_padding(padding), m_casp_go_param_file(casp_go_param_file){}
  
  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<GotchaPerBlockView> pixel_accessor;

  inline vw::int32 cols() const { return m_input_disp.cols(); }
  inline vw::int32 rows() const { return m_input_disp.rows(); }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, vw::int32/*p*/ = 0 ) const {
    vw::vw_throw(vw::NoImplErr() << "GotchaPerBlockView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef vw::CropView<vw::ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {

    // Need to see a bit more of the input image to avoid tiling artifacts
    vw::BBox2i biased_box = bbox;
    biased_box.expand(m_padding);
    biased_box.crop(bounding_box(m_input_disp));

    // Run Gotcha on the expanded and cropped tile.
    // TODO(oalexan1): Verify that it assumes a value of 0 for invalid disparities
    vw::ImageView<result_type> cropped_disp = crop(m_input_disp, biased_box);
    vw::ImageView<float> output_dispX, output_dispY;
    CBatchProc batchProc(m_casp_go_param_file,
                         crop(m_left_img, biased_box), 
                         crop(m_right_img, biased_box), 
                         vw::select_channel(cropped_disp, 0),
                         vw::select_channel(cropped_disp, 1));
    batchProc.doBatchProcessing(output_dispX, output_dispY);
    
    // Integrate back the processed bands.
    vw::select_channel(cropped_disp, 0) = output_dispX;
    vw::select_channel(cropped_disp, 1) = output_dispY;

    return prerasterize_type(cropped_disp, -biased_box.min().x(), -biased_box.min().y(),
                             cols(), rows());
  }
  
  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

GotchaPerBlockView gotcha_refine(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> input_disp,
                                 vw::ImageViewRef<float> left_img,
                                 vw::ImageViewRef<float> right_img,
                                 int padding, std::string const& casp_go_param_file){
  return GotchaPerBlockView(input_disp, left_img, right_img, padding, casp_go_param_file);
}


#endif // CBATCHPROC_H
