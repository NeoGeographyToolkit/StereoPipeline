#ifndef __STEREO_GUI_SESSION_H__
#define __STEREO_GUI_SESSION_H__

#include <QObject>

#include <vw/Image.h>
#include <vw/Stereo.h>

#include "gui/KyleStereo.h"

class StereoGuiSession : public QObject {
  Q_OBJECT

  Q_ENUMS(CostType)
  Q_ENUMS(SubpixelType)
  
public:
  enum CostType {
    ABS_DIFFERENCE_COST,
    SQ_DIFFERENCE_COST,
    NORM_XCORR_COST
  };

  enum SubpixelType {
    PARABOLA_SUBPIXEL,
    AFFINE_ADAPTIVE_SUBPIXEL
  };
  
private:
  int m_kernSize;
  int m_costBlurSize;
  int m_xSearchOffset, m_ySearchOffset;
  int m_xSearchSize, m_ySearchSize;

  vw::ImageView<vw::float32> m_leftImage, m_rightImage;
  vw::ImageView<vw::PixelDisparity<vw::float32> > m_disparity_map;
  
  CostType m_costType;
  SubpixelType m_subpixelType;
  
  boost::shared_ptr<vw::stereo::CostFunction<vw::float32> > m_costFunctionObject;
  
  vw::Vector2i m_searchWindowPreviewPoint;
  bool m_crosshairEnabled;
  
  int m_xDisparityPreview, m_yDisparityPreview;

  void updateCostFunctionObject() {
    if (hasBothImagesLoaded()) {
        switch (m_costType) {
        case ABS_DIFFERENCE_COST:
          m_costFunctionObject = boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >(new vw::stereo::AbsDifferenceCost<vw::float32>(m_leftImage, m_rightImage, m_kernSize)); 
          break;
        case SQ_DIFFERENCE_COST:
          m_costFunctionObject = boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >(new vw::stereo::SqDifferenceCost<vw::float32>(m_leftImage, m_rightImage, m_kernSize));
          break;
        case NORM_XCORR_COST:
          m_costFunctionObject = boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >(new vw::stereo::NormXCorrCost<vw::float32>(m_leftImage, m_rightImage, m_kernSize));
            break;
        }
        
        if (m_costBlurSize != 0) {
          m_costFunctionObject = boost::shared_ptr<vw::stereo::CostFunction<vw::float32> >(new vw::stereo::BlurCost<vw::float32>(m_costFunctionObject, m_costBlurSize));
        }
        
        emit costFunctionObjectChanged(m_costFunctionObject);
    }
  }
  
  public:
  StereoGuiSession(QObject *parent = 0) : QObject(parent),
                                             m_kernSize(25),
                                             m_costBlurSize(0),
                                             
                                             m_xSearchOffset(0),
                                             m_ySearchOffset(0),
                                             
                                             m_xSearchSize(10),
                                             m_ySearchSize(10),
                                             
                                             m_costType(ABS_DIFFERENCE_COST),
                                             
                                             m_crosshairEnabled(false),
                                             
                                             m_xDisparityPreview(0),
                                             m_yDisparityPreview(0) {}
  
public:
  
  // Convenience functions
  
  vw::BBox2i searchWindow() {
    return vw::BBox2i(m_xSearchOffset, m_ySearchOffset, m_xSearchSize, m_ySearchSize);
  }
  
  bool hasBothImagesLoaded() {
    return m_leftImage.cols() != 0 && 
      m_rightImage.cols() != 0 && 
      m_leftImage.cols() == m_rightImage.cols() && 
      m_rightImage.rows() == m_rightImage.rows();
  }
  
  bool hasDisparityMapLoaded() {
    return m_disparity_map.cols() != 0 && 
      m_disparity_map.cols() != 0 && 
      m_disparity_map.cols() == m_rightImage.cols() && 
      m_disparity_map.rows() == m_rightImage.rows();
  }

  boost::shared_ptr<vw::stereo::CostFunction<vw::float32> > getCostFunctionObject() {
    return m_costFunctionObject;
  }
  
  void setDisparityPreview(int dx, int dy) {
    m_xDisparityPreview = dx;
    m_yDisparityPreview = dy; 
    emit xDisparityPreviewChanged(dx);
    emit yDisparityPreviewChanged(dy);      
    emit disparityPreviewChanged(dx, dy);
  }
  
  // Properties
  
  int kernSize() {
    return m_kernSize;
  }

  int costBlurSize() {
    return m_costBlurSize;
  }
  
  int xSearchSize() {
    return m_xSearchSize;
  }

  int ySearchSize() { 
    return m_ySearchSize;
  }

  int xSearchOffset() {
    return m_xSearchOffset;
  }

  int ySearchOffset() {
    return m_ySearchOffset;
  }

  vw::ImageView<vw::float32> leftImage() {
    return m_leftImage;
  }

  vw::ImageView<vw::float32> rightImage() {
    return m_rightImage;
  }

  vw::ImageView<vw::PixelDisparity<vw::float32> > disparityMap() {
    return m_disparity_map;
  }

  CostType costType() {
    return m_costType;
  }

  vw::Vector2i searchWindowPreviewPoint() {
    return m_searchWindowPreviewPoint;
  }

  bool crosshairEnabled() { 
    return m_crosshairEnabled;
  }

  int xDisparityPreview() {
    return m_xDisparityPreview;
  }

  int yDisparityPreview() {
    return m_yDisparityPreview;
  }

public slots:
  void setKernSize(int kernSize) {
    if (m_kernSize != kernSize) {
      m_kernSize = kernSize;
      updateCostFunctionObject();
      emit kernSizeChanged(kernSize);
    }
  }

  void setCostBlurSize(int costBlurSize) {
    if (m_costBlurSize != costBlurSize) {
      m_costBlurSize = costBlurSize;
      updateCostFunctionObject();
      emit costBlurSizeChanged(costBlurSize);
    }
  }

  void setXSearchSize(int xSearchSize) {
    if (m_xSearchSize != xSearchSize) {
      m_xSearchSize = xSearchSize;
      emit xSearchSizeChanged(xSearchSize);
      emit searchWindowChanged(searchWindow());
    }
  }

  void setYSearchSize(int ySearchSize) {
    if (m_ySearchSize != ySearchSize) {
      m_ySearchSize = ySearchSize;
      emit ySearchSizeChanged(ySearchSize);
      emit searchWindowChanged(searchWindow());
    }
  }

  void setXSearchOffset(int xSearchOffset) {
    if (m_xSearchOffset != xSearchOffset) {
      m_xSearchOffset = xSearchOffset;
      emit xSearchOffsetChanged(xSearchOffset);
      emit searchWindowChanged(searchWindow());
    }
  }

  void setYSearchOffset(int ySearchOffset) {
    if (m_ySearchOffset != ySearchOffset) {
      m_ySearchOffset = ySearchOffset;
      emit ySearchOffsetChanged(ySearchOffset);
      emit searchWindowChanged(searchWindow());
    }
  }
    
  void setLeftImage(vw::ImageView<vw::float32>  leftImage) {
    m_leftImage = leftImage;
    updateCostFunctionObject();
    emit leftImageChanged(leftImage);
  }

  void setRightImage(vw::ImageView<vw::float32>  rightImage) {
    m_rightImage = rightImage;
    updateCostFunctionObject();
    emit rightImageChanged(rightImage);
  }

  void setImages(vw::ImageView<vw::float32>  leftImage, vw::ImageView<vw::float32>  rightImage) {
    m_leftImage = leftImage;
    m_rightImage = rightImage;
    updateCostFunctionObject();
    emit leftImageChanged(leftImage);
    emit rightImageChanged(rightImage);
  }

  void setDisparityMap(vw::ImageView<vw::PixelDisparity<vw::float32> > disparityMap) {
    m_disparity_map = disparityMap;
    emit disparityMapChanged(disparityMap);
  }

  void setCostType(StereoGuiSession::CostType costType) {
    if (m_costType != costType) {
      m_costType = costType;
      updateCostFunctionObject();
      emit costTypeChanged(costType);
    }
  }

  void setSubpixelType(StereoGuiSession::SubpixelType subpixelType) {
    if (m_subpixelType != subpixelType) {
      m_subpixelType = subpixelType;
      emit subpixelTypeChanged(subpixelType);
    }
  }

  void setSearchWindowPreviewPoint(vw::Vector2i searchWindowPreviewPoint) {
    if (m_searchWindowPreviewPoint != searchWindowPreviewPoint) {
      m_searchWindowPreviewPoint = searchWindowPreviewPoint;
      emit searchWindowPreviewPointChanged(searchWindowPreviewPoint);
    }
  }

  void setCrosshairEnabled(bool crosshairEnabled) {
    if (m_crosshairEnabled != crosshairEnabled) {
      m_crosshairEnabled = crosshairEnabled;
      emit crosshairEnabledChanged(crosshairEnabled);
    }
  }

  void setXDisparityPreview(int xDisparityPreview) {
    if (m_xDisparityPreview != xDisparityPreview) {
      m_xDisparityPreview = xDisparityPreview;
      emit xDisparityPreviewChanged(xDisparityPreview);
      emit disparityPreviewChanged(m_xDisparityPreview, m_yDisparityPreview);
    }
  }

  void setYDisparityPreview(int yDisparityPreview) {
    if (m_yDisparityPreview != yDisparityPreview) {
      m_yDisparityPreview = yDisparityPreview;
      emit yDisparityPreviewChanged(yDisparityPreview);
      emit disparityPreviewChanged(m_xDisparityPreview, m_yDisparityPreview);
    }
  }

signals:
  void kernSizeChanged(int kernSize);
  void costBlurSizeChanged(int costBlurSize);
  void xSearchSizeChanged(int xSearchSize);
  void ySearchSizeChanged(int ySearchSize);
  void xSearchOffsetChanged(int xSearchOffset);
  void ySearchOffsetChanged(int ySearchOffset);
  void leftImageChanged(vw::ImageView<vw::float32> leftImage);
  void rightImageChanged(vw::ImageView<vw::float32> rightImage);
  void disparityMapChanged(vw::ImageView<vw::PixelDisparity<vw::float32> > disparityMap);
  void costTypeChanged(StereoGuiSession::CostType costType);
  void subpixelTypeChanged(StereoGuiSession::SubpixelType subpixelType);
  void searchWindowPreviewPointChanged(vw::Vector2i searchWindowPreviewPoint);
  void crosshairEnabledChanged(bool crosshairEnabled);
  void costFunctionObjectChanged(boost::shared_ptr<vw::stereo::CostFunction<vw::float32> > costFunctionObject);
  void searchWindowChanged(vw::BBox2i searchWindow);
  void xDisparityPreviewChanged(int xDisparityPreview);
  void yDisparityPreviewChanged(int yDisparityPreview);
  void disparityPreviewChanged(int xDisparityPreview, int yDisparityPreview);
};

#endif
