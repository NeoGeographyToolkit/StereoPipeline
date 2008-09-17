#ifndef __INPUT_WIDGET_H__
#define __INPUT_WIDGET_H__

#include <vw/Image/ImageView.h>

// Forward Declarations
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QString>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
class PreviewGLWidget;

class InputWidget : public QWidget {
  Q_OBJECT

  // Private member variables
  PreviewGLWidget *m_glPreview;
  vw::ImageView<vw::PixelRGB<vw::float32> > m_inputImage;
  
  QPushButton *m_fileBrowseButton;
  QLineEdit *m_fileNameEdit;
  
  // Private methods
  QGroupBox *genSettingsBox(QString const& name);
    
public:
  InputWidget(QString const& name, QWidget *parent = 0);
  
  vw::ImageView<vw::PixelRGB<vw::float32> > result() const { return m_inputImage; }
  
  bool hasImageLoaded() const { return m_inputImage.cols() != 0; }

private slots:
  void fileBrowseButtonClicked();
  void loadImage();
};

#endif // __PREPROCESS_WIDGET_H__
