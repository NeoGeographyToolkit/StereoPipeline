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
  PreviewGLWidget *m_left_preview;
  PreviewGLWidget *m_right_preview;
  QLineEdit *m_left_filename_edit;
  QLineEdit *m_right_filename_edit;

public:
  InputWidget(QWidget *parent = 0);

  void load_left_image(std::string filename);
  void load_right_image(std::string filename);

private slots:
  void reload_left_image();
  void reload_right_image();
  void left_browse_button_clicked();
  void right_browse_button_clicked();
};

#endif // __PREPROCESS_WIDGET_H__
