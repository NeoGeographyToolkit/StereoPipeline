#include <QtGui>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Stereo.h>

using namespace vw;

#include "gui/InputWidget.h"
#include "gui/PreviewGLWidget.h"

class QCompatFormLayout : public QVBoxLayout {

public:
  QCompatFormLayout(QWidget *parent = 0) : QVBoxLayout(parent) {}
  
  void addRow(QWidget *label, QWidget *field) {
    QHBoxLayout *layout = new QHBoxLayout;
    
    layout->addWidget(label);
    layout->addWidget(field);

    this->addLayout(layout);
  }
  
  void addRow(QWidget *label, QLayout *field) {
    QHBoxLayout *layout = new QHBoxLayout;
    
    layout->addWidget(label);
    layout->addLayout(field);
    
    this->addLayout(layout);
  }
  
  void addRow(QLayout *layout) {
    this->addLayout(layout);
  }
  
  void addRow(QWidget *widget) {
    this->addWidget(widget);
  }
};

InputWidget::InputWidget(QString const& name, QWidget *parent) : QWidget(parent) {
  m_glPreview = new PreviewGLWidget(this);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(m_glPreview);
  mainLayout->addWidget(genSettingsBox(name + " Settings"));
  this->setLayout(mainLayout);

  connect(m_fileBrowseButton, SIGNAL(clicked()), this, SLOT(fileBrowseButtonClicked()));
  connect(m_fileNameEdit, SIGNAL(returnPressed()), this, SLOT(loadImage()));
}

QGroupBox *InputWidget::genSettingsBox(QString const& name) {
  QLabel *fileLabel = new QLabel("Filename: ");

  m_fileNameEdit = new QLineEdit;
  m_fileBrowseButton = new QPushButton("Browse");
  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(fileLabel);
  layout->addWidget(m_fileNameEdit);
  layout->addWidget(m_fileBrowseButton);

  QGroupBox *box = new QGroupBox(name);
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  box->setLayout(layout);
  return box;
}


void InputWidget::fileBrowseButtonClicked() {
  QString filename = QFileDialog::getOpenFileName(this, "Open...", "", "Images (*.png *.jpg *.tif *.cub *.img)");

  if (filename != "") {
    m_fileNameEdit->setText(filename);
    loadImage();
  }
}

void InputWidget::loadImage() {
  QString filename = m_fileNameEdit->text();

  if (filename != "") {
    try {
      read_image(m_inputImage, filename.toStdString());
      m_fileNameEdit->setText(filename);
      m_glPreview->setImage(m_inputImage);
      m_glPreview->sizeToFit();
    }
    catch(vw::Exception& e) {
      QMessageBox::critical(this, "Error opening image", e.what());
    }
  }
}
