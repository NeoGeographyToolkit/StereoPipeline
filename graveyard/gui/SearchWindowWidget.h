// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#ifndef __SEARCH_WINDOW_WIDGET_H__
#define __SEARCH_WINDOW_WIDGET_H__

#include <QSpinBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QLabel>
#include <QGroupBox>
#include <QComboBox>
#include <QMetaObject>
#include <QMetaEnum>

#include <vw/Image.h>

#include "gui/StereoGuiSession.h"
#include "gui/PreviewGLWidget.h"

class CostTypeComboBox : public QComboBox {
  Q_OBJECT

  Q_ENUMS(StereoGuiSession::CostType)

public:
  CostTypeComboBox(QWidget *parent = 0) : QComboBox(parent) {
    addItem("Absolute Difference", QVariant("ABS_DIFFERENCE_COST"));
    addItem("Squared Difference", QVariant("SQ_DIFFERENCE_COST"));
    addItem("Normalized Cross Correlation", QVariant("NORM_XCORR_COST"));
    
    connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(emitValueChanged()));
  }

  StereoGuiSession::CostType value() const {
    QString costTypeName = itemData(currentIndex()).toString();
    
    const QMetaObject metaobject = StereoGuiSession::staticMetaObject;
    int enumidx = metaobject.indexOfEnumerator("CostType");
    
    return static_cast<StereoGuiSession::CostType>(metaobject.enumerator(enumidx).keyToValue(costTypeName.toAscii().constData()));
  }

  public slots:
    void setValue(StereoGuiSession::CostType value) {
      const QMetaObject metaobject = StereoGuiSession::staticMetaObject;
      int enumidx = metaobject.indexOfEnumerator("CostType");
      
      QString costTypeName(metaobject.enumerator(enumidx).valueToKey(value));
      
      setCurrentIndex(findData(costTypeName));
    }

  private slots:
    void emitValueChanged() {
      emit valueChanged(value());
    }

  signals:
    void valueChanged(StereoGuiSession::CostType);
};

class SearchWindowWidget : public QWidget {
  Q_OBJECT

  private:
    StereoGuiSession *m_cs;
    PreviewGLWidget *m_imagePreview;

    QSpinBox *m_kernSizeSpin;
    QSpinBox *m_costBlurSizeSpin;
    QSpinBox *m_xSearchSizeSpin;
    QSpinBox *m_ySearchSizeSpin;
    QSpinBox *m_xSearchOffsetSpin;
    QSpinBox *m_ySearchOffsetSpin;

    CostTypeComboBox *m_costTypeBox;

    QGroupBox *genSettingsBox(QString const& name);
    QHBoxLayout *genSettingsKnobs();

    QLabel *m_currPixelLabel;

  public:
    SearchWindowWidget(StereoGuiSession *cs, QWidget *parent = 0);

  public slots:
    void updateWidgets();

  private slots:
    void updatePreview();
    void correlatePixel();
    void imageClicked(int x, int y);
};

#endif
