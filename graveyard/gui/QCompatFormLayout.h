// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __QCOMPAT_FORM_LAYOUT_H__
#define __QCOMPAT_FORM_LAYOUT_H__

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

#endif
