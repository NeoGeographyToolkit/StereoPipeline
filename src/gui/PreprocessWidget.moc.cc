/****************************************************************************
** Meta object code from reading C++ file 'PreprocessWidget.h'
**
** Created: Mon Sep 15 23:51:56 2008
**      by: The Qt Meta Object Compiler version 59 (Qt 4.3.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "PreprocessWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PreprocessWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 59
#error "This file was generated using the moc from 4.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

static const uint qt_meta_data_PreprocessComboBox[] = {

 // content:
       1,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets

       0        // eod
};

static const char qt_meta_stringdata_PreprocessComboBox[] = {
    "PreprocessComboBox\0"
};

const QMetaObject PreprocessComboBox::staticMetaObject = {
    { &QComboBox::staticMetaObject, qt_meta_stringdata_PreprocessComboBox,
      qt_meta_data_PreprocessComboBox, 0 }
};

const QMetaObject *PreprocessComboBox::metaObject() const
{
    return &staticMetaObject;
}

void *PreprocessComboBox::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PreprocessComboBox))
	return static_cast<void*>(const_cast< PreprocessComboBox*>(this));
    return QComboBox::qt_metacast(_clname);
}

int PreprocessComboBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QComboBox::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_PreprocessWidget[] = {

 // content:
       1,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   10, // methods
       0,    0, // properties
       0,    0, // enums/sets

 // slots: signature, parameters, type, tag, flags
      18,   17,   17,   17, 0x08,
      44,   17,   17,   17, 0x08,
      58,   17,   17,   17, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PreprocessWidget[] = {
    "PreprocessWidget\0\0fileBrowseButtonClicked()\0"
    "updateImage()\0loadImage()\0"
};

const QMetaObject PreprocessWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PreprocessWidget,
      qt_meta_data_PreprocessWidget, 0 }
};

const QMetaObject *PreprocessWidget::metaObject() const
{
    return &staticMetaObject;
}

void *PreprocessWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PreprocessWidget))
	return static_cast<void*>(const_cast< PreprocessWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PreprocessWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: fileBrowseButtonClicked(); break;
        case 1: updateImage(); break;
        case 2: loadImage(); break;
        }
        _id -= 3;
    }
    return _id;
}
