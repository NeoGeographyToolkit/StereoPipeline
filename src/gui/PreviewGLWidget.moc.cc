/****************************************************************************
** Meta object code from reading C++ file 'PreviewGLWidget.h'
**
** Created: Sun Sep 21 16:35:18 2008
**      by: The Qt Meta Object Compiler version 59 (Qt 4.3.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "PreviewGLWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PreviewGLWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 59
#error "This file was generated using the moc from 4.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

static const uint qt_meta_data_PreviewGLWidget[] = {

 // content:
       1,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   10, // methods
       0,    0, // properties
       0,    0, // enums/sets

 // slots: signature, parameters, type, tag, flags
      26,   17,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PreviewGLWidget[] = {
    "PreviewGLWidget\0\0filename\0"
    "load_image_from_file(std::string)\0"
};

const QMetaObject PreviewGLWidget::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_PreviewGLWidget,
      qt_meta_data_PreviewGLWidget, 0 }
};

const QMetaObject *PreviewGLWidget::metaObject() const
{
    return &staticMetaObject;
}

void *PreviewGLWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PreviewGLWidget))
	return static_cast<void*>(const_cast< PreviewGLWidget*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int PreviewGLWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: load_image_from_file((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        }
        _id -= 1;
    }
    return _id;
}
