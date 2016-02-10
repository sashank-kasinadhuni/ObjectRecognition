/****************************************************************************
** Meta object code from reading C++ file 'pclviewer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/pclviewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PCLViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x0a,
      38,   10,   10,   10, 0x0a,
      58,   10,   10,   10, 0x0a,
      83,   77,   10,   10, 0x0a,
     113,   77,   10,   10, 0x0a,
     143,   77,   10,   10, 0x0a,
     173,   77,   10,   10, 0x0a,
     197,   77,   10,   10, 0x0a,
     220,   77,   10,   10, 0x0a,
     242,   77,   10,   10, 0x0a,
     267,   77,   10,   10, 0x0a,
     290,   77,   10,   10, 0x0a,
     312,   77,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PCLViewer[] = {
    "PCLViewer\0\0RandomColorButtonPressed()\0"
    "SaveButtonPressed()\0XYZSliderChanged()\0"
    "value\0xAngleSliderValueChanged(int)\0"
    "yAngleSliderValueChanged(int)\0"
    "zAngleSliderValueChanged(int)\0"
    "x_right_changed(double)\0x_left_changed(double)\0"
    "y_top_changed(double)\0y_bottom_changed(double)\0"
    "z_near_changed(double)\0z_far_changed(double)\0"
    "pSliderValueChanged(int)\0"
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        switch (_id) {
        case 0: _t->RandomColorButtonPressed(); break;
        case 1: _t->SaveButtonPressed(); break;
        case 2: _t->XYZSliderChanged(); break;
        case 3: _t->xAngleSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->yAngleSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->zAngleSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->x_right_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->x_left_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->y_top_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->y_bottom_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->z_near_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->z_far_changed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->pSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PCLViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PCLViewer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PCLViewer,
      qt_meta_data_PCLViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PCLViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PCLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PCLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PCLViewer))
        return static_cast<void*>(const_cast< PCLViewer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int PCLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
