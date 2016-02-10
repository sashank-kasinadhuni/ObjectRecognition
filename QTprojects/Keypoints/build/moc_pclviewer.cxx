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
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x0a,
      45,   10,   10,   10, 0x0a,
      79,   10,   10,   10, 0x0a,
     106,   10,   10,   10, 0x0a,
     141,  135,   10,   10, 0x0a,
     171,  135,   10,   10, 0x0a,
     201,  135,   10,   10, 0x0a,
     229,  135,   10,   10, 0x0a,
     257,  135,   10,   10, 0x0a,
     287,  135,   10,   10, 0x0a,
     320,  135,   10,   10, 0x0a,
     352,  135,   10,   10, 0x0a,
     384,  135,   10,   10, 0x0a,
     418,  135,   10,   10, 0x0a,
     451,  135,   10,   10, 0x0a,
     484,  135,   10,   10, 0x0a,
     515,  135,   10,   10, 0x0a,
     548,   10,   10,   10, 0x0a,
     568,   10,   10,   10, 0x0a,
     596,  586,   10,   10, 0x0a,
     626,  135,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PCLViewer[] = {
    "PCLViewer\0\0CalculateKeypointsButtonPressed()\0"
    "MultiplyResolutionButtonPressed()\0"
    "NonMultiplyButtonPressed()\0"
    "DefaultValuesButtonPressed()\0value\0"
    "CubeEdgeLengthChanged(double)\0"
    "CubeEdgePointsChanged(double)\0"
    "gamma21ValueChanged(double)\0"
    "gamma32ValueChanged(double)\0"
    "nonMaxSuppMultChanged(double)\0"
    "salientRadiusMultChanged(double)\0"
    "normalRadiusMultChanged(double)\0"
    "borderRadiusMultChanged(double)\0"
    "salientRadiusValueChanged(double)\0"
    "normalRadiusValueChanged(double)\0"
    "borderRadiusValueChanged(double)\0"
    "nonMaxSuppValueChanged(double)\0"
    "minNeighborsValueChanged(double)\0"
    "ParametersChanged()\0DownSampleModel()\0"
    "mfilename\0modelFileNameChanged(QString)\0"
    "leafSizeValueChanged(double)\0"
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        switch (_id) {
        case 0: _t->CalculateKeypointsButtonPressed(); break;
        case 1: _t->MultiplyResolutionButtonPressed(); break;
        case 2: _t->NonMultiplyButtonPressed(); break;
        case 3: _t->DefaultValuesButtonPressed(); break;
        case 4: _t->CubeEdgeLengthChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->CubeEdgePointsChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->gamma21ValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->gamma32ValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->nonMaxSuppMultChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->salientRadiusMultChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->normalRadiusMultChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->borderRadiusMultChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->salientRadiusValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->normalRadiusValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: _t->borderRadiusValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 15: _t->nonMaxSuppValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->minNeighborsValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 17: _t->ParametersChanged(); break;
        case 18: _t->DownSampleModel(); break;
        case 19: _t->modelFileNameChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 20: _t->leafSizeValueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
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
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
