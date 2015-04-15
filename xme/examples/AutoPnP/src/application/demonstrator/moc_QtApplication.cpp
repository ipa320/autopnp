/****************************************************************************
** Meta object code from reading C++ file 'QtApplication.h'
**
** Created: Mon Jun 2 15:59:28 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../xme/ports/software/os/generic-os/qt/generic/xme/hal/QtApplication.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtApplication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtApplication[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      33,   15,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
      76,   15,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtApplication[] = {
    "QtApplication\0\0callback,userData\0"
    "executeSignal(xme_hal_qt_callback_t,void*)\0"
    "execute(xme_hal_qt_callback_t,void*)\0"
};

void QtApplication::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtApplication *_t = static_cast<QtApplication *>(_o);
        switch (_id) {
        case 0: _t->executeSignal((*reinterpret_cast< xme_hal_qt_callback_t(*)>(_a[1])),(*reinterpret_cast< void*(*)>(_a[2]))); break;
        case 1: _t->execute((*reinterpret_cast< xme_hal_qt_callback_t(*)>(_a[1])),(*reinterpret_cast< void*(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtApplication::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtApplication::staticMetaObject = {
    { &QApplication::staticMetaObject, qt_meta_stringdata_QtApplication,
      qt_meta_data_QtApplication, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtApplication::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtApplication::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtApplication::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtApplication))
        return static_cast<void*>(const_cast< QtApplication*>(this));
    return QApplication::qt_metacast(_clname);
}

int QtApplication::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QApplication::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void QtApplication::executeSignal(xme_hal_qt_callback_t _t1, void * _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
