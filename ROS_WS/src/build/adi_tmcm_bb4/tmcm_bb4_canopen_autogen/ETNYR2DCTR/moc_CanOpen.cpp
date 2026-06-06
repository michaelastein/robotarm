/****************************************************************************
** Meta object code from reading C++ file 'CanOpen.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../adi_tmcm_bb4/include/adi_tmcm_bb4/CanOpen.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CanOpen.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CanOpen_t {
    QByteArrayData data[15];
    char stringdata0[225];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CanOpen_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CanOpen_t qt_meta_stringdata_CanOpen = {
    {
QT_MOC_LITERAL(0, 0, 7), // "CanOpen"
QT_MOC_LITERAL(1, 8, 23), // "connectionStatusChanged"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 7), // "aStatus"
QT_MOC_LITERAL(4, 41, 20), // "connectionStrChanged"
QT_MOC_LITERAL(5, 62, 7), // "aString"
QT_MOC_LITERAL(6, 70, 13), // "receivedFrame"
QT_MOC_LITERAL(7, 84, 16), // "statusStrChanged"
QT_MOC_LITERAL(8, 101, 13), // "processErrors"
QT_MOC_LITERAL(9, 115, 26), // "QCanBusDevice::CanBusError"
QT_MOC_LITERAL(10, 142, 6), // "aError"
QT_MOC_LITERAL(11, 149, 15), // "processRxFrames"
QT_MOC_LITERAL(12, 165, 19), // "processStateChanged"
QT_MOC_LITERAL(13, 185, 32), // "QCanBusDevice::CanBusDeviceState"
QT_MOC_LITERAL(14, 218, 6) // "aState"

    },
    "CanOpen\0connectionStatusChanged\0\0"
    "aStatus\0connectionStrChanged\0aString\0"
    "receivedFrame\0statusStrChanged\0"
    "processErrors\0QCanBusDevice::CanBusError\0"
    "aError\0processRxFrames\0processStateChanged\0"
    "QCanBusDevice::CanBusDeviceState\0"
    "aState"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CanOpen[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       4,    1,   52,    2, 0x06 /* Public */,
       6,    1,   55,    2, 0x06 /* Public */,
       7,    1,   58,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,   61,    2, 0x08 /* Private */,
      11,    0,   64,    2, 0x08 /* Private */,
      12,    1,   65,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    5,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 13,   14,

       0        // eod
};

void CanOpen::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CanOpen *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectionStatusChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->connectionStrChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->receivedFrame((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->statusStrChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->processErrors((*reinterpret_cast< QCanBusDevice::CanBusError(*)>(_a[1]))); break;
        case 5: _t->processRxFrames(); break;
        case 6: _t->processStateChanged((*reinterpret_cast< QCanBusDevice::CanBusDeviceState(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CanOpen::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanOpen::connectionStatusChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CanOpen::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanOpen::connectionStrChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CanOpen::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanOpen::receivedFrame)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (CanOpen::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanOpen::statusStrChanged)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CanOpen::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CanOpen.data,
    qt_meta_data_CanOpen,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CanOpen::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CanOpen::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CanOpen.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int CanOpen::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void CanOpen::connectionStatusChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CanOpen::connectionStrChanged(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CanOpen::receivedFrame(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CanOpen::statusStrChanged(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
