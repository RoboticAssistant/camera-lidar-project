/****************************************************************************
** Meta object code from reading C++ file 'TcpServerPool.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../find-object/app/TcpServerPool.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TcpServerPool.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_FindObjectWorker_t {
    QByteArrayData data[11];
    char stringdata0[132];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FindObjectWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FindObjectWorker_t qt_meta_stringdata_FindObjectWorker = {
    {
QT_MOC_LITERAL(0, 0, 16), // "FindObjectWorker"
QT_MOC_LITERAL(1, 17, 12), // "objectsFound"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 26), // "find_object::DetectionInfo"
QT_MOC_LITERAL(4, 58, 6), // "detect"
QT_MOC_LITERAL(5, 65, 7), // "cv::Mat"
QT_MOC_LITERAL(6, 73, 5), // "image"
QT_MOC_LITERAL(7, 79, 18), // "addObjectAndUpdate"
QT_MOC_LITERAL(8, 98, 2), // "id"
QT_MOC_LITERAL(9, 101, 8), // "filePath"
QT_MOC_LITERAL(10, 110, 21) // "removeObjectAndUpdate"

    },
    "FindObjectWorker\0objectsFound\0\0"
    "find_object::DetectionInfo\0detect\0"
    "cv::Mat\0image\0addObjectAndUpdate\0id\0"
    "filePath\0removeObjectAndUpdate"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FindObjectWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   37,    2, 0x0a /* Public */,
       7,    3,   40,    2, 0x0a /* Public */,
      10,    1,   47,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5, QMetaType::Int, QMetaType::QString,    6,    8,    9,
    QMetaType::Void, QMetaType::Int,    8,

       0        // eod
};

void FindObjectWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FindObjectWorker *_t = static_cast<FindObjectWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->objectsFound((*reinterpret_cast< const find_object::DetectionInfo(*)>(_a[1]))); break;
        case 1: _t->detect((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 2: _t->addObjectAndUpdate((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
        case 3: _t->removeObjectAndUpdate((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (FindObjectWorker::*_t)(const find_object::DetectionInfo & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&FindObjectWorker::objectsFound)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject FindObjectWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_FindObjectWorker.data,
      qt_meta_data_FindObjectWorker,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *FindObjectWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FindObjectWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_FindObjectWorker.stringdata0))
        return static_cast<void*>(const_cast< FindObjectWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int FindObjectWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void FindObjectWorker::objectsFound(const find_object::DetectionInfo & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_TcpServerPool_t {
    QByteArrayData data[1];
    char stringdata0[14];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TcpServerPool_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TcpServerPool_t qt_meta_stringdata_TcpServerPool = {
    {
QT_MOC_LITERAL(0, 0, 13) // "TcpServerPool"

    },
    "TcpServerPool"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TcpServerPool[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void TcpServerPool::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject TcpServerPool::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TcpServerPool.data,
      qt_meta_data_TcpServerPool,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TcpServerPool::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TcpServerPool::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TcpServerPool.stringdata0))
        return static_cast<void*>(const_cast< TcpServerPool*>(this));
    return QObject::qt_metacast(_clname);
}

int TcpServerPool::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
