QT+=widgets

INCLUDEPATH += $$PWD\include

HEADERS += \
    $$PWD/include/rmatrix.h\
    $$PWD/include/rvector.h

SOURCES += \
    $$PWD/src/rmatrix.cpp\
    $$PWD/src/rvector.cpp \
    $$PWD/src/svd.cpp
