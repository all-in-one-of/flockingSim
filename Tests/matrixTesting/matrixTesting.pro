TARGET = app.bin

CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIB_INSTALL_DIR=../../lib


INCLUDEPATH+=../../include
DEPENDPATH+=../../include

#LIBS+= -lgtest \
#        -L../../lib -lflock_gpu -lflock_cpu

LIBS+= -L/usr/local/lib -lgtest -lpthread \
       -L/public/devel/lib/ -lgtest \
       -L$$LIB_INSTALL_DIR -lflock_gpu -lflock_cpu \

QMAKE_RPATHDIR+=../../lib

SOURCES += \
    matrixTesting.cpp


OTHER_FILES+=$$PWD/app
