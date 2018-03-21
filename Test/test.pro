TARGET = Test

CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIB_INSTALL_DIR=../lib


LIBS+= -L/usr/local/lib -lgtest -lpthread \
       -L/public/devel/lib/ -lgtest \
       -L$$LIB_INSTALL_DIR -lflock_gpu -lflock_cpu \

SOURCES += \
    test.cpp

QMAKE_RPATHDIR+=../lib
