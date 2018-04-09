#TARGET = Test

#CONFIG += console c++11
#CONFIG -= app_bundle
#CONFIG -= qt

#LIB_INSTALL_DIR=../lib


#LIBS+= -L/usr/local/lib -lgtest -lpthread \
#       -L/public/devel/lib/ -lgtest \
#       -L$$LIB_INSTALL_DIR -lflock_gpu -lflock_cpu \

#SOURCES += \
#    main.cpp

#QMAKE_RPATHDIR+=../lib





include(../common.pri)

TARGET = gtest
CONFIG += console c++11
CONFIG -= app_bundle
QT += core

OTHER_FILES += Common

#OBJECTS_DIR = $$PWD/obj

INCLUDEPATH+= include /usr/local/include /public/devel/include

LIBS+= -L/usr/local/lib -lgtest -lpthread \
       -L/public/devel/lib/ -lgtest \
       -L$$LIB_INSTALL_DIR -lflock_gpu -lflock_cpu

INCLUDEPATH+= $$PWD/../ \
              $$INC_INSTALL_DIR \

macx:CONFIG-=app_bundle

QMAKE_RPATHDIR += $$LIB_INSTALL_DIR

#HEADERS += include/constructor.h \

SOURCES +=$$PWD/*.cpp

NGLPATH=$$(NGLDIR)
isEmpty(NGLPATH){ # note brace must be here
        message("including $HOME/NGL")
        include($(HOME)/NGL/UseNGL.pri)
}
else{ # note brace must be here
        message("Using custom NGL location")
        include($(NGLDIR)/UseNGL.pri)
}


