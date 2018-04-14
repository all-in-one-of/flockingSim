#TEMPLATE=app

#include(../common.pri)

##LIB_INSTALL_DIR=../lib
##BIN_INSTALL_DIR=../bin
##INC_INSTALL_DIR=../include


## This specifies the exe name
#TARGET=$$BIN_INSTALL_DIR/FlockingSim
## where to put the .o files
#OBJECTS_DIR=obj
## core Qt Libs to use add more here if needed.
#QT+=gui opengl core


#INCLUDEPATH += $$INC_INSTALL_DIR

## as I want to support 4.8 and 5 this will set a flag for some of the mac stuff
## mainly in the types.h file for the setMacVisual which is native in Qt5
#isEqual(QT_MAJOR_VERSION, 5) {
#	cache()
#	DEFINES +=QT5BUILD
#}
## where to put moc auto generated files
#MOC_DIR=moc
## on a mac we don't create a .app bundle file ( for ease of multiplatform use)
#CONFIG-=app_bundle
#SOURCES+= $$PWD/src/*.cpp \

## same for the .h files
#HEADERS+= $$PWD/include/*.h \


## and add the include dir into the search path for Qt and make
#INCLUDEPATH +=./include \
#               $$PWD/../../glm \
#               $$PWD/ui \
## where our exe is going to live (root of project)
#DESTDIR=./
## add the glsl shader files
#OTHER_FILES+= shaders/*.glsl \
#              models/*

#FORMS += ui/mainwindow.ui

#UI_HEADERS_DIR = ui
#OBJECTS_DIR = obj
#MOC_DIR = moc
#UI_DIR = ui
#							README.md
## were are going to default to a console app
#CONFIG += console
## note each command you add needs a ; as it will be run as a single line
## first check if we are shadow building or not easiest way is to check out against current
#!equals(PWD, $${OUT_PWD}){
#	copydata.commands = echo "creating destination dirs" ;
#	# now make a dir
#	copydata.commands += mkdir -p $$OUT_PWD/shaders ;
#	copydata.commands += echo "copying files" ;
#	# then copy the files
#	copydata.commands += $(COPY_DIR) $$PWD/shaders/* $$OUT_PWD/shaders/ ;
#	# now make sure the first target is built before copy
#	first.depends = $(first) copydata
#	export(first.depends)
#	export(copydata.commands)
#	# now add it as an extra target
#	QMAKE_EXTRA_TARGETS += first copydata
#}
#NGLPATH=$$(NGLDIR)
#isEmpty(NGLPATH){ # note brace must be here
#	message("including $HOME/NGL")
#	include($(HOME)/NGL/UseNGL.pri)
#}
#else{ # note brace must be here
#	message("Using custom NGL location")
#	include($(NGLDIR)/UseNGL.pri)
#}

#LIBS += -L$$LIB_INSTALL_DIR -lflock_cpu -lProject -lGL -lGLU -lGLEW #-lflock_gpu

#QMAKE_RPATHDIR += $$LIB_INSTALL_DIR


include(../common.pri)

TEMPLATE=app
TARGET=$$BIN_INSTALL_DIR/flockingSimTest

SOURCES += src/*.cpp

INCLUDEPATH += $$INC_INSTALL_DIR

OBJECTS_DIR = obj

QMAKE_CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic

LIBS += -L$$LIB_INSTALL_DIR -llibflock_cpu -lflock_gpu #-lrand_gpu

QMAKE_RPATHDIR += $$LIB_INSTALL_DIR
