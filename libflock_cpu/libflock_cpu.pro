#-------------------------------------------------
#
# Project created by QtCreator 2017-09-28T14:53:36
#
#-------------------------------------------------
# Read shared environment settings from the common include file
include(../common.pri)
QT += core gui
CONFIG += console c++11

TEMPLATE = lib
TARGET = $$LIB_INSTALL_DIR/libflock_cpu

QT += opengl \
      core \
      gui

CONFIG += console \
          c++11

CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/include \
               $$PWD/ui \
               $$PWD/../../glm \
               $$PWD/shaders

HEADERS += include/Boid.h \
           include/flock_cpu.h \
           include/Buffer.h \
           include/Camera.h \
           include/Flock.h \
           include/GLWindow.h \
           include/MainWindow.h \
           include/Mesh.h \
           include/prey.h \
           include/Scene.h \
           include/Shader.h \
           include/TrackballCamera.h \


SOURCES += src/Boid.cpp \
           src/Buffer.cpp \
           src/Camera.cpp \
           src/Flock.cpp \
           src/flock_cpu.cpp \
           src/GLWindow.cpp \
           src/MainWindow.cpp \
           src/Mesh.cpp \
           src/prey.cpp \
           src/Scene.cpp \
           src/Shader.cpp \
           src/TrackballCamera.cpp \

OTHER_FILES += shaders/* \
               models/*

FORMS += ui/mainwindow.ui

UI_HEADERS_DIR = ui
OBJECTS_DIR = obj
MOC_DIR = moc
UI_DIR = ui

# Set up the include path
INCLUDEPATH += include

linux:LIBS += -lGL -lGLU -lGLEW

# Set up the post install script to copy the headers into the appropriate directory
includeinstall.commands = mkdir -p $$INC_INSTALL_DIR && cp include/*.h $$INC_INSTALL_DIR
QMAKE_EXTRA_TARGETS += includeinstall
POST_TARGETDEPS += includeinstall


#DISTFILES +=
