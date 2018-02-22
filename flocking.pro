TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


# Input
SOURCES += src/main.cpp \
           src/boid.cpp \
    src/prey.cpp


HEADERS += include/boid.h \
    include/prey.h
