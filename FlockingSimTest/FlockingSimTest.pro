

include(../common.pri)

TEMPLATE=app
TARGET=$$BIN_INSTALL_DIR/flockingSimTest

SOURCES += src/main.cpp

INCLUDEPATH += $$INC_INSTALL_DIR \
               $$PWD/../../glm \

OBJECTS_DIR = obj

QMAKE_CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic

LIBS += -L$$LIB_INSTALL_DIR -llibflock_cpu -lflock_gpu #-lrand_gpu

QMAKE_RPATHDIR += $$LIB_INSTALL_DIR
